"""AgiBot G01（智元精灵 G01）硬件适配器。

基于智元 GDK Python SDK（``a2d_sdk``）的 ``RobotDds`` / ``CosineCamera`` /
``RobotController`` 接口实现
:class:`~cloudrobo_r2c.core.interfaces.IRobotHardwareAdapter`，接口依据
《智元精灵G01+GDK v1.5.0使用指南》第 7 章（RobotDds）、第 8 章（RobotController）
与第 9 章（CosineCamera）。

覆盖部位：
- 双臂 14 关节（左 7 + 右 7，弧度）
- 夹持器 2 通道（``[0, 1]`` 归一化或 ``[35, 120]`` 原始值，任意角度连续透传）
- 灵巧手 12 关节（弧度）
- 头部 ``[yaw, pitch]``（弧度）、腰部 ``[pitch(弧度), height(cm)]``
- 轮式底盘 ``move_wheel(linear, angular)``
- 末端绝对位姿 ``set_end_effector_pose_control``（第 8 章，7 维位姿）
- 相机通道（头部 / 左右手腕等，``np.ndarray`` HWC）

观测平铺字段（供 ``ConfigurableDeviceTranslator`` 映射到 R2C 标准格式）：

- ``joint_positions``       — 双臂 14 关节位置（弧度）
- ``gripper_positions``     — 夹持器 2 通道
- ``hand_joint_positions``  — 灵巧手 12 关节
- ``head_joint_positions``  — 头部 ``[yaw, pitch]``
- ``waist_joint_positions`` — 腰部 ``[pitch, height_cm]``
- ``left_ee_pose`` / ``right_ee_pose`` — 左/右末端位姿 ``[x, y, z, qx, qy, qz, qw]``
  （来自 ``get_motion_status().frames``，未就绪时不产出）
- ``body_pose``             — 整体姿态关节状态（原样透传，float32 数组）
- 相机图像按 ``camera_key_map`` 映射后以 R2C 键暴露（默认
  ``cam_high`` / ``cam_left_wrist`` / ``cam_right_wrist``）

动作平铺字段（由 ``r2c_to_device`` 映射产生，缺省字段自动跳过）：

- ``arm_cmd_joint_pos``     — 双臂 14 关节目标（弧度）→ ``move_arm``
- ``gripper_cmd_pos``       — 夹持器 2 通道目标（任意角度）→ ``move_gripper``
- ``hand_cmd_joint_pos``    — 灵巧手 12 关节目标 → ``move_hand``
- ``head_cmd_joint_pos``    — 头部 ``[yaw, pitch]`` 目标 → ``move_head``
- ``waist_cmd_joint_pos``   — 腰部 ``[pitch, height_cm]`` 目标 → ``move_waist``
- ``wheel_cmd_speed``       — 底盘 ``[linear, angular]`` → ``move_wheel``
- ``left_pose_cmd`` / ``right_pose_cmd`` — 左/右末端绝对位姿 7 维目标
  → ``set_end_effector_pose_control``
- ``motion_stop_cmd``       — 真值触发全身急停 → ``set_motion_stop``

动作字段支持 ``ExtensionValue``（FLOAT32/FLOAT64，形状一维），云端可将头/腰/轮
等扩展指令放进 ``Actions.extensions``，由适配器负责解码。

《指南》注意事项：
- 所有的角度输入均使用弧度制；腰部高度单位为厘米
- 手臂目标关节角与实时关节角差值需小于 0.2618 rad（15°），角速度限制 3 rad/s
  （见《指南》5.1，由上层规划保证）
- 第 8.2.1 节 ``ABS_POSE`` 在 trajectory 轨迹接口中标注"当前版本暂不支持"；
  适配器末端位姿走 8.1 节 ``set_end_effector_pose_control`` 绝对位姿接口，
  现场可用性以 GDK 实际版本为准

YAML 配置示例见 ``config/robot_agibot_g01_config.yaml``，
样例云端循环动作用 ``examples/agibot_g01_cloud_adapter.py``。
"""

from __future__ import annotations

import logging
import time
from typing import Any, Dict, List, Mapping, Optional

import numpy as np

from cloudrobo_r2c.core.interfaces import IRobotHardwareAdapter
from cloudrobo_r2c.robots.agibot_g01.agibot_g01_client import (
    DEFAULT_CAMERA_GROUP,
    DEFAULT_CAMERA_KEY_MAP,
    AgiBotG01Client,
)
from cloudrobo_r2c.robots.commands.base import AdapterCommand

logger = logging.getLogger(__name__)

# G01 关节维度（《指南》7.1）
ARM_DOF = 14
GRIPPER_DOF = 2
HAND_DOF = 12
HEAD_DOF = 2
WAIST_DOF = 2
# 末端位姿维度 [x, y, z, qx, qy, qz, qw]
POSE_DOF = 7

# 适配器观测/动作字段的期望维度（超长截断、不足补零前先告警）
EXPECTED_DIMS: Dict[str, int] = {
    "joint_positions": ARM_DOF,
    "gripper_positions": GRIPPER_DOF,
    "hand_joint_positions": HAND_DOF,
    "head_joint_positions": HEAD_DOF,
    "waist_joint_positions": WAIST_DOF,
    "arm_cmd_joint_pos": ARM_DOF,
    "gripper_cmd_pos": GRIPPER_DOF,
    "hand_cmd_joint_pos": HAND_DOF,
    "head_cmd_joint_pos": HEAD_DOF,
    "waist_cmd_joint_pos": WAIST_DOF,
    "wheel_cmd_speed": 2,
    "left_pose_cmd": POSE_DOF,
    "right_pose_cmd": POSE_DOF,
}

# 复位指令（go_home）各部位键的期望维度（与动作键名不同，单独维护）
RESET_EXPECTED_DIMS: Dict[str, int] = {
    "arm_positions": ARM_DOF,
    "gripper_positions": GRIPPER_DOF,
    "hand_positions": HAND_DOF,
    "head_positions": HEAD_DOF,
    "waist_positions": WAIST_DOF,
}

# 头/腰/手/整体姿态等扩展字段使用 float32，与 extension 映射的 dtype 声明一致
_EXT_FLOAT_DTYPES = {"FLOAT32": "<f4", "FLOAT64": "<f8"}


def create_agibot_g01_adapter(
    config: Mapping[str, Any], **extra_kwargs: Any
) -> IRobotHardwareAdapter:
    """Entry_point factory for AgiBotG01HardwareAdapter."""
    return AgiBotG01HardwareAdapter(config=dict(config))


class AgiBotG01ResetCommand(AdapterCommand):
    """复位指令：通过 GDK ``reset()`` 平滑回到初始或指定部位位姿。

    YAML ``commands.go_home`` 支持（均可选，未配置的部位不复位）：

    - ``arm_positions``      双臂 14 关节
    - ``gripper_positions``  夹持器 2 通道
    - ``hand_positions``     灵巧手 12 关节
    - ``head_positions``     头部 ``[yaw, pitch]``
    - ``waist_positions``    腰部 ``[pitch, height_cm]``
    """

    def execute(self, **kwargs: Any) -> None:
        targets: Dict[str, List[float]] = {}
        for key in RESET_EXPECTED_DIMS:
            value = kwargs.get(key, self.config.get(key))
            if value is None:
                continue
            values = AgiBotG01HardwareAdapter.as_float_list(value)
            expected = RESET_EXPECTED_DIMS[key]
            if len(values) != expected:
                raise ValueError(f"go_home.{key} expects {expected} values, got {len(values)}")
            targets[key] = values
        client = self.adapter._require_client()
        logger.info("[AgiBotG01] reset targets=%s", sorted(targets))
        client.reset(**targets)


class AgiBotG01HardwareAdapter(IRobotHardwareAdapter):
    """智元精灵 G01 硬件适配器（GDK RobotDds + CosineCamera + RobotController）。

    用法（ ``config/robot_agibot_g01_config.yaml``）：

    - ``hardware.type: "agibot_g01"``（entry point ``r2c_sdk.adapters``）
    - ``hardware.config`` 支持的键：
        - ``init_wait_s``             RobotDds 初始化等待秒数（默认 0.5）
        - ``camera_enabled``          是否启用相机（默认 true）
        - ``camera_group``            启用的 G01 相机通道列表
        - ``camera_key_map``          G01 通道名 → R2C 图像键
        - ``camera_poll_interval_s``  相机取流线程轮询间隔（默认 0.03s ≈ 30Hz）
        - ``enable_pose_control``     是否启用 RobotController（《指南》8 章，默认 true；
                                      失败时自动降级为仅关节控制）
        - ``pose_lifetime_s``         末端位姿指令有效时间（默认 1.0s）
        - ``pose_control_group``      位姿控制部位（默认 ["left_arm", "right_arm"]）
        - ``dry_run``                 适配器级干运行：只打印动作不执行
        - ``commands``                如 ``go_home`` 复位指令
    """

    # RobotController.get_motion_status() frames 中左右末端的 link 键
    _LEFT_EE_FRAME = "arm_left_link7"
    _RIGHT_EE_FRAME = "arm_right_link7"

    def __init__(self, config: Optional[Mapping[str, Any]] = None) -> None:
        self._config: Dict[str, Any] = dict(config or {})
        self._dry_run = bool(self._config.get("dry_run", False))
        self._init_wait_s = float(self._config.get("init_wait_s", 0.5))
        self._camera_enabled = bool(self._config.get("camera_enabled", True))
        self._camera_group = list(self._config.get("camera_group", DEFAULT_CAMERA_GROUP))
        camera_key_map = self._config.get("camera_key_map")
        self._camera_key_map: Dict[str, str] = dict(
            camera_key_map if camera_key_map else DEFAULT_CAMERA_KEY_MAP
        )
        self._camera_poll_interval_s = float(self._config.get("camera_poll_interval_s", 0.03))
        self._enable_pose_control = bool(self._config.get("enable_pose_control", True))
        self._pose_lifetime_s = float(self._config.get("pose_lifetime_s", 1.0))
        self._pose_control_group = list(
            self._config.get("pose_control_group", ["left_arm", "right_arm"])
        )

        self._client: Optional[AgiBotG01Client] = None
        self._connected = False
        self._sequence = 0
        self._last_observation: Dict[str, Any] = {}

        if self._dry_run:
            logger.info("[AgiBotG01] DRY_RUN MODE: log actions only, no hardware calls")
        # 复位指令（keyboard_control / 命令系统可触发）
        self.register_command_class("go_home", AgiBotG01ResetCommand)

    # ------------------------------------------------------------------
    # 生命周期
    # ------------------------------------------------------------------

    def connect(self) -> None:
        """连接 GDK：创建 ``RobotDds`` 与（可选）``CosineCamera``。"""
        if self._connected:
            logger.debug("[AgiBotG01] already connected, skipping connect()")
            return
        client = AgiBotG01Client(
            camera_group=self._camera_group,
            camera_key_map=self._camera_key_map,
            camera_enabled=self._camera_enabled,
            init_wait_s=self._init_wait_s,
            camera_poll_interval_s=self._camera_poll_interval_s,
            enable_motion_controller=self._enable_pose_control,
        )
        try:
            client.connect()
        except Exception:
            # 回滚已分配的资源（如相机线程），避免半连接状态
            try:
                client.close()
            except Exception:
                pass
            raise
        self._client = client
        self._connected = True
        logger.info(
            "[AgiBotG01] adapter connected (dry_run=%s, cameras=%s)",
            self._dry_run,
            self._camera_group if self._camera_enabled else [],
        )

    def disconnect(self) -> None:
        """断开连接：释放相机与 DDS 资源。"""
        if not self._connected:
            logger.debug("[AgiBotG01] already disconnected, skipping disconnect()")
            return
        if self._client is not None:
            try:
                self._client.close()
            except Exception as exc:
                logger.warning("[AgiBotG01] error during client close: %s", exc)
            self._client = None
        self._connected = False
        logger.info("[AgiBotG01] adapter disconnected")

    @property
    def is_connected(self) -> bool:
        return self._connected

    def _require_client(self) -> AgiBotG01Client:
        if not self._connected or self._client is None:
            raise RuntimeError("AgiBot G01 adapter is not connected. Call connect() first.")
        return self._client

    # ------------------------------------------------------------------
    # 观测
    # ------------------------------------------------------------------

    def get_observation(self) -> Mapping[str, Any]:
        """采集一帧观测（平铺字段，见模块 docstring）。

        单个状态查询失败时记录告警并沿用上一次值（或零值），保证控制循环不被
        单次 DDS 异常打断。相机无帧的通道不产出字段（translator 对缺失的
        ``required: false`` 映射自动跳过）。
        """
        client = self._require_client()
        self._sequence += 1
        observation: Dict[str, Any] = {
            "timestamp": int(time.time() * 1000),
            "id": self._sequence,
        }

        self._fill_state(observation, "joint_positions", client.arm_joint_states, ARM_DOF)
        self._fill_state(observation, "gripper_positions", client.gripper_states, GRIPPER_DOF)
        # 头/腰/手经 extensions 以 FLOAT32 打包，观测数组统一使用 float32
        self._fill_state(
            observation,
            "hand_joint_positions",
            client.hand_joint_states,
            HAND_DOF,
            dtype=np.float32,
        )
        self._fill_state(
            observation,
            "head_joint_positions",
            client.head_joint_states,
            HEAD_DOF,
            dtype=np.float32,
        )
        self._fill_state(
            observation,
            "waist_joint_positions",
            client.waist_joint_states,
            WAIST_DOF,
            dtype=np.float32,
        )
        self._fill_body_pose(observation, client.body_pose_joint_states)
        self._fill_ee_poses(observation, client)

        # 相机图像：以 R2C 键（camera_key_map 映射）暴露
        for g01_name, image in client.get_camera_images().items():
            key = self._camera_key_map.get(g01_name, g01_name)
            observation[key] = image

        self._last_observation = observation
        return observation

    def _fill_state(
        self,
        observation: Dict[str, Any],
        key: str,
        getter: Any,
        size: int,
        dtype: type = np.float64,
    ) -> None:
        """读取 ``(数据, 时间戳)`` 型状态并写入观测，失败时回退上次值/零值。"""
        values = self._safe_state(getter)
        if values is None:
            cached = self._last_observation.get(key)
            observation[key] = cached if cached is not None else np.zeros(size, dtype=dtype)
            return
        observation[key] = self._normalize_dim(values, key, size).astype(dtype)

    def _fill_body_pose(self, observation: Dict[str, Any], getter: Any) -> None:
        """整体姿态关节状态（《指南》7.1.1，返回 list 无时间戳）。"""
        try:
            raw = getter()
        except Exception as exc:
            logger.warning("[AgiBotG01] body_pose_joint_states failed: %s", exc)
            raw = None
        if isinstance(raw, tuple):
            # 防御：签名标注为 list，若实现返回 (数据, 时间戳) 取数据部分
            raw = raw[0] if raw else None
        if raw is None:
            cached = self._last_observation.get("body_pose")
            observation["body_pose"] = (
                cached if cached is not None else np.zeros(0, dtype=np.float32)
            )
            return
        arr = np.asarray(raw, dtype=np.float64).reshape(-1)
        # float32 与 extensions 映射的 dtype 声明一致（tobytes 打包）
        observation["body_pose"] = arr.astype(np.float32)

    def _fill_ee_poses(self, observation: Dict[str, Any], client: AgiBotG01Client) -> None:
        """末端位姿观测（《指南》8.2.3 ``get_motion_status().frames``）。

        产出 ``left_ee_pose`` / ``right_ee_pose``（``[x, y, z, qx, qy, qz, qw]`` list）
        与合并字段 ``ee_poses``（按 left、right 顺序的位姿列表，供映射到 R2C
        ``end_effector_poses.pose``）；运控状态未就绪时不产出字段。
        """
        try:
            status = client.get_motion_status()
        except Exception as exc:
            logger.warning("[AgiBotG01] get_motion_status failed: %s", exc)
            return
        if not isinstance(status, Mapping):
            return
        frames = status.get("frames") or {}
        left = self._frame_to_pose7d(frames.get(self._LEFT_EE_FRAME))
        right = self._frame_to_pose7d(frames.get(self._RIGHT_EE_FRAME))
        if left is not None:
            observation["left_ee_pose"] = left
        if right is not None:
            observation["right_ee_pose"] = right
        ee_poses = [pose for pose in (left, right) if pose is not None]
        if ee_poses:
            observation["ee_poses"] = ee_poses

    @staticmethod
    def _frame_to_pose7d(frame: Any) -> Optional[List[float]]:
        """把 ``frames.<link>`` 的 position+quaternion 展平为 7 维位姿 list。"""
        if not isinstance(frame, Mapping):
            return None
        position = frame.get("position") or {}
        quaternion = (frame.get("orientation") or {}).get("quaternion") or {}
        try:
            pose = [
                float(position["x"]),
                float(position["y"]),
                float(position["z"]),
                float(quaternion["x"]),
                float(quaternion["y"]),
                float(quaternion["z"]),
                float(quaternion["w"]),
            ]
        except (KeyError, TypeError, ValueError):
            return None
        return pose

    def _safe_state(self, getter: Any) -> Optional[np.ndarray]:
        """调用 ``(数据, 时间戳)`` 型状态查询，异常/空值返回 ``None``。"""
        try:
            raw = getter()
        except Exception as exc:
            logger.warning("[AgiBotG01] state query failed: %s", exc)
            return None
        values = raw[0] if isinstance(raw, tuple) else raw
        if values is None:
            return None
        return np.asarray(values, dtype=np.float64).reshape(-1)

    @staticmethod
    def _normalize_dim(values: np.ndarray, key: str, size: int) -> np.ndarray:
        """按期望维度规整数组（超长截断、不足补零），并记录告警。"""
        if values.shape[0] == size:
            return values
        logger.warning(
            "[AgiBotG01] %s expects %d values, got %d; %s",
            key,
            size,
            values.shape[0],
            "truncating" if values.shape[0] > size else "zero-padding",
        )
        result = np.zeros(size, dtype=np.float64)
        n = min(values.shape[0], size)
        result[:n] = values[:n]
        return result

    # ------------------------------------------------------------------
    # 动作
    # ------------------------------------------------------------------

    def send_action(self, command: Mapping[str, Any]) -> None:
        """执行云端动作（平铺字段，见模块 docstring；缺失字段自动跳过）。

        ``runtime.dry_run``（客户端级）或 ``hardware.config.dry_run``（适配器级）
        开启时只打印不执行。
        """
        client = self._require_client()

        if self._dry_run:
            logger.info(
                "[AgiBotG01][DRY_RUN] action dims: %s",
                {
                    key: len(AgiBotG01HardwareAdapter.as_float_list(value))
                    for key, value in command.items()
                },
            )
            return

        executed: List[str] = []
        handlers = (
            ("arm_cmd_joint_pos", client.move_arm),
            ("gripper_cmd_pos", client.move_gripper),
            ("hand_cmd_joint_pos", client.move_hand),
            ("head_cmd_joint_pos", client.move_head),
            ("waist_cmd_joint_pos", client.move_waist),
        )
        for key, move in handlers:
            if key not in command:
                continue
            values = self.as_float_list(command[key])
            if not self._check_dims(key, values):
                continue
            move(values)
            executed.append(key)

        if "wheel_cmd_speed" in command:
            values = self.as_float_list(command["wheel_cmd_speed"])
            if self._check_dims("wheel_cmd_speed", values):
                # move_wheel(linear, angular)
                client.move_wheel(values[0], values[1])
                executed.append("wheel_cmd_speed")

        pose_cmd = self._handle_pose_command(client, command)
        if pose_cmd:
            executed.append(pose_cmd)

        if self._is_truthy(command.get("motion_stop_cmd")):
            # 《指南》8.1 set_motion_stop：全身急停
            client.set_motion_stop()
            executed.append("motion_stop_cmd")

        if not executed:
            logger.warning(
                "[AgiBotG01] send_action received no AgiBot G01 recognized fields "
                "(got: %s); skipped",
                sorted(command),
            )
        else:
            logger.debug("[AgiBotG01] executed actions: %s", executed)

    def _handle_pose_command(self, client: AgiBotG01Client, command: Mapping[str, Any]) -> str:
        """处理末端位姿命令（``left_pose_cmd`` / ``right_pose_cmd``，7 维位姿）。

        调用《指南》8.1 ``set_end_effector_pose_control(lifetime, control_group,
        left_pose, right_pose)``。返回已执行的键名描述，未执行返回空串。
        """
        left: Optional[List[float]] = None
        right: Optional[List[float]] = None
        if "left_pose_cmd" in command:
            left = self.as_float_list(command["left_pose_cmd"])
            if not self._check_dims("left_pose_cmd", left):
                left = None
        if "right_pose_cmd" in command:
            right = self.as_float_list(command["right_pose_cmd"])
            if not self._check_dims("right_pose_cmd", right):
                right = None
        if left is None and right is None:
            return ""
        client.set_end_effector_pose(
            lifetime=self._pose_lifetime_s,
            control_group=self._pose_control_group,
            left_pose=self._pose_to_dict(left) if left else None,
            right_pose=self._pose_to_dict(right) if right else None,
        )
        keys = [k for k, v in (("left", left), ("right", right)) if v is not None]
        return "pose_cmd(" + "/".join(keys) + ")"

    @staticmethod
    def _pose_to_dict(pose: List[float]) -> Dict[str, float]:
        """7 维位姿 → GDK 位姿 dict ``{x, y, z, qx, qy, qz, qw}``。"""
        return {
            "x": pose[0],
            "y": pose[1],
            "z": pose[2],
            "qx": pose[3],
            "qy": pose[4],
            "qz": pose[5],
            "qw": pose[6],
        }

    @staticmethod
    def _is_truthy(value: Any) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return value != 0
        if isinstance(value, str):
            return value.strip().lower() in ("true", "1", "yes", "on")
        return bool(value)

    # ------------------------------------------------------------------
    # 工具方法
    # ------------------------------------------------------------------

    def _check_dims(self, key: str, values: List[float]) -> bool:
        expected = EXPECTED_DIMS.get(key)
        if expected is not None and len(values) != expected:
            logger.error(
                "[AgiBotG01] action %s expects %d values, got %d; skipped",
                key,
                expected,
                len(values),
            )
            return False
        return True

    @staticmethod
    def as_float_list(value: Any) -> List[float]:
        """将 ndarray / 序列 / Pose7D / ExtensionValue 解码为 float 列表。

        - ``Pose7D``（含 ``.data`` 数值列表的位姿对象）按列表处理
        - ``ExtensionValue``（或同字段 dict）按 ``dtype``（FLOAT32/FLOAT64）
          从 ``data`` 字节解包
        """
        if value is None:
            return []
        if isinstance(value, np.ndarray):
            return value.reshape(-1).astype(float).tolist()
        if isinstance(value, (list, tuple)):
            return [float(v) for v in value]
        data = getattr(value, "data", None)
        dtype_name = getattr(value, "dtype", None)
        if data is None and isinstance(value, Mapping):
            data = value.get("data")
            dtype_name = value.get("dtype", dtype_name)
        if data is not None:
            if isinstance(data, (list, tuple)):
                # Pose7D 等位姿对象：.data 为数值列表
                return [float(v) for v in data]
            dtype_code = _EXT_FLOAT_DTYPES.get(str(dtype_name or "FLOAT32").upper(), "<f4")
            return np.frombuffer(bytes(data), dtype=dtype_code).astype(float).tolist()
        # 标量兜底
        return [float(value)]
