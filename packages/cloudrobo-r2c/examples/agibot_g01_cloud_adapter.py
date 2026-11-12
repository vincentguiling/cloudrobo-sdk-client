"""AgiBot G01 样例 cloud-adapter：在多组动作之间来回运动。

配合机器人端 ``config/robot_agibot_g01_config.yaml`` 使用。动作组由
``GROUPS`` dict 数组驱动（默认三组为《指南》第 7 章 ``RobotDds`` 关节空间动作，
真机调试采集的目标值）：

- **夹爪任意角度 + 全程插值**：关节、位姿、夹爪均从当前值线性/球面插值到目标
  （chunk size = 2 × publish_hz，每组执行约 2 秒），避免目标跳变超过
  0.2618 rad（15°）限幅；欧拉角由连续 SLERP 四元数逐步转换，天然避免过圈

**第 8 章示例组**——按 ``joint_states.names`` 路由，演示第 8 章新增能力
（默认注释，取消注释并加入 ``GROUPS`` 即可启用；机器人端建议先保持
``dry_run: true`` 观察一轮）：

- ``GROUP_POSE`` ``ch8_pose(绝对位姿)``：**绝对位姿控制**——直接下发组内配置的
  ``left_pose`` / ``right_pose``（四元数 ``[x,y,z,qx,qy,qz,qw]`` 绝对位姿，
  现场按实际工作空间修改）→ ``set_end_effector_pose_control``；下发通道与角度
  表示由 ``--pose-channel joint_states|ee`` 与 ``--pose-repr quat|euler`` 配置
  （欧拉角为 ``left_pose_x/y/z/roll/pitch/yaw`` 逐维 names，ZYX，机器人端内部
  转换为四元数；ee 通道为四元数原生格式）。注意：关节与末端互斥，ee 通道时
  joint_states 仅携带夹爪分量
- ``GROUP_JOINTS8`` ``ch8_joints(组名关节)``：names = ``left_arm_joint1..7`` /
  ``right_arm_joint1..7``（逐关节名，不带下划线，与第 7 章
  ``left_arm_joint_1`` 区分），position 按名对应 → ``set_joint_position_control``
- 第 8 章示例组同样**全程插值**：位姿从观测当前末端位姿 SLERP/线性插值到目标
  （欧拉角由连续四元数转换，避免过圈），夹爪从当前角度线性插值

连接约定（client config）：zenohd 与网关监听 ``tcp/0.0.0.0:7447``（所有网卡），
云端适配器 ``endpoint_role: connect``，同机部署连 ``tcp/0.0.0.0:7447``，跨机部署用
``--zenoh-endpoints`` 指向网关/zenohd 的实际局域网地址（如 ``tcp/192.168.1.10:7447``）；
``project_id`` / ``device_id`` 按现场部署填写（默认 ``r2c`` / ``local``）。

用法示例：

    python examples/agibot_g01_cloud_adapter.py
    python examples/agibot_g01_cloud_adapter.py --project-id r2c --device-id local
    python examples/agibot_g01_cloud_adapter.py --duration 60 --steps 60

安全提示：机器人端请先以 ``runtime.dry_run: true`` 验证数据链路与动作序列，
确认无误后再改为 ``false`` 真实执行。
"""

from __future__ import annotations

import argparse
import logging
import math
import re
import struct
import time
from typing import Any, Dict, List, Optional, Sequence

import numpy as np
from _cloud_adapter_common import build_parser, normalize_for_logging
from cloudrobo_r2c import ClientConfig, R2CClient
from cloudrobo_r2c.common.models import Actions

logger = logging.getLogger(__name__)

ADAPTER_NAME = "AgiBotG01CloudAdapter"
DEFAULT_PROJECT_ID = "r2c"
DEFAULT_DEVICE_ID = "local"
DEFAULT_CLIENT_ID = "agibot-g01-cloud-adapter"
# zenohd 与网关监听 tcp/0.0.0.0:7447（所有网卡）；跨机部署时用
# --zenoh-endpoints 指向网关/zenohd 的实际局域网地址
DEFAULT_ZENOH_ENDPOINTS = "tcp/0.0.0.0:7447"

# 与机器人端 runtime.publish_hz 保持一致
PUBLISH_HZ = 30.0

# 与 config/robot_agibot_g01_config.yaml 的 joint_states.names 保持一致
JOINT_NAMES = [f"left_arm_joint_{i}" for i in range(1, 8)] + [
    f"right_arm_joint_{i}" for i in range(1, 8)
]
GRIPPER_NAMES = ["left_gripper", "right_gripper"]

ARM_DOF = 14
GRIPPER_DOF = 2
HEAD_DOF = 2
WAIST_DOF = 2
POSE_DOF = 7  # 末端位姿维度 [x, y, z, qx, qy, qz, qw]（《指南》8.1）
# chunk size 一般为 2 倍 publish_hz（每组动作执行约 2 秒）
INTERP_STEPS = int(2 * PUBLISH_HZ)

# 第 8 章位姿的两种角度表示对应的逐维 names（《指南》8.1 位姿 dict 分量）
_POSE_DIM_QUAT = ("x", "y", "z", "rx", "ry", "rz", "rw")
_POSE_DIM_EULER = ("x", "y", "z", "roll", "pitch", "yaw")


def _pose_dim_names(side: str, pose_repr: str = "quat") -> List[str]:
    """逐维位姿 names：四元数 left_pose_x/y/z/rx/ry/rz/rw 或欧拉角 .../roll/pitch/yaw。"""
    dims = _POSE_DIM_EULER if pose_repr == "euler" else _POSE_DIM_QUAT
    return [f"{side}_pose_{d}" for d in dims]


def _quat_to_euler_rpy(qx: float, qy: float, qz: float, qw: float):
    """四元数 (qx, qy, qz, qw) → RPY 欧拉角（ZYX 约定）。"""
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def _pack_ext(values: Sequence[float]) -> Dict[str, Any]:
    """把一维数值打包为 extension dict（FLOAT32，适配器侧负责解码）。"""
    return {
        "dtype": "FLOAT32",
        "shape": [len(values)],
        "data": struct.pack(f"<{len(values)}f", *[float(v) for v in values]),
        "mime_type": "",
    }


# ---------------------------------------------------------------------------
# 动作组定义（GROUPS dict 数组驱动循环；现场请按机器人实际位姿/限位调整目标值）
# 需要演示第 8 章能力时，把下方注释掉的 GROUP_POSE / GROUP_JOINTS8 取消注释
# 并加入 GROUPS 即可（机器人端建议先保持 dry_run: true 观察一轮）。
# ---------------------------------------------------------------------------

# 组 0：关节组 A —— 双臂微屈的"收拢"姿态，夹爪小开度
GROUP_A = {
    "name": "joint_a(收拢)",
    "kind": "joints",
    "joints": [
        -1.125699520111084,
        0.5228543281555176,
        0.10490940511226654,
        -1.283866286277771,
        0.7302126884460449,
        1.476741075515747,
        -0.16992810368537903,
        1.1257692575454712,
        -0.522889256477356,
        -0.10499665141105652,
        1.2836743593215942,
        -0.7302126884460449,
        -1.495011329650879,
        0.16991065442562103,
    ],
    "gripper": [0.15, 0.15],  # 任意角度示例：连续小数值
}

# 组 1：关节组 B —— 双臂抬起展开姿态，夹爪大开度（左右不同角度）
GROUP_B = {
    "name": "joint_b(展开)",
    "kind": "joints",
    "joints": [
        -1.187420129776001,
        0.5014432072639465,
        0.7781304121017456,
        -1.119522213935852,
        0.6628905534744263,
        1.0817254781723022,
        -0.25557270646095276,
        1.1248793601989746,
        -0.9522988796234131,
        -0.04086789861321449,
        0.5556254386901855,
        -1.3621994256973267,
        -1.5153056383132935,
        0.16874149441719055,
    ],
    # "head": [0, 0],
    # "waist": [0, 10],
    "gripper": [0.85, 0.35],  # 左右夹爪各自任意角度
}

# 组 2：全身姿态组 —— 手臂 + 头部 + 腰部一起到达新姿态（第 7 章"整体姿态"语义）
GROUP_C = {
    "name": "joints(全身姿态)",
    "kind": "joints",
    "joints": [
        -1.1872631311416626,
        0.7889842987060547,
        1.6079477071762085,
        -1.1197315454483032,
        0.6622798442840576,
        0.26489099860191345,
        -0.2558518946170807,
        1.1237276792526245,
        -0.9279735684394836,
        -0.01715334877371788,
        0.14202554523944855,
        -1.5966051816940308,
        -1.3580461740493774,
        0.20629389584064484,
    ],
    # 头部 [yaw(rad), pitch(rad)]，腰部 [pitch(rad), height(cm)]（《指南》7.2.1）
    # "head": [10, 10],
    # "waist": [0, 20],
    "gripper": [0.5, 0.8],
}

# 参与往复循环的动作组（dict 数组）
GROUPS: List[Dict[str, Any]] = [GROUP_A, GROUP_B, GROUP_C]

# ---------------------------------------------------------------------------
# 第 8 章示例组（默认注释）：绝对位姿控制 + 逐关节名关节控制。
# 启用方式：取消注释并加入上方 GROUPS（机器人端建议先 dry_run: true 观察一轮）。
# ---------------------------------------------------------------------------

# # 组 3：第 8 章绝对位姿控制 —— 直接下发组内配置的绝对位姿目标
# # （left_pose / right_pose，四元数 [x,y,z,qx,qy,qz,qw]，示例值请按实际
# # 工作空间修改）→ set_end_effector_pose_control；
# # 下发通道（joint_states 逐维 names / end_effector_poses 通道）与角度表示
# # （四元数 / 欧拉角）由 --pose-channel / --pose-repr 配置选择。
# # 注意：关节与末端互斥 —— ee 通道时 joint_states 仅携带夹爪分量。
# GROUP_POSE = {
#     "name": "ch8_pose(绝对位姿)",
#     "kind": "ch8_pose",
#     "left_pose": [0.35, -0.30, 0.30, 0.0, 0.0, 0.0, 1.0],
#     "right_pose": [0.35, 0.30, 0.30, 0.0, 0.0, 0.0, 1.0],
# }

# # 组 4：第 8 章关节控制 —— names = left_arm_joint1..7 / right_arm_joint1..7
# # （逐关节名，不带下划线，与第 7 章 left_arm_joint_1 区分），关节目标复用
# # GROUP_C 真机值 → set_joint_position_control
# GROUP_JOINTS8 = {
#     "name": "ch8_joints(组名关节)",
#     "kind": "ch8_joints",
#     "joints": [
#         -1.1872631311416626,
#         0.7889842987060547,
#         1.6079477071762085,
#         -1.1197315454483032,
#         0.6622798442840576,
#         0.26489099860191345,
#         -0.2558518946170807,
#         1.1237276792526245,
#         -0.9279735684394836,
#         -0.01715334877371788,
#         0.14202554523944855,
#         -1.5966051816940308,
#         -1.3580461740493774,
#         0.20629389584064484,
#     ],
#     "gripper": [0.5, 0.8],
# }


def _ping_pong_sequence(count: int) -> List[int]:
    """生成往复序列：n=3 → [0, 1, 2, 1]；n=4 → [0, 1, 2, 3, 2, 1]。"""
    if count <= 1:
        return [0]
    return list(range(count)) + list(range(count - 2, 0, -1))


class ActionGroupCycler:
    """按 GROUPS dict 数组往复生成 Actions（关节插值、夹爪任意角度）。"""

    def __init__(
        self,
        groups: List[Dict[str, Any]],
        steps: int = INTERP_STEPS,
        sequence: Optional[Sequence[int]] = None,
        pose_channel: str = "joint_states",
        pose_repr: str = "quat",
    ) -> None:
        self._groups = groups
        self._steps = max(int(steps), 1)
        self._sequence = list(sequence) if sequence else _ping_pong_sequence(len(groups))
        self._cycle_index = 0
        self._pose_channel = pose_channel
        self._pose_repr = pose_repr

    def next_group_index(self) -> int:
        index = self._sequence[self._cycle_index % len(self._sequence)]
        self._cycle_index += 1
        return index

    def build_action(
        self,
        now_ms: int,
        current_joints: Optional[Sequence[float]],
        current_gripper: Optional[Sequence[float]],
        current_poses: Optional[Dict[str, Sequence[float]]] = None,
    ) -> Actions:
        """按下一组动作构建 Actions（关节/位姿/夹爪全程插值）。"""
        group = self._groups[self.next_group_index()]
        kind = group.get("kind", "joints")

        if kind == "ch8_pose":
            return self._build_ch8_pose(now_ms, group, current_gripper, current_poses)
        if kind == "ch8_joints":
            return self._build_ch8_joints(now_ms, group, current_gripper, current_joints)
        return self._build_joint_action(now_ms, group, current_joints, current_gripper)

    # -- 第 7 章关节组（现有主链路） ----------------------------------------------

    def _build_joint_action(
        self,
        now_ms: int,
        group: Dict[str, Any],
        current_joints: Optional[Sequence[float]],
        current_gripper: Optional[Sequence[float]],
    ) -> Actions:
        target = np.asarray(group["joints"], dtype=float)
        gripper_target = np.asarray(group["gripper"], dtype=float)
        start_joints = np.asarray(current_joints or target, dtype=float)
        if start_joints.shape != target.shape:
            logger.warning(
                "[%s] observation joints dim=%s != %d, use target as start",
                ADAPTER_NAME,
                start_joints.shape,
                ARM_DOF,
            )
            start_joints = target
        start_gripper = np.asarray(current_gripper or gripper_target, dtype=float)
        if start_gripper.shape != gripper_target.shape:
            start_gripper = gripper_target

        steps = self._steps
        joint_chunk = [
            (start_joints + (target - start_joints) * (i + 1) / steps).tolist()
            for i in range(steps)
        ]
        gripper_chunk = [
            (start_gripper + (gripper_target - start_gripper) * (i + 1) / steps).tolist()
            for i in range(steps)
        ]

        payload: Dict[str, Any] = {
            "timestamp": now_ms,
            "chunk_size": steps,
            "joint_states": {
                "names": list(JOINT_NAMES),
                "position": joint_chunk,
                "velocity": [],
                "torque": [],
            },
            "end_effector_poses": {},
            "end_effector_states": {
                "names": list(GRIPPER_NAMES),
                "position": gripper_chunk,  # 任意角度：连续值透传
                "velocity": [],
                "torque": [],
            },
            "localization": {"odom_pose": [], "map_pose": []},
            "extensions": {},
        }

        # 全身姿态组：头/腰目标经 extensions 下发（《指南》7.2.1 move_head/move_waist）
        if "head" in group:
            payload["extensions"]["head_joint"] = _pack_ext(group["head"])
        if "waist" in group:
            payload["extensions"]["waist_joint"] = _pack_ext(group["waist"])

        logger.info(
            "[%s] group=%s joints=%.3f→%.3f gripper=%s",
            ADAPTER_NAME,
            group["name"],
            float(start_joints[1]),
            float(target[1]),
            group["gripper"],
        )
        return Actions.from_dict(payload)

    # -- 第 8 章示例组（绝对位姿 / 逐关节名关节） ----------------------------------

    @staticmethod
    def _quat_slerp(q0: Sequence[float], q1: Sequence[float], t: float) -> List[float]:
        """四元数球面插值（SLERP，自动取最短路径），q 为 [qx, qy, qz, qw]。"""
        dot = sum(a * b for a, b in zip(q0, q1))
        if dot < 0:  # 取最短路径
            q1 = [-v for v in q1]
            dot = -dot
        dot = min(1.0, max(-1.0, dot))
        if dot > 0.9995:  # 近似平行：线性插值 + 归一化
            out = [a + (b - a) * t for a, b in zip(q0, q1)]
            norm = math.sqrt(sum(v * v for v in out)) or 1.0
            return [v / norm for v in out]
        theta0 = math.acos(dot)
        sin_theta0 = math.sin(theta0)
        s0 = math.sin((1 - t) * theta0) / sin_theta0
        s1 = math.sin(t * theta0) / sin_theta0
        return [s0 * a + s1 * b for a, b in zip(q0, q1)]

    @staticmethod
    def _interp_pose7d(start: Sequence[float], target: Sequence[float], t: float) -> List[float]:
        """位姿 7 维插值：平移线性插值 + 姿态四元数 SLERP（避免欧拉角过圈）。"""
        trans = [s + (g - s) * t for s, g in zip(start[:3], target[:3])]
        return trans + ActionGroupCycler._quat_slerp(start[3:7], target[3:7], t)

    def _build_ch8_pose(
        self,
        now_ms: int,
        group: Dict[str, Any],
        current_gripper: Optional[Sequence[float]],
        current_poses: Optional[Dict[str, Sequence[float]]],
    ) -> Actions:
        """第 8 章绝对位姿控制示例（通道/表示由配置选择，全程插值）。

        - 目标位姿 = 组内配置的 ``left_pose`` / ``right_pose``（四元数
          ``[x,y,z,qx,qy,qz,qw]`` 绝对位姿）
        - 插值：起点为观测中的当前末端位姿（``end_effector_poses``，缺失时
          退化为目标保持）；平移线性插值 + 四元数 SLERP，**欧拉角由逐插值
          四元数转换得出，天然连续、避免过圈**
        - ``pose_channel="joint_states"``：经 joint_states 逐维 names 下发
          （四元数 rx/ry/rz/rw 或欧拉角 roll/pitch/yaw，按 pose_repr）
        - ``pose_channel="ee"``：经 end_effector_poses.pose 通道下发
          （该通道为四元数）；关节与末端互斥，joint_states 仅携带夹爪分量
        - 夹爪同样从当前角度线性插值到目标（任意角度连续值）
        """
        left_target = [float(v) for v in group["left_pose"]]
        right_target = [float(v) for v in group["right_pose"]]
        if len(left_target) != POSE_DOF or len(right_target) != POSE_DOF:
            raise ValueError(
                f"{group['name']}: left_pose/right_pose 须为 {POSE_DOF} 维"
                f"[x,y,z,qx,qy,qz,qw]，got {len(left_target)}/{len(right_target)}"
            )
        gripper_target = list(current_gripper) if current_gripper else [0.5, 0.8]

        # 插值起点：观测中的当前末端位姿（缺失时退化为目标保持）
        poses = current_poses or {}
        start_left = poses.get("left")
        start_right = poses.get("right")
        if start_left is None or len(start_left) != POSE_DOF:
            start_left = left_target
        if start_right is None or len(start_right) != POSE_DOF:
            start_right = right_target
        start_gripper = list(current_gripper) if current_gripper else list(gripper_target)

        steps = self._steps
        left_chunk = [
            self._interp_pose7d(start_left, left_target, (i + 1) / steps) for i in range(steps)
        ]
        right_chunk = [
            self._interp_pose7d(start_right, right_target, (i + 1) / steps) for i in range(steps)
        ]
        gripper_chunk = [
            (
                start_gripper[0] + (gripper_target[0] - start_gripper[0]) * (i + 1) / steps,
                start_gripper[1] + (gripper_target[1] - start_gripper[1]) * (i + 1) / steps,
            )
            for i in range(steps)
        ]

        if self._pose_channel == "ee":
            # ee 通道（四元数原生）；关节与末端互斥 → joint_states 仅携带夹爪分量
            names: List[str] = ["left_gripper", "right_gripper"]
            position_chunk = gripper_chunk
            ee_poses = {
                "left": {"pose": left_chunk},
                "right": {"pose": right_chunk},
            }
            channel_desc = "via end_effector_poses channel"
        else:
            # joint_states 逐维 names（四元数或欧拉角按 pose_repr，逐插值步转换）
            names = (
                _pose_dim_names("left", self._pose_repr)
                + _pose_dim_names("right", self._pose_repr)
                + ["left_gripper", "right_gripper"]
            )
            position_chunk = []
            for i in range(steps):
                row: List[float] = []
                for pose in (left_chunk[i], right_chunk[i]):
                    row += pose[:3]
                    if self._pose_repr == "euler":
                        # 由连续 SLERP 四元数转换，天然避免欧拉角过圈
                        row += list(_quat_to_euler_rpy(*pose[3:7]))
                    else:
                        row += pose[3:7]
                row += gripper_chunk[i]
                position_chunk.append(row)
            ee_poses = {}
            channel_desc = "via joint_states names"

        payload: Dict[str, Any] = {
            "timestamp": now_ms,
            "chunk_size": steps,
            "joint_states": {
                "names": names,
                "position": position_chunk,
                "velocity": [],
                "torque": [],
            },
            "end_effector_poses": ee_poses,
            "end_effector_states": {
                "names": list(GRIPPER_NAMES),
                "position": gripper_chunk,
                "velocity": [],
                "torque": [],
            },
            "localization": {"odom_pose": [], "map_pose": []},
            "extensions": {},
        }
        logger.info(
            "[%s] group=%s left=%s right=%s gripper=%s (%s, %s)",
            ADAPTER_NAME,
            group["name"],
            np.round(left_target, 3).tolist(),
            np.round(right_target, 3).tolist(),
            gripper_target,
            channel_desc,
            self._pose_repr,
        )
        return Actions.from_dict(payload)

    def _build_ch8_joints(
        self,
        now_ms: int,
        group: Dict[str, Any],
        current_gripper: Optional[Sequence[float]],
        current_joints: Optional[Sequence[float]],
    ) -> Actions:
        """第 8 章关节控制示例（names=left_arm_joint1..7/right_arm_joint1..7，
        逐关节名与 position 一一对应 → set_joint_position_control）。
        关节从当前角度线性插值到目标（避免单步跳变超限）。"""
        target = np.asarray(group["joints"], dtype=float)
        start_joints = np.asarray(current_joints or target, dtype=float)
        if start_joints.shape != target.shape:
            start_joints = target
        gripper_target = np.asarray(group["gripper"], dtype=float)
        start_gripper = np.asarray(current_gripper or gripper_target, dtype=float)
        if start_gripper.shape != gripper_target.shape:
            start_gripper = gripper_target

        steps = self._steps
        names = [f"left_arm_joint{i}" for i in range(1, 8)]
        names += [f"right_arm_joint{i}" for i in range(1, 8)]
        joint_chunk = [
            (start_joints + (target - start_joints) * (i + 1) / steps).tolist()
            for i in range(steps)
        ]
        gripper_chunk = [
            (start_gripper + (gripper_target - start_gripper) * (i + 1) / steps).tolist()
            for i in range(steps)
        ]

        payload: Dict[str, Any] = {
            "timestamp": now_ms,
            "chunk_size": self._steps,
            "joint_states": {
                "names": names,  # 逐关节名（不带下划线）→ 第 8 章关节控制
                "position": joint_chunk,
                "velocity": [],
                "torque": [],
            },
            "end_effector_poses": {},
            "end_effector_states": {
                "names": list(GRIPPER_NAMES),
                "position": gripper_chunk,
                "velocity": [],
                "torque": [],
            },
            "localization": {"odom_pose": [], "map_pose": []},
            "extensions": {},
        }
        logger.info(
            "[%s] group=%s left_arm=%.3f→%.3f right_arm=%.3f→%.3f (via "
            "set_joint_position_control)",
            ADAPTER_NAME,
            group["name"],
            float(start_joints[0]),
            float(target[0]),
            float(start_joints[7]),
            float(target[7]),
        )
        return Actions.from_dict(payload)


def parse_args() -> argparse.Namespace:
    parser = build_parser(
        description="AgiBot G01 cloud adapter demo: cycle between action groups (ch7/ch8 routing)",
        default_project_id=DEFAULT_PROJECT_ID,
        default_device_id=DEFAULT_DEVICE_ID,
        default_client_id=DEFAULT_CLIENT_ID,
        help_project_id="Project ID",
        help_device_id="Target device ID",
        help_client_id="Client ID",
        help_duration="Running duration in seconds; 0 means continuous",
    )
    parser.add_argument(
        "--steps",
        type=int,
        default=INTERP_STEPS,
        help="Interpolation steps per group (chunk size, default 2 x publish_hz)",
    )
    parser.add_argument(
        "--pose-channel",
        type=str,
        default="joint_states",
        choices=["joint_states", "ee"],
        help="Where the chapter-8 pose group sends ee poses "
        "(takes effect when a ch8_pose group is enabled in GROUPS)",
    )
    parser.add_argument(
        "--pose-repr",
        type=str,
        default="quat",
        choices=["quat", "euler"],
        help="Angle representation for the chapter-8 pose group "
        "(euler = left_pose_roll/pitch/yaw, ZYX)",
    )
    parser.add_argument(
        "--zenoh-endpoints",
        type=str,
        default=DEFAULT_ZENOH_ENDPOINTS,
        help="Zenoh endpoints of zenohd/gateway, e.g. tcp/0.0.0.0:7447 "
        "(same machine) or tcp/<gateway-ip>:7447 (cross machine)",
    )
    return parser.parse_args()


def _extract_current(observation: Any) -> Dict[str, Any]:
    """从观测中按 names 提取当前双臂 14 关节与夹爪，缺失时返回 None。

    joint_states 可能携带位姿/夹爪镜像等额外分量（names 与 position 一一对应），
    因此必须按 names 逐维提取双臂关节，不能直接取 position 数组。
    """
    try:
        names = [str(v) for v in observation.joint_states.names]
        position = [float(v) for v in observation.joint_states.position]
    except Exception:
        return {"joints": None, "gripper": None, "poses": {}}

    arm: List[Optional[float]] = [None] * ARM_DOF
    gripper: Dict[str, float] = {}
    for index, name in enumerate(names):
        if index >= len(position):
            break
        value = position[index]
        arm_match = re.match(r"^(left|right)_arm_joint_([1-7])$", name)
        if arm_match is not None:
            side, joint_no = arm_match.group(1), int(arm_match.group(2))
            arm[(0 if side == "left" else 7) + joint_no - 1] = value
            continue
        if name in GRIPPER_NAMES:
            gripper[name] = value

    joints = arm if all(v is not None for v in arm) else None
    left_grip = gripper.get("left_gripper")
    right_grip = gripper.get("right_gripper")
    gripper_out = (
        [left_grip, right_grip] if left_grip is not None and right_grip is not None else None
    )
    return {"joints": joints, "gripper": gripper_out}


def main() -> None:
    args = parse_args()
    cycler = ActionGroupCycler(
        groups=GROUPS,
        steps=args.steps,
        pose_channel=args.pose_channel,
        pose_repr=args.pose_repr,
    )

    config = ClientConfig(
        project_id=args.project_id,
        device_id=args.device_id,
        client_id=args.client_id,
        endpoint_role="connect",
        endpoints=[args.zenoh_endpoints],
        mode="peer",
    )
    logger.info(
        "[%s] Connecting: project=%s, device=%s, client=%s, endpoints=%s",
        ADAPTER_NAME,
        args.project_id,
        args.device_id,
        args.client_id,
        args.zenoh_endpoints,
    )
    client = R2CClient.connect(config)

    def on_observation(observation: Any) -> None:
        normalized = normalize_for_logging(observation)
        logger.debug(
            "\n=== Received observation ===\n%s\n============================",
            normalized,
        )
        current = _extract_current(observation)
        action = cycler.build_action(
            now_ms=int(time.time() * 1000),
            current_joints=current["joints"],
            current_gripper=current["gripper"],
            current_poses=current["poses"],
        )
        client.publish_actions(action)

    client.subscribe_observations(on_observation, target_device_id=args.device_id)
    logger.info(
        "[%s] Subscribed: %s/%s/inference/observations",
        ADAPTER_NAME,
        args.project_id,
        args.device_id,
    )
    logger.info("[%s] Waiting for observations (Ctrl+C to stop)...", ADAPTER_NAME)

    start = time.time()
    try:
        while True:
            if args.duration > 0 and time.time() - start >= args.duration:
                logger.info("[%s] Duration reached, stopping.", ADAPTER_NAME)
                break
            time.sleep(1)
    except KeyboardInterrupt:
        logger.info("[%s] Interrupted by user.", ADAPTER_NAME)
    finally:
        client.close()
        logger.info("[%s] Closed.", ADAPTER_NAME)


if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s [%(levelname)s] %(name)s: %(message)s"
    )
    main()
