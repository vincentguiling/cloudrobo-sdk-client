"""AgiBot G01 样例 cloud-adapter：在多组动作之间来回运动。

配合机器人端 ``config/robot_agibot_g01_config.yaml`` 使用。默认三组为
《指南》第 7 章 ``RobotDds`` 关节空间动作（``move_arm`` / ``move_gripper``，
真机调试采集的目标值）：

- **夹爪任意角度**：夹爪值连续任意（默认归一化 ``[0, 1]``，也可用 ``[35, 120]``
  原始值），随组切换到不同开度（用最终角度，不插值）
- **往复运动**：组序列 ``0 → 1 → 2 → 1 → 0 → ...`` 来回循环；关节部分从当前
  关节角线性插值到目标（chunk size = 2 × publish_hz，每组执行约 2 秒），
  避免目标跳变超过 0.2618 rad（15°）限幅

**第 8 章示例组（``--include-ch8`` 启用）**——按 ``joint_states.names`` 路由的三种
新格式演示（序列 ``0→1→2→1 → 3→4→5→4 → ...``）：

- 组 3 ``ch8_pose(joint_states位姿)``：names = ``["left_pose", "right_pose"]``，
  position 为两个 7 维位姿 ``[x,y,z,qx,qy,qz,qw]`` → ``set_end_effector_pose_control``
- 组 4 ``ch8_pose(ee通道)``：joint_states 保持当前关节，位姿经 ``end_effector_poses``
  通道下发（演示双通道兜底：joint_states 位姿存在时优先）
- 组 5 ``ch8_joints(组名关节)``：names = ``["left_arm", "right_arm"]``，position 按每组
  7 关节解析 → ``set_joint_position_control``
- 位姿目标 = 观测中的当前末端位姿（``end_effector_poses``）+ z 向偏移（默认 +5cm），
  观测无位姿时自动跳过该轮；首次启用建议机器人端保持 ``dry_run: true`` 观察一轮

连接约定（client config）：云端适配器 ``endpoint_role: connect`` 连接 Zenoh 路由
``tcp/127.0.0.1:7447``，``project_id`` / ``device_id`` 按现场部署填写（默认
``r2c`` / ``local``）。

用法示例：

    python examples/agibot_g01_cloud_adapter.py
    python examples/agibot_g01_cloud_adapter.py --project-id r2c --device-id local
    python examples/agibot_g01_cloud_adapter.py --duration 60 --steps 60
    python examples/agibot_g01_cloud_adapter.py --include-ch8 --pose-offset-z 0.05

安全提示：机器人端请先以 ``runtime.dry_run: true`` 验证数据链路与动作序列，
确认无误后再改为 ``false`` 真实执行。
"""

from __future__ import annotations

import argparse
import logging
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


def _pose_dim_names(side: str) -> List[str]:
    """逐维位姿 names（四元数表示）：left_pose_x/y/z/rx/ry/rz/rw。"""
    dims = ["x", "y", "z", "rx", "ry", "rz", "rw"]
    return [f"{side}_pose_{d}" for d in dims]


def _pack_ext(values: Sequence[float]) -> Dict[str, Any]:
    """把一维数值打包为 extension dict（FLOAT32，适配器侧负责解码）。"""
    return {
        "dtype": "FLOAT32",
        "shape": [len(values)],
        "data": struct.pack(f"<{len(values)}f", *[float(v) for v in values]),
        "mime_type": "",
    }


# ---------------------------------------------------------------------------
# 3 组动作定义（现场请按机器人实际位姿/限位调整目标值）
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
# 组 2：全身位姿组 —— 手臂 + 头部 + 腰部一起到达新姿态（第 7 章"整体姿态"语义）
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

# 往复序列：0 → 1 → 2 → 1 → 0 → 1 → 2 → ...（来回）
CYCLE_SEQUENCE = [0, 1, 2, 1]

# ---------------------------------------------------------------------------
# 第 8 章示例组（--include-ch8 启用）：names 路由的三种新格式演示
# 真机注意：位姿目标基于当前观测末端位姿 + z 向偏移，组名关节复用 GROUP_C 真机值；
# 首次启用建议机器人端保持 dry_run: true 观察一轮。
# ---------------------------------------------------------------------------

# 组 3：末端位姿（经 joint_states names 路由）——
# names=["left_pose","right_pose"]，position 为两个 7 维位姿 [x,y,z,qx,qy,qz,qw]
GROUP_D = {
    "name": "ch8_pose(joint_states位姿)",
    "kind": "pose_joints_names",
    "pose_offset": [0.0, 0.0, 0.05],  # 基于当前末端位姿的 [x, y, z] 偏移（米）
}

# 组 4：末端位姿（经 end_effector_poses.pose 通道）——
# joint_states 保持当前关节角不动，位姿经 end_effector_poses 下发
# （演示双通道：joint_states 位姿存在时优先，本组无 joint_states 位姿 → 走 ee 通道）
GROUP_E = {
    "name": "ch8_pose(ee通道)",
    "kind": "pose_ee_channel",
    "pose_offset": [0.0, 0.0, 0.05],
}

# 组 5：第 8 章关节控制（经部位组名路由）——
# names=["left_arm","right_arm"]，position 按每组 7 关节解析 → set_joint_position_control
# 关节目标复用 GROUP_C 真机值（左 7 + 右 7）
GROUP_F = {
    "name": "ch8_joints(组名关节)",
    "kind": "joints8_groups",
    "gripper": [0.5, 0.8],
}

# 启用第 8 章示例时的往复序列：0→1→2→1→ 3→4→5→4 → 0→1→2→1 → ...
CH8_CYCLE_SEQUENCE = [0, 1, 2, 1, 3, 4, 5, 4]


class ThreeGroupActionCycler:
    """在多组动作之间往复生成 Actions（关节插值、夹爪任意角度、可选第 8 章示例组）。"""

    def __init__(
        self,
        steps: int = INTERP_STEPS,
        sequence: Sequence[int] = CYCLE_SEQUENCE,
        include_ch8: bool = False,
        pose_offset_z: float = 0.05,
    ) -> None:
        self._steps = max(int(steps), 1)
        self._sequence = list(sequence)
        self._cycle_index = 0
        self._groups: List[Dict[str, Any]] = [GROUP_A, GROUP_B, GROUP_C]
        if include_ch8:
            group_d = dict(GROUP_D)
            group_e = dict(GROUP_E)
            for group in (group_d, group_e):
                group["pose_offset"] = [0.0, 0.0, float(pose_offset_z)]
            group_f = dict(GROUP_F)
            group_f["joints"] = list(GROUP_C["joints"])  # 复用真机关节值
            self._groups += [group_d, group_e, group_f]

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
    ) -> Optional[Actions]:
        """按下一组动作构建 Actions；第 8 章位姿组缺当前末端位姿时跳过本轮（返回 None）。"""
        group = self._groups[self.next_group_index()]
        kind = group.get("kind", "joints")

        if kind == "pose_joints_names":
            return self._build_pose_via_names(now_ms, group, current_poses, current_gripper)
        if kind == "pose_ee_channel":
            return self._build_pose_via_ee(
                now_ms, group, current_joints, current_gripper, current_poses
            )
        if kind == "joints8_groups":
            return self._build_joints8_groups(now_ms, group, current_gripper)
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
            # 夹爪用最终角度，不插值（真机调试结论）
            (gripper_target).tolist()
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
            "[%s] group=%s joints=%.3f→%.3f gripper=%s%s",
            ADAPTER_NAME,
            group["name"],
            float(start_joints[1]),
            float(target[1]),
            group["gripper"],
            (
                " head=%s waist=%s" % (group["head"], group["waist"])
                if group["kind"] == "pose"
                else ""
            ),
        )
        return Actions.from_dict(payload)

    # -- 第 8 章示例组（names 路由的三种格式） ------------------------------------

    def _offset_pose(self, pose: Sequence[float], offset: Sequence[float]) -> List[float]:
        """当前末端位姿 + xyz 偏移（姿态四元数保持）。"""
        moved = [float(v) for v in pose]
        moved[0] += float(offset[0])
        moved[1] += float(offset[1])
        moved[2] += float(offset[2])
        return moved

    def _hold_chunk(self, values: Sequence[float], steps: int) -> List[List[float]]:
        """整块保持同一目标值（每组动作执行约 2 秒的持续指令）。"""
        row = [float(v) for v in values]
        return [list(row) for _ in range(steps)]

    def _build_pose_via_names(
        self,
        now_ms: int,
        group: Dict[str, Any],
        current_poses: Optional[Dict[str, Sequence[float]]],
        current_gripper: Optional[Sequence[float]],
    ) -> Optional[Actions]:
        """组 D：末端位姿 + 夹爪均经 joint_states 逐维 names 路由。

        names = ``left_pose_x/y/z/rx/ry/rz/rw`` + ``right_pose_*`` +
        ``left_gripper`` / ``right_gripper``，position 与 names 一一对应。
        """
        poses = current_poses or {}
        left = poses.get("left")
        right = poses.get("right")
        if left is None or right is None or len(left) != POSE_DOF or len(right) != POSE_DOF:
            logger.warning(
                "[%s] group=%s missing ee poses in observation, skip this round",
                ADAPTER_NAME,
                group["name"],
            )
            return None
        offset = group["pose_offset"]
        hold_gripper = list(current_gripper) if current_gripper else [0.5, 0.8]
        names = (
            _pose_dim_names("left") + _pose_dim_names("right") + ["left_gripper", "right_gripper"]
        )
        position_row = (
            self._offset_pose(left, offset)
            + self._offset_pose(right, offset)
            + [float(v) for v in hold_gripper]
        )

        payload: Dict[str, Any] = {
            "timestamp": now_ms,
            "chunk_size": self._steps,
            "joint_states": {
                "names": names,  # names 路由 → 第 8 章末端位姿 + 夹爪
                "position": self._hold_chunk(position_row, self._steps),
                "velocity": [],
                "torque": [],
            },
            "end_effector_poses": {},
            "end_effector_states": {
                "names": list(GRIPPER_NAMES),
                "position": self._hold_chunk(hold_gripper, self._steps),
                "velocity": [],
                "torque": [],
            },
            "localization": {"odom_pose": [], "map_pose": []},
            "extensions": {},
        }
        logger.info(
            "[%s] group=%s left_pose=%s right_pose=%s gripper=%s (via joint_states names)",
            ADAPTER_NAME,
            group["name"],
            np.round(self._offset_pose(left, offset), 3).tolist(),
            np.round(self._offset_pose(right, offset), 3).tolist(),
            hold_gripper,
        )
        return Actions.from_dict(payload)

    def _build_pose_via_ee(
        self,
        now_ms: int,
        group: Dict[str, Any],
        current_joints: Optional[Sequence[float]],
        current_gripper: Optional[Sequence[float]],
        current_poses: Optional[Dict[str, Sequence[float]]],
    ) -> Optional[Actions]:
        """组 E：末端位姿经 end_effector_poses.pose 通道（joint_states 保持当前关节）。"""
        poses = current_poses or {}
        left = poses.get("left")
        right = poses.get("right")
        if left is None or right is None or len(left) != POSE_DOF or len(right) != POSE_DOF:
            logger.warning(
                "[%s] group=%s missing ee poses in observation, skip this round",
                ADAPTER_NAME,
                group["name"],
            )
            return None
        offset = group["pose_offset"]
        hold_joints = list(current_joints) if current_joints else [0.0] * ARM_DOF
        hold_gripper = list(current_gripper) if current_gripper else [0.5, 0.8]

        payload: Dict[str, Any] = {
            "timestamp": now_ms,
            "chunk_size": self._steps,
            "joint_states": {
                "names": list(JOINT_NAMES),  # 关节名路由 → 第 7 章（保持不动）
                "position": self._hold_chunk(hold_joints, self._steps),
                "velocity": [],
                "torque": [],
            },
            # names 无位姿名 → joint_states 位姿不存在，走 ee 通道（优先级兜底演示）
            "end_effector_poses": {
                "left": {"pose": [self._offset_pose(left, offset)]},
                "right": {"pose": [self._offset_pose(right, offset)]},
            },
            "end_effector_states": {
                "names": list(GRIPPER_NAMES),
                "position": self._hold_chunk(hold_gripper, self._steps),
                "velocity": [],
                "torque": [],
            },
            "localization": {"odom_pose": [], "map_pose": []},
            "extensions": {},
        }
        logger.info(
            "[%s] group=%s ee_left=%s ee_right=%s (via end_effector_poses channel)",
            ADAPTER_NAME,
            group["name"],
            np.round(self._offset_pose(left, offset), 3).tolist(),
            np.round(self._offset_pose(right, offset), 3).tolist(),
        )
        return Actions.from_dict(payload)

    def _build_joints8_groups(
        self,
        now_ms: int,
        group: Dict[str, Any],
        current_gripper: Optional[Sequence[float]],
    ) -> Actions:
        """组 F：第 8 章关节控制经部位组名路由（names=left_arm/right_arm，每组 7 关节）。"""
        joints = np.asarray(group["joints"], dtype=float)
        position_row = joints[:7].tolist() + joints[7:14].tolist()
        hold_gripper = list(current_gripper) if current_gripper else list(group["gripper"])

        payload: Dict[str, Any] = {
            "timestamp": now_ms,
            "chunk_size": self._steps,
            "joint_states": {
                "names": ["left_arm", "right_arm"],  # 组名路由 → 第 8 章关节控制
                "position": self._hold_chunk(position_row, self._steps),
                "velocity": [],
                "torque": [],
            },
            "end_effector_poses": {},
            "end_effector_states": {
                "names": list(GRIPPER_NAMES),
                "position": self._hold_chunk(hold_gripper, self._steps),
                "velocity": [],
                "torque": [],
            },
            "localization": {"odom_pose": [], "map_pose": []},
            "extensions": {},
        }
        logger.info(
            "[%s] group=%s left_arm=%s right_arm=%s (via set_joint_position_control)",
            ADAPTER_NAME,
            group["name"],
            np.round(joints[:7], 3).tolist(),
            np.round(joints[7:], 3).tolist(),
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
        "--include-ch8",
        action="store_true",
        help="Include chapter-8 demo groups (ee pose via names/ee-channel, group joints)",
    )
    parser.add_argument(
        "--pose-offset-z",
        type=float,
        default=0.05,
        help="Z offset (m) applied to current ee poses in chapter-8 pose groups",
    )
    return parser.parse_args()


def _extract_current(observation: Any) -> Dict[str, Any]:
    """从观测中提取当前关节/夹爪/末端位姿，缺失时返回 None/空。"""
    try:
        joints = list(observation.joint_states.position) or None
    except Exception:
        joints = None
    try:
        gripper = list(observation.end_effector_states.position) or None
    except Exception:
        gripper = None
    poses: Dict[str, List[float]] = {}
    try:
        names = list(observation.end_effector_poses.names)
        values = list(observation.end_effector_poses.pose)
        for name, pose in zip(names, values):
            poses[str(name)] = [float(v) for v in pose]
    except Exception:
        poses = {}
    return {"joints": joints, "gripper": gripper, "poses": poses}


def main() -> None:
    args = parse_args()
    if args.include_ch8:
        # 0→1→2→1 → 3→4→5→4 → ...（第 7/8 章示例一起往复）
        sequence = CH8_CYCLE_SEQUENCE
    else:
        sequence = CYCLE_SEQUENCE
    cycler = ThreeGroupActionCycler(
        steps=args.steps,
        sequence=sequence,
        include_ch8=args.include_ch8,
        pose_offset_z=args.pose_offset_z,
    )

    config = ClientConfig(
        project_id=args.project_id,
        device_id=args.device_id,
        client_id=args.client_id,
        endpoint_role="connect",
        endpoints=["tcp/127.0.0.1:7447"],
        mode="peer",
    )
    logger.info(
        "[%s] Connecting: project=%s, device=%s, client=%s",
        ADAPTER_NAME,
        args.project_id,
        args.device_id,
        args.client_id,
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
        if action is None:
            return  # 第 8 章位姿组缺观测位姿时跳过本轮
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
