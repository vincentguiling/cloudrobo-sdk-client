"""AgiBot G01 样例 cloud-adapter：在 3 组动作之间来回运动。

配合机器人端 ``config/robot_agibot_g01_config.yaml`` 使用，演示：

- **关节 + 位姿** 两种动作空间：组 1/2 为关节组（双臂 14 关节），组 3 为末端位姿组
  （左右末端 7 维位姿 ``[x, y, z, qx, qy, qz, qw]``，基于当前位姿加 z 向偏移）
- **夹爪任意角度**：夹爪值连续任意（默认归一化 ``[0, 1]``，也可用 ``[35, 120]``
  原始值），随组切换到不同开度
- **往复运动**：组序列 ``0 → 1 → 2 → 1 → 0 → ...`` 来回循环；关节组从当前关节角
  线性插值到目标（默认 15 步/chunk），避免目标跳变超过 0.2618 rad（15°）限幅

用法示例：

    python examples/agibot_g01_cloud_adapter.py
    python examples/agibot_g01_cloud_adapter.py --project-id test-tenant --device-id agibot-g01-01
    python examples/agibot_g01_cloud_adapter.py --duration 60 --steps 15 --pose-offset-z 0.05

安全提示：机器人端请先以 ``runtime.dry_run: true`` 验证数据链路与动作序列，
确认无误后再改为 ``false`` 真实执行。
"""

from __future__ import annotations

import argparse
import logging
import time
from typing import Any, Dict, List, Optional, Sequence

import numpy as np
from _cloud_adapter_common import build_parser, normalize_for_logging
from cloudrobo_r2c import ClientConfig, R2CClient
from cloudrobo_r2c.common.models import Actions

logger = logging.getLogger(__name__)

ADAPTER_NAME = "AgiBotG01CloudAdapter"
DEFAULT_PROJECT_ID = "test-tenant"
DEFAULT_DEVICE_ID = "agibot-g01-01"
DEFAULT_CLIENT_ID = "agibot-g01-cloud-adapter"

# 与 config/robot_agibot_g01_config.yaml 的 joint_states.names 保持一致
JOINT_NAMES = [f"left_arm_joint_{i}" for i in range(1, 8)] + [
    f"right_arm_joint_{i}" for i in range(1, 8)
]
GRIPPER_NAMES = ["left_gripper", "right_gripper"]

ARM_DOF = 14
GRIPPER_DOF = 2
POSE_DOF = 7
INTERP_STEPS = 15  # 关节组插值步数（每步在 publish_hz 节拍下执行）

# ---------------------------------------------------------------------------
# 3 组动作定义（现场请按机器人实际位姿/限位调整目标值）
# ---------------------------------------------------------------------------

# 组 0：关节位姿 A —— 双臂微屈的"收拢"姿态，夹爪小开度
JOINT_GROUP_A = {
    "name": "joint_a(收拢)",
    "kind": "joints",
    "joints": [0.0] * ARM_DOF,
    "gripper": [0.15, 0.15],  # 任意角度示例：连续小数值
}

# 组 1：关节位姿 B —— 双臂抬起展开姿态，夹爪大开度（左右不同角度）
JOINT_GROUP_B = {
    "name": "joint_b(展开)",
    "kind": "joints",
    "joints": [
        # left arm 1..7（示例值，现场按限位调整）
        0.0,
        0.35,
        -0.55,
        0.9,
        0.0,
        0.3,
        0.0,
        # right arm 1..7
        0.0,
        -0.35,
        -0.55,
        0.9,
        0.0,
        -0.3,
        0.0,
    ],
    "gripper": [0.85, 0.35],  # 左右夹爪各自任意角度
}

# 组 2：末端位姿组 —— 基于当前左右末端位姿加 z 向偏移（无需预知工作坐标）
POSE_GROUP = {
    "name": "pose(末端位姿)",
    "kind": "pose",
    "pose_offset": [0.0, 0.0, 0.05],  # 末端 [x, y, z] 偏移（米），姿态保持不变
    "gripper": [0.5, 0.8],
}

# 往复序列：0 → 1 → 2 → 1 → 0 → 1 → 2 → ...（来回）
CYCLE_SEQUENCE = [0, 1, 2, 1]


class ThreeGroupActionCycler:
    """在 3 组动作之间往复生成 Actions（关节组插值、位姿组偏移、夹爪任意角度）。"""

    def __init__(
        self,
        steps: int = INTERP_STEPS,
        pose_offset_z: float = 0.05,
        sequence: Sequence[int] = CYCLE_SEQUENCE,
    ) -> None:
        self._steps = max(int(steps), 1)
        self._sequence = list(sequence)
        self._cycle_index = 0
        pose_group = dict(POSE_GROUP)
        pose_group["pose_offset"] = [0.0, 0.0, float(pose_offset_z)]
        self._groups = [JOINT_GROUP_A, JOINT_GROUP_B, pose_group]

    def next_group_index(self) -> int:
        index = self._sequence[self._cycle_index % len(self._sequence)]
        self._cycle_index += 1
        return index

    def build_action(
        self,
        now_ms: int,
        current_joints: Optional[Sequence[float]],
        current_gripper: Optional[Sequence[float]],
        current_poses: Dict[str, Sequence[float]],
    ) -> Optional[Actions]:
        """按下一组动作构建 Actions；位姿组缺当前位姿时跳过该组。"""
        group_index = self.next_group_index()
        group = self._groups[group_index]

        if group["kind"] == "pose":
            return self._build_pose_action(now_ms, group, current_poses)
        return self._build_joint_action(now_ms, group, current_joints, current_gripper)

    # -- 关节组：当前 → 目标线性插值 ------------------------------------------------

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
        }
        logger.info(
            "[%s] group=%s joints=%.3f→%.3f gripper=%s",
            ADAPTER_NAME,
            group["name"],
            float(start_joints[1]),
            float(target[1]),
            group["gripper"],
        )
        return Actions.from_dict(payload)

    # -- 位姿组：当前末端位姿 + 偏移 ------------------------------------------------

    def _build_pose_action(
        self,
        now_ms: int,
        group: Dict[str, Any],
        current_poses: Dict[str, Sequence[float]],
    ) -> Optional[Actions]:
        offset = np.asarray(group["pose_offset"], dtype=float)
        poses: Dict[str, List[float]] = {}
        for side in ("left", "right"):
            pose = current_poses.get(side)
            if pose is None or len(pose) != POSE_DOF:
                logger.warning(
                    "[%s] group=%s missing %s ee pose, skip this round",
                    ADAPTER_NAME,
                    group["name"],
                    side,
                )
                return None
            pose = np.asarray(pose, dtype=float)
            pose[:3] += offset  # 仅平移，姿态四元数保持
            poses[side] = pose.tolist()

        payload: Dict[str, Any] = {
            "timestamp": now_ms,
            "chunk_size": 1,
            "joint_states": {
                "names": list(JOINT_NAMES),
                "position": [],
                "velocity": [],
                "torque": [],
            },
            "end_effector_poses": {
                "left": {"pose": [poses["left"]]},
                "right": {"pose": [poses["right"]]},
            },
            "end_effector_states": {
                "names": list(GRIPPER_NAMES),
                "position": [list(group["gripper"])],
                "velocity": [],
                "torque": [],
            },
            "localization": {"odom_pose": [], "map_pose": []},
        }
        logger.info(
            "[%s] group=%s left_pose=%s gripper=%s",
            ADAPTER_NAME,
            group["name"],
            np.round(poses["left"], 3).tolist(),
            group["gripper"],
        )
        return Actions.from_dict(payload)


def parse_args() -> argparse.Namespace:
    parser = build_parser(
        description="AgiBot G01 cloud adapter demo: cycle between 3 action groups",
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
        help="Interpolation steps per joint group (chunk size)",
    )
    parser.add_argument(
        "--pose-offset-z",
        type=float,
        default=POSE_GROUP["pose_offset"][2],
        help="Z offset (m) applied to current ee poses in the pose group",
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

    poses: Dict[str, Sequence[float]] = {}
    try:
        names = list(observation.end_effector_poses.names)
        values = list(observation.end_effector_poses.pose)
        for name, pose in zip(names, values):
            poses[str(name)] = list(pose)
    except Exception:
        poses = {}
    return {"joints": joints, "gripper": gripper, "poses": poses}


def main() -> None:
    args = parse_args()
    cycler = ThreeGroupActionCycler(steps=args.steps, pose_offset_z=args.pose_offset_z)

    config = ClientConfig(
        project_id=args.project_id,
        device_id=args.device_id,
        client_id=args.client_id,
        endpoint_role="listen",
        endpoints=["tcp/0.0.0.0:7447"],
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
            return
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
