"""AgiBot G01 样例 cloud-adapter：在 3 组动作之间来回运动（仅第 7 章接口）。

配合机器人端 ``config/robot_agibot_g01_config.yaml`` 使用，全部动作只使用
《智元精灵G01+GDK v1.5.0使用指南》第 7 章 ``RobotDds`` 关节空间接口
（``move_arm`` / ``move_head`` / ``move_waist`` / ``move_gripper``）：

- **关节 + 位姿**：组 0/1 为纯关节组（双臂 14 关节 + 夹爪）；组 2 为全身位姿组
  （手臂 + 头部 ``[yaw, pitch]`` + 腰部 ``[pitch, height_cm]`` 一起到达新姿态，
  即第 7 章语义下的"整体姿态"）
- **夹爪任意角度**：夹爪值连续任意（默认归一化 ``[0, 1]``，也可用 ``[35, 120]``
  原始值），随组切换到不同开度
- **往复运动**：组序列 ``0 → 1 → 2 → 1 → 0 → ...`` 来回循环；关节部分从当前
  关节角线性插值到目标（默认 15 步/chunk），避免目标跳变超过 0.2618 rad（15°）限幅

头部/腰部目标经 ``Actions.extensions``（FLOAT32 一维数组）下发，由机器人端适配器
解码后调用 ``move_head`` / ``move_waist``。

用法示例：

    python examples/agibot_g01_cloud_adapter.py
    python examples/agibot_g01_cloud_adapter.py --project-id test-tenant --device-id agibot-g01-01
    python examples/agibot_g01_cloud_adapter.py --duration 60 --steps 15

安全提示：机器人端请先以 ``runtime.dry_run: true`` 验证数据链路与动作序列，
确认无误后再改为 ``false`` 真实执行。
"""

from __future__ import annotations

import argparse
import logging
import struct
import time
from typing import Any, Dict, Optional, Sequence

import numpy as np
from _cloud_adapter_common import build_parser, normalize_for_logging
from cloudrobo_r2c import ClientConfig, R2CClient
from cloudrobo_r2c.common.models import Actions

logger = logging.getLogger(__name__)

ADAPTER_NAME = "AgiBotG01CloudAdapter"
DEFAULT_PROJECT_ID = "r2c"
DEFAULT_DEVICE_ID = "local"
DEFAULT_CLIENT_ID = "agibot-g01-cloud-adapter"

# 与 config/robot_agibot_g01_config.yaml 的 joint_states.names 保持一致
JOINT_NAMES = [f"left_arm_joint_{i}" for i in range(1, 8)] + [
    f"right_arm_joint_{i}" for i in range(1, 8)
]
GRIPPER_NAMES = ["left_gripper", "right_gripper"]

ARM_DOF = 14
GRIPPER_DOF = 2
HEAD_DOF = 2
WAIST_DOF = 2
INTERP_STEPS = 60  # 关节插值步数（每步在 publish_hz 节拍下执行2s）


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
    "joints": [-1.125699520111084, 0.5228543281555176, 0.10490940511226654, -1.283866286277771, 0.7302126884460449, 1.476741075515747, -0.16992810368537903, 1.1257692575454712, -0.522889256477356, -0.10499665141105652, 1.2836743593215942, -0.7302126884460449, -1.495011329650879, 0.16991065442562103],
    "gripper": [0.15, 0.15],  # 任意角度示例：连续小数值
}

# 组 1：关节组 B —— 双臂抬起展开姿态，夹爪大开度（左右不同角度）
GROUP_B = {
    "name": "joint_b(展开)",
    "kind": "joints",
    "joints": [-1.187420129776001, 0.5014432072639465, 0.7781304121017456, -1.119522213935852, 0.6628905534744263, 1.0817254781723022, -0.25557270646095276, 1.1248793601989746, -0.9522988796234131, -0.04086789861321449, 0.5556254386901855, -1.3621994256973267, -1.5153056383132935, 0.16874149441719055],
    "gripper": [0.85, 0.35],  # 左右夹爪各自任意角度
}
# 组 2：全身位姿组 —— 手臂 + 头部 + 腰部一起到达新姿态（第 7 章"整体姿态"语义）
GROUP_C = {
    "name": "pose(全身姿态)",
    "kind": "joints",
    "joints": [-1.1872631311416626, 0.7889842987060547, 1.6079477071762085, -1.1197315454483032, 0.6622798442840576, 0.26489099860191345, -0.2558518946170807, 1.1237276792526245, -0.9279735684394836, -0.01715334877371788, 0.14202554523944855, -1.5966051816940308, -1.3580461740493774, 0.20629389584064484],
    # 头部 [yaw(rad), pitch(rad)]，腰部 [pitch(rad), height(cm)]（《指南》7.2.1）
    "head": [0.1, 0.1],
    "waist": [0.1, 10.0],
    "gripper": [0.5, 0.8],
}

# 往复序列：0 → 1 → 2 → 1 → 0 → 1 → 2 → ...（来回）
CYCLE_SEQUENCE = [0, 1, 2, 1]


class ThreeGroupActionCycler:
    """在 3 组动作之间往复生成 Actions（关节插值、全身姿态组、夹爪任意角度）。"""

    def __init__(
        self,
        steps: int = INTERP_STEPS,
        sequence: Sequence[int] = CYCLE_SEQUENCE,
    ) -> None:
        self._steps = max(int(steps), 1)
        self._sequence = list(sequence)
        self._cycle_index = 0
        self._groups = [GROUP_A, GROUP_B, GROUP_C]

    def next_group_index(self) -> int:
        index = self._sequence[self._cycle_index % len(self._sequence)]
        self._cycle_index += 1
        return index

    def build_action(
        self,
        now_ms: int,
        current_joints: Optional[Sequence[float]],
        current_gripper: Optional[Sequence[float]],
    ) -> Actions:
        """按下一组动作构建 Actions（关节部分线性插值，全身姿态组附带头/腰目标）。"""
        group = self._groups[self.next_group_index()]

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

        if group["kind"] == "pose":
            # 全身姿态组：头/腰目标经 extensions 下发（《指南》7.2.1 move_head/move_waist）
            payload["extensions"]["head_joint"] = _pack_ext(group["head"])
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


def parse_args() -> argparse.Namespace:
    parser = build_parser(
        description="AgiBot G01 cloud adapter demo: cycle between 3 action groups (chapter-7 joints only)",
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
        help="Interpolation steps per group (chunk size)",
    )
    return parser.parse_args()


def _extract_current(observation: Any) -> Dict[str, Any]:
    """从观测中提取当前关节/夹爪，缺失时返回 None。"""
    try:
        joints = list(observation.joint_states.position) or None
    except Exception:
        joints = None
    try:
        gripper = list(observation.end_effector_states.position) or None
    except Exception:
        gripper = None
    return {"joints": joints, "gripper": gripper}


def main() -> None:
    args = parse_args()
    cycler = ThreeGroupActionCycler(steps=args.steps)

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
