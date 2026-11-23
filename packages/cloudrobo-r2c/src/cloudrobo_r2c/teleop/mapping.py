"""TeleopTarget → Actions 映射，以及 Observations → 录制 dict 的展平。

映射职责（纯函数，可单测）：

- ``build_joint_action``：关节目标（源关节键 → 机器人 R2C 关节名）→
  ``Actions.joint_states``（chunk_size=1，position 为单步 2D 列表）。
- ``build_ee_action``：末端位姿目标 → ``Actions.end_effector_poses``；
  夹爪按配置附加到 ``joint_states``（``emit: joint``）或
  ``end_effector_states``（``emit: end_effector_state``）。
- ``observation_to_dict``：把订阅到的 Observations / ObservationsH264
  展平为可录制 dict（图像保持原始编码 bytes，供 safe_serialization 落盘）。
"""

from __future__ import annotations

import logging
from dataclasses import asdict, is_dataclass
from typing import Any, Dict, Mapping, Optional, Sequence, Tuple

from cloudrobo_r2c.common.models import Actions, Pose7D
from cloudrobo_r2c.common.models.wrappers.action import (
    EndEffectorPoseChunk,
    EndEffectorStateAction,
    JointAction,
)

logger = logging.getLogger(__name__)


def build_joint_action(
    *,
    timestamp_ms: int,
    seq: int,
    names: Sequence[str],
    positions: Sequence[float],
) -> Actions:
    """构造单步关节动作（chunk_size=1）。

    ``positions`` 与 ``names`` 一一对应；机器人侧 ``r2c_to_device`` 映射按
    ``source_index`` 取值，因此顺序即语义。
    """
    if len(names) != len(positions):
        raise ValueError(
            f"joint names/positions length mismatch: {len(names)} != {len(positions)}"
        )
    return Actions(
        timestamp=timestamp_ms,
        id=seq,
        chunk_size=1,
        joint_states=JointAction(
            names=[str(n) for n in names],
            position=[[float(v) for v in positions]],
        ),
    )


def build_ee_action(
    *,
    timestamp_ms: int,
    seq: int,
    pose_map: Mapping[str, Sequence[float]],
    gripper: Optional[float] = None,
    gripper_emit: str = "joint",
    gripper_joint_name: str = "gripper",
    gripper_range: Tuple[float, float] = (0.0, 1.0),
) -> Actions:
    """构造末端位姿动作（chunk_size=1）。

    ``pose_map``：R2C 末端名 → Pose7D（7 元素 [tx,ty,tz,qx,qy,qz,qw]）。
    ``gripper``：归一化开度 0~1，映射到 ``gripper_range`` 后按 ``gripper_emit`` 输出：
    ``joint`` → ``joint_states`` 附加关节；``end_effector_state`` →
    ``end_effector_states`` 单字段；``none`` → 不输出。
    """
    ee_poses: Dict[str, EndEffectorPoseChunk] = {}
    for name, pose in pose_map.items():
        values = [float(v) for v in pose]
        if len(values) != 7:
            raise ValueError(
                f"end effector pose for {name!r} must have 7 values "
                f"[tx,ty,tz,qx,qy,qz,qw], got {len(values)}"
            )
        ee_poses[str(name)] = EndEffectorPoseChunk(pose=[Pose7D(values)])

    joint_states = JointAction([], [], [], [])
    ee_states = EndEffectorStateAction([], [], [], [])
    if gripper is not None and gripper_emit == "joint":
        lo, hi = float(gripper_range[0]), float(gripper_range[1])
        value = lo + (hi - lo) * _clamp01(gripper)
        joint_states = JointAction(names=[str(gripper_joint_name)], position=[[value]])
    elif gripper is not None and gripper_emit == "end_effector_state":
        lo, hi = float(gripper_range[0]), float(gripper_range[1])
        value = lo + (hi - lo) * _clamp01(gripper)
        ee_states = EndEffectorStateAction(
            names=[str(gripper_joint_name)], position=[[value]]
        )

    return Actions(
        timestamp=timestamp_ms,
        id=seq,
        chunk_size=1,
        joint_states=joint_states,
        end_effector_poses=ee_poses,
        end_effector_states=ee_states,
    )


def _clamp01(value: float) -> float:
    return 0.0 if value < 0.0 else (1.0 if value > 1.0 else float(value))


def observation_to_dict(observation: Any) -> Dict[str, Any]:
    """把 Observations / ObservationsH264（或 dict）展平为可录制结构。

    - 图像字段保持原始编码 bytes（safe_serialization 以 bytes 块落盘，
      回放/转数据集时按 mime 自行解码）。
    - 未知结构按 dataclass/dict/基础类型尽力转换，保证录制永不因观测
      字段变化而失败。
    """
    if isinstance(observation, Mapping):
        return _to_plain(observation)
    if is_dataclass(observation) and not isinstance(observation, type):
        return _to_plain(asdict(observation))
    # 尽力兜底：有 to_dict/to_dict 方法则用之，否则 str 化
    for method_name in ("to_dict", "as_dict"):
        method = getattr(observation, method_name, None)
        if callable(method):
            try:
                return _to_plain(method())
            except Exception:
                logger.debug("observation.%s() failed", method_name, exc_info=True)
    return {"repr": repr(observation)}


def _to_plain(value: Any) -> Any:
    """递归转换为 safe_serialization 支持的结构（dict/list/scalar/bytes/ndarray）。"""
    if isinstance(value, Mapping):
        return {str(k): _to_plain(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_to_plain(v) for v in value]
    if isinstance(value, (str, bool, int, float, bytes, bytearray, memoryview)) or value is None:
        return value
    # numpy 标量 / ndarray 原样透传（safe_serialization 支持 ndarray 与标量）
    module = getattr(type(value), "__module__", "") or ""
    if module.startswith("numpy"):
        return value
    if is_dataclass(value) and not isinstance(value, type):
        return _to_plain(asdict(value))
    if isinstance(value, Pose7D):
        return list(value.data)
    return str(value)
