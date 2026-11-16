"""teleop/actions → 设备侧动作转换。

把 ``teleop.proto`` 的 ``TeleopFrame`` 转换为硬件适配器 ``send_action``
接受的平铺字段（与 AgiBot G01 等适配器的字段约定一致）：

- 末端绝对位姿 ``end_effector_poses``（names + Pose7D）→
  ``left_pose_cmd_ee`` / ``right_pose_cmd_ee``（7 维 ``[x,y,z,qx,qy,qz,qw]``）
- 触发值 ``end_effector_states``（每手 0..1）→ ``gripper_cmd_pos``（2 通道）
- ``joint_states``（teleop.proto 预留，当前为空；非空时）→
  ``joint_names`` / ``arm_cmd_joint_pos``（逐维透传）
- 生命周期 ``command`` 帧（start/pause/resume/finish）不驱动设备，输出
  ``teleop_command`` 供上层会话门控

遥操语义（grip 未按下的手不进入消息）由操作端保证：缺失侧保持不动；夹爪缺失侧
用调用方传入的 ``current_gripper``（设备当前 [left, right]）补齐，无法补齐时
跳过夹爪下发（避免误动未按 grip 的手）。

用法（机器人端把 teleop 帧直接下发给硬件适配器）::

    from cloudrobo_r2c.common.models import TeleopFrame
    from cloudrobo_r2c.translators import teleop_to_device

    def on_teleop(frame: TeleopFrame) -> None:
        command = teleop_to_device(frame, current_gripper=adapter_last_gripper)
        if command:
            adapter.send_action(command)
"""

from __future__ import annotations

import logging
from typing import Any, Dict, List, Optional, Sequence

logger = logging.getLogger(__name__)

# 位姿/触发通道的左右侧名称（teleop.proto 复用 observation.proto 的
# EndEffectorPoses/EndEffectorStates，names 约定 left/right；兼容 arm 命名）
_SIDE_ALIASES = {
    "left": "left",
    "right": "right",
    "left_arm": "left",
    "right_arm": "right",
}

POSE_DOF = 7
GRIPPER_DOF = 2


def _side_of(name: Any) -> Optional[str]:
    return _SIDE_ALIASES.get(str(name).strip().lower())


def _pose_values(pose: Any) -> List[float]:
    data = getattr(pose, "data", pose)
    return [float(v) for v in data]


def teleop_to_device(
    frame: Any,
    current_gripper: Optional[Sequence[float]] = None,
) -> Dict[str, Any]:
    """TeleopFrame → 设备侧平铺 command dict（空 dict 表示无设备动作）。

    Args:
        frame: ``TeleopFrame``（``teleop/actions`` 解码结果）。
        current_gripper: 设备当前夹爪值 ``[left, right]``（``[0, 1]`` 归一化），
            用于补齐单侧触发帧（另一只手未按 grip、消息里不出现）。

    Returns:
        ``send_action`` 可消费的 dict：``left_pose_cmd_ee`` / ``right_pose_cmd_ee``、
        ``gripper_cmd_pos``、``joint_names`` / ``arm_cmd_joint_pos``、
        ``teleop_command``；无设备动作时返回 ``{}``。
    """
    command: Dict[str, Any] = {}
    if frame is None:
        return command

    command_name = getattr(frame, "command", None)
    if command_name:
        # 生命周期命令帧：不驱动设备，交给上层会话状态机（start/pause/...）
        command["teleop_command"] = str(command_name)
        return command

    poses = getattr(frame, "end_effector_poses", None)
    if poses is not None:
        names = list(getattr(poses, "names", []) or [])
        pose_values = list(getattr(poses, "pose", []) or [])
        for name, pose in zip(names, pose_values):
            side = _side_of(name)
            if side is None:
                logger.warning("[teleop_to_device] unknown pose side %r; skipped", name)
                continue
            values = _pose_values(pose)
            if len(values) != POSE_DOF:
                logger.warning(
                    "[teleop_to_device] %s pose expects %d values, got %d; skipped",
                    side,
                    POSE_DOF,
                    len(values),
                )
                continue
            command[f"{side}_pose_cmd_ee"] = values

    states = getattr(frame, "end_effector_states", None)
    if states is not None:
        left: Optional[float] = None
        right: Optional[float] = None
        names = list(getattr(states, "names", []) or [])
        position = list(getattr(states, "position", []) or [])
        for name, value in zip(names, position):
            side = _side_of(name)
            if side is None:
                logger.warning("[teleop_to_device] unknown trigger side %r; skipped", name)
                continue
            if side == "left":
                left = float(value)
            else:
                right = float(value)
        gripper = _resolve_gripper(left, right, current_gripper)
        if gripper is not None:
            command["gripper_cmd_pos"] = gripper

    joint_states = getattr(frame, "joint_states", None)
    if joint_states is not None:
        joint_names = list(getattr(joint_states, "names", []) or [])
        joint_position = [float(v) for v in (getattr(joint_states, "position", []) or [])]
        if joint_names and joint_position:
            command["joint_names"] = joint_names
            command["arm_cmd_joint_pos"] = joint_position

    return command


def _resolve_gripper(
    left: Optional[float],
    right: Optional[float],
    current_gripper: Optional[Sequence[float]],
) -> Optional[List[float]]:
    """补齐单侧触发帧的夹爪 2 通道值；无法补齐时返回 ``None``（跳过下发）。"""
    if left is not None and right is not None:
        return [left, right]
    if left is None and right is None:
        return None
    hold = list(current_gripper) if current_gripper is not None else None
    if hold is None or len(hold) != GRIPPER_DOF:
        logger.debug(
            "[teleop_to_device] single-side trigger without valid current_gripper; skipped"
        )
        return None
    if left is None:
        left = float(hold[0])
    if right is None:
        right = float(hold[1])
    return [left, right]
