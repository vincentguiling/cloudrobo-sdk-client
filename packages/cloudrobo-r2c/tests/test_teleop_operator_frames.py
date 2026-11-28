"""TeleopFrame construction + roundtrip + extensions tests."""

import pytest
from cloudrobo_r2c.common.models.generated import teleop_pb2
from cloudrobo_r2c.teleop.frame_builder import (
    build_action_frame,
    build_command_frame,
    serialize,
)
from cloudrobo_r2c.teleop.pose_math import Pose


def test_command_frame_roundtrip():
    frame = build_command_frame("start", session_id="s-1", operator_id="op")
    parsed = teleop_pb2.TeleopFrame.FromString(serialize(frame))
    assert parsed.WhichOneof("command") == "start"
    assert parsed.session_id == "s-1"
    assert parsed.operator_id == "op"
    assert parsed.input_mode == teleop_pb2.InputMode.INPUT_MODE_VR


def test_command_frame_start_matrices_roundtrip():
    mat = [1, 0, 0, 0, 1, 0, 0, 0, 1]
    frame = build_command_frame(
        "start",
        session_id="s",
        start={"pos_left": mat, "rot_left": [0, -1, 0, 1, 0, 0, 0, 0, 1]},
    )
    parsed = teleop_pb2.TeleopFrame.FromString(serialize(frame))
    assert list(parsed.start.pos_matrix_left.values) == pytest.approx(mat)
    assert len(parsed.start.rot_matrix_left.values) == 9
    # unset matrices stay empty
    assert len(parsed.start.pos_matrix_right.values) == 0


def test_action_frame_roundtrip_with_extensions():
    hands = {
        "left": (Pose([0.5, 0.1, 0.3], [0, 0, 0, 1]), 0.25),
        "right": (Pose([0.5, -0.1, 0.3], [0, 0, 0, 1]), 0.75),
    }
    frame = build_action_frame(
        "sess",
        "op",
        hands,
        raw_inputs={
            "left": {"primary": True, "thumbstick": False, "thumb": [0.1, -0.2]},
            "right": {"primary": False, "thumbstick": True, "thumb": [0.0, 0.0]},
        },
    )
    parsed = teleop_pb2.TeleopFrame.FromString(serialize(frame))
    assert parsed.WhichOneof("command") is None
    assert parsed.session_id == "sess"
    assert set(parsed.end_effector_poses.pose.keys()) == {"left", "right"}
    left = list(parsed.end_effector_poses.pose["left"].pose.data)
    assert left == pytest.approx([0.5, 0.1, 0.3, 0.0, 0.0, 0.0, 1.0])
    assert list(parsed.end_effector_states.names) == ["left", "right"]
    assert list(parsed.end_effector_states.position.data) == pytest.approx([0.25, 0.75])
    ext = parsed.extensions
    assert ext["left.btn.primary"].dtype == 6  # BOOL
    assert ext["left.btn.primary"].data == b"\x01"
    assert ext["right.btn.primary"].data == b"\x00"
    # float64 little-endian payload
    import struct

    assert struct.unpack("<d", ext["left.stick.x"].data)[0] == pytest.approx(0.1)


def test_action_frame_omits_hands_not_passed():
    frame = build_action_frame("s", "op", {})
    parsed = teleop_pb2.TeleopFrame.FromString(serialize(frame))
    assert len(parsed.end_effector_poses.pose) == 0
    assert len(parsed.end_effector_states.names) == 0


def test_command_frame_rejects_unknown_kind():
    with pytest.raises(ValueError):
        build_command_frame("bogus", session_id="")


def test_input_mode_override():
    frame = build_action_frame(
        "s", "op", {}, input_mode=teleop_pb2.InputMode.INPUT_MODE_ISOMORPHIC
    )
    assert frame.input_mode == teleop_pb2.InputMode.INPUT_MODE_ISOMORPHIC
