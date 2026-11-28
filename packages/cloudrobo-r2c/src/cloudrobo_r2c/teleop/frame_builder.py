"""TeleopFrame construction (port of core/TeleopFrameBuilder.cs).

Frame rules (design §3.3.1.5):
  * command frames carry a frame-level ``command`` oneof and no pose data;
  * action frames carry per-hand Pose7D + trigger values, one chunk per frame;
  * hands without grip pressed are omitted (absence IS the deadman signal);
  * extensions carry raw headset inputs with the documented key groups:
    <hand>.btn.primary / <hand>.btn.thumbstick / <hand>.stick.x|y.
"""

from __future__ import annotations

import struct
from typing import Mapping, Optional

from .pose_math import Pose, pose7d

# DType values (common.proto)
_DTYPE_FLOAT64 = 2
_DTYPE_BOOL = 6

_COMMAND_FIELDS = {
    "start": "start",
    "pause": "pause",
    "resume": "resume",
    "finish": "finish",
    "quit": "quit",
}


def _teleop_pb():
    try:
        from ..common.models.generated import teleop_pb2
    except Exception as exc:  # pragma: no cover - generated at build time
        raise RuntimeError(
            "protobuf generated modules missing; run the package build first "
            f"(compile_protobuf): {exc}"
        ) from exc
    return teleop_pb2


def build_command_frame(
    kind: str,
    *,
    session_id: str = "",
    operator_id: str = "",
    input_mode: Optional[int] = None,
    start: Optional[Mapping[str, list[float]]] = None,
) -> object:
    """Build a lifecycle command frame (RELIABLE publisher).

    ``start`` optionally carries per-hand transform matrices:
    {"pos_left": [9], "rot_left": [9], "pos_right": [9], "rot_right": [9]}.
    """
    pb = _teleop_pb()
    frame = pb.TeleopFrame()
    frame.timestamp = int(__import__("time").time() * 1000)
    frame.operator_id = operator_id
    frame.session_id = session_id
    frame.input_mode = (
        input_mode if input_mode is not None else pb.InputMode.INPUT_MODE_VR
    )
    field = _COMMAND_FIELDS.get(kind)
    if field is None:
        raise ValueError(f"unknown teleop command: {kind!r}")
    sub = getattr(frame, field)
    if field == "start" and start:
        mat_map = {
            "pos_left": "pos_matrix_left",
            "rot_left": "rot_matrix_left",
            "pos_right": "pos_matrix_right",
            "rot_right": "rot_matrix_right",
        }
        for key, attr in mat_map.items():
            values = start.get(key)
            if values and len(values) == 9:
                getattr(sub, attr).values.extend(float(v) for v in values)
    sub.SetInParent()
    return frame


def build_action_frame(
    session_id: str,
    operator_id: str,
    hands: Mapping[str, tuple[Pose, float]],
    raw_inputs: Optional[Mapping[str, Mapping[str, object]]] = None,
    *,
    input_mode: Optional[int] = None,
) -> object:
    """Build an action frame (BEST_EFFORT publisher).

    ``hands`` maps hand name -> (pose, trigger). Hands without grip must not
    be present (the gateway only includes following hands).
    """
    pb = _teleop_pb()
    frame = pb.TeleopFrame()
    frame.timestamp = int(__import__("time").time() * 1000)
    frame.operator_id = operator_id
    frame.session_id = session_id
    frame.input_mode = (
        input_mode if input_mode is not None else pb.InputMode.INPUT_MODE_VR
    )
    for hand, (pose, trigger) in hands.items():
        frame.end_effector_poses.pose[hand].pose.data.extend(pose7d(pose))
        frame.end_effector_states.names.append(hand)
        frame.end_effector_states.position.data.append(float(trigger))
    if raw_inputs:
        for hand in ("left", "right"):
            ext = raw_inputs.get(hand)
            if not ext:
                continue
            _put_bool(frame.extensions, f"{hand}.btn.primary", bool(ext.get("primary")))
            _put_bool(
                frame.extensions, f"{hand}.btn.thumbstick", bool(ext.get("thumbstick"))
            )
            thumb = ext.get("thumb") or [0.0, 0.0]
            _put_float64(frame.extensions, f"{hand}.stick.x", float(thumb[0]))
            _put_float64(frame.extensions, f"{hand}.stick.y", float(thumb[1]))
    return frame


def _put_bool(exts: Mapping[str, object], key: str, value: bool) -> None:
    v = exts[key]
    v.dtype = _DTYPE_BOOL
    v.data = bytes([1 if value else 0])


def _put_float64(exts: Mapping[str, object], key: str, value: float) -> None:
    v = exts[key]
    v.dtype = _DTYPE_FLOAT64
    v.data = struct.pack("<d", value)


def serialize(frame: object) -> bytes:
    return frame.SerializeToString()  # type: ignore[attr-defined]
