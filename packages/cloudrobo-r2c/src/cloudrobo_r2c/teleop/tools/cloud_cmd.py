"""Publish a remote teleop lifecycle command (cloud console debug tool).

Port of tools/validation/cloud_cmd_pub.py: publishes start/pause/resume/
finish/quit on {project}/{device}/teleop/actions with RELIABLE QoS, optionally
carrying per-hand transform matrices in the start payload.

Usage: cloudrobo r2c teleop cloud-cmd start --session my-session \
         --pos-left "1,0,0, 0,1,0, 0,0,1"
"""

from __future__ import annotations

import json
import time


def publish_cloud_command(
    *,
    kind: str,
    session_id: str = "",
    endpoints: str | None = None,
    project_id: str = "r2c",
    device_id: str = "local",
    operator_id: str = "cloud-console",
    pos_left: list[float] | None = None,
    rot_left: list[float] | None = None,
    pos_right: list[float] | None = None,
    rot_right: list[float] | None = None,
) -> None:
    import zenoh

    from ..common.models.generated import teleop_pb2

    eps = (
        [e.strip() for e in endpoints.split(",") if e.strip()]
        if endpoints
        else ["tcp/127.0.0.1:7447"]
    )
    topic = f"{project_id}/{device_id}/teleop/actions"
    cfg = zenoh.Config()
    cfg.insert_json5("mode", '"client"')
    cfg.insert_json5("connect/endpoints", json.dumps(eps))
    session = zenoh.open(cfg)
    pub = session.declare_publisher(
        topic,
        reliability=zenoh.Reliability.RELIABLE,
        priority=zenoh.Priority.INTERACTIVE_HIGH,
    )
    frame = teleop_pb2.TeleopFrame()
    frame.timestamp = int(time.time() * 1000)
    frame.operator_id = operator_id
    frame.session_id = session_id
    frame.input_mode = teleop_pb2.InputMode.INPUT_MODE_VR
    sub = getattr(frame, kind)
    if kind == "start":
        for name, values in (
            ("pos_matrix_left", pos_left),
            ("rot_matrix_left", rot_left),
            ("pos_matrix_right", pos_right),
            ("rot_matrix_right", rot_right),
        ):
            if values is not None:
                if len(values) != 9:
                    raise ValueError(f"{name} needs 9 values, got {len(values)}")
                getattr(sub, name).values.extend(float(v) for v in values)
    sub.SetInParent()
    pub.put(frame.SerializeToString())
    print(
        f"[cloud-cmd] published {kind} session={session_id or '-'} -> {topic}",
        flush=True,
    )
    time.sleep(0.2)
    pub.undeclare()
    session.close()