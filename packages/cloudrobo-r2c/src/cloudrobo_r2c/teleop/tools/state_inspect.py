"""Diagnose {project}/{device}/state/current rate and decoded per-arm poses.

Port of tools/validation/inspect_state_current.py: prints sample rate, the
top-level fields present and the decoded end-effector poses, so "no pose on
the HUD" can be traced to data absence vs. parsing/layout problems.
"""

from __future__ import annotations

import json
import time


def run_state_inspect(
    *,
    endpoint: str = "tcp/127.0.0.1:7447",
    project_id: str = "r2c",
    device_id: str = "local",
    seconds: float = 10.0,
) -> None:
    import zenoh

    from ...common.models.generated import current_state_pb2

    topic = f"{project_id}/{device_id}/state/current"
    cfg = zenoh.Config()
    cfg.insert_json5("mode", '"client"')
    cfg.insert_json5("connect/endpoints", json.dumps([endpoint]))
    session = zenoh.open(cfg)
    state = {"count": 0, "last": None, "first_at": None}

    def on_sample(sample) -> None:
        try:
            msg = current_state_pb2.CurrentState.FromString(sample.payload.to_bytes())
        except Exception as exc:  # noqa: BLE001
            print(f"[state-inspect] decode error: {exc}", flush=True)
            return
        now = time.monotonic()
        if state["first_at"] is None:
            state["first_at"] = now
        state["count"] += 1
        ee = msg.end_effector_poses
        poses = {}
        for i, name in enumerate(ee.names):
            if i < len(ee.pose):
                poses[name] = [round(float(v), 4) for v in ee.pose[i].data]
        state["last"] = {
            "source": msg.source,
            "timestamp": msg.timestamp,
            "ee_names": list(ee.names),
            "ee_poses": poses,
            "joint_names": list(msg.joint_states.names),
            "has_ee": len(ee.names) > 0,
        }

    sub = session.declare_subscriber(topic, on_sample)
    print(f"[state-inspect] subscribed {topic} ({seconds:g}s)", flush=True)
    try:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            time.sleep(2.0)
            elapsed = time.monotonic() - (state["first_at"] or time.monotonic())
            rate = state["count"] / elapsed if elapsed > 0 else 0.0
            print(
                f"[state-inspect] samples={state['count']} rate={rate:.2f}Hz "
                f"last={state['last']}",
                flush=True,
            )
    except KeyboardInterrupt:
        pass
    finally:
        sub.undeclare()
        session.close()
