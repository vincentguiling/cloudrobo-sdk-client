"""Subscribe teleop/actions and report rate / one-way latency / frame size.

Port of the C# tools/bench: prints frame rate, latency avg/p50/p95/max,
average frame bytes and command-frame transitions.

    cloudrobo r2c teleop bench --seconds 20 --label mytest
"""

from __future__ import annotations

import json
import time


def _endpoints(endpoints: str | None) -> list[str]:
    if endpoints:
        eps = [e.strip() for e in endpoints.split(",") if e.strip()]
        if eps:
            return eps
    return ["tcp/127.0.0.1:7447"]


def run_bench(
    *,
    seconds: float = 20.0,
    label: str = "",
    endpoints: str | None = None,
    project_id: str = "r2c",
    device_id: str = "local",
) -> None:
    import zenoh

    from ...common.models.generated import teleop_pb2

    topic = f"{project_id}/{device_id}/teleop/actions"
    cfg = zenoh.Config()
    cfg.insert_json5("mode", '"client"')
    cfg.insert_json5("connect/endpoints", json.dumps(_endpoints(endpoints)))
    session = zenoh.open(cfg)
    stats = {
        "frames": 0,
        "bytes": 0,
        "latencies": [],
        "commands": {},
        "actions": 0,
        "pose_frames": 0,
        "first_ts": None,
        "last_ts": None,
    }

    def on_sample(sample) -> None:
        try:
            frame = teleop_pb2.TeleopFrame.FromString(sample.payload.to_bytes())
        except Exception:  # noqa: BLE001
            return
        now_ms = time.time() * 1000.0
        stats["frames"] += 1
        stats["bytes"] += len(sample.payload)
        if frame.timestamp:
            stats["latencies"].append(now_ms - frame.timestamp)
            if stats["first_ts"] is None:
                stats["first_ts"] = frame.timestamp
            stats["last_ts"] = frame.timestamp
        which = frame.WhichOneof("command")
        if which:
            stats["commands"][which] = stats["commands"].get(which, 0) + 1
            print(
                f"[bench] command={which} session={frame.session_id} "
                f"operator={frame.operator_id}",
                flush=True,
            )
        else:
            stats["actions"] += 1
            if frame.end_effector_poses.pose:
                stats["pose_frames"] += 1

    sub = session.declare_subscriber(topic, on_sample)
    print(
        f"[bench] subscribed {topic} (label={label or '-'}, {seconds:g}s)", flush=True
    )
    try:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            time.sleep(0.5)
            _report(stats, elapsed=seconds - (deadline - time.monotonic()), partial=True)
    except KeyboardInterrupt:
        pass
    finally:
        sub.undeclare()
        session.close()
    _report(stats, elapsed=seconds)


def _report(stats: dict, *, elapsed: float, partial: bool = False) -> None:
    lat = sorted(stats["latencies"][-5000:])
    n = len(lat)
    if not n:
        return
    p50 = lat[n // 2]
    p95 = lat[min(n - 1, int(n * 0.95))]
    avg = sum(lat) / n
    rate = stats["frames"] / max(elapsed, 1e-6)
    tag = "partial" if partial else "BENCH_RESULT"
    print(
        f"[bench] {tag} frames={stats['frames']} rate={rate:.2f}Hz "
        f"latency avg/p50/p95/max={avg:.1f}/{p50:.1f}/{p95:.1f}/{lat[-1]:.1f}ms "
        f"avg_size={(stats['bytes']/stats['frames'] if stats['frames'] else 0):.0f}B "
        f"actions={stats['actions']} pose_frames={stats['pose_frames']} "
        f"commands={stats['commands']}",
        flush=True,
    )
