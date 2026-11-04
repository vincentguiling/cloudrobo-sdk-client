"""VR 姿态流 mock 发送端：无 VR 设备时联调 vr_udp 遥操源。

按 docs/teleop.md 的 JSON 协议，以固定频率向目标 UDP 端口发送双手
正弦位姿 + 夹爪开度。配合 ``config/teleop_vr_udp.yaml`` 验证链路::

    python examples/vr_pose_mock_sender.py --host 127.0.0.1 --port 9870

配合 mock 机器人端（robot_dummy_config.yaml）可全链路 dry-run 遥操。
"""

from __future__ import annotations

import argparse
import json
import math
import socket
import time
from typing import Sequence


def _hand_frame(t: float, phase: float, radius: float, height: float) -> dict:
    return {
        "pos": [
            radius * math.cos(2 * math.pi * 0.2 * t + phase),
            radius * math.sin(2 * math.pi * 0.2 * t + phase),
            height + 0.05 * math.sin(2 * math.pi * 0.5 * t),
        ],
        "quat": [0.0, 0.0, 0.0, 1.0],
    }


def main(argv: Sequence[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1", help="Teleop operator UDP host")
    parser.add_argument("--port", type=int, default=9870, help="Teleop operator UDP port")
    parser.add_argument("--hz", type=float, default=30.0, help="Send rate in Hz")
    parser.add_argument("--radius", type=float, default=0.2, help="Circle radius (m)")
    parser.add_argument("--height", type=float, default=0.5, help="Base height (m)")
    args = parser.parse_args(argv)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    interval = 1.0 / max(args.hz, 1e-6)
    print(f"sending mock VR poses to udp/{args.host}:{args.port} at {args.hz} Hz (Ctrl+C to stop)")
    start = time.monotonic()
    try:
        while True:
            t = time.monotonic() - start
            frame = {
                "ts": int(time.time() * 1000),
                "left": _hand_frame(t, phase=0.0, radius=args.radius, height=args.height),
                "right": _hand_frame(t, phase=math.pi, radius=args.radius, height=args.height),
            }
            grip = 0.5 + 0.5 * math.sin(2 * math.pi * 0.25 * t)
            frame["right"]["grip"] = grip
            frame["left"]["grip"] = grip
            sock.sendto(json.dumps(frame).encode("utf-8"), (args.host, args.port))
            time.sleep(interval)
    except KeyboardInterrupt:
        print("stopped")


if __name__ == "__main__":
    main()
