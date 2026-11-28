"""Simulated WebXR HUD client (port of tools/wsdriver).

Connects to the gateway's /ws and plays a scripted interaction so the whole
path can be exercised without a headset:

  t=1.0s  B/Y press+release  -> start (two-phase PAUSED)
  t=2.0s  B/Y press+release  -> resume (RUNNING)
  t=2.5-6.5s grip held       -> action frames with poses
  t=8.0-8.9s B/Y long press  -> finish

Usage: cloudrobo r2c teleop wsdriver --port 8000 --seconds 15
"""

from __future__ import annotations

import asyncio
import json
import math
import time

TIMELINE = [
    (1.0, 1.3, {"secondary": True}),
    (2.0, 2.3, {"secondary": True}),
    (2.5, 6.5, {"grip": True}),
    (8.0, 8.9, {"secondary": True}),
]


def _scripted(t: float, key: str) -> bool:
    return any(lo <= t < hi and state.get(key) for lo, hi, state in TIMELINE)


def _hand(t: float, grip: bool, secondary: bool) -> dict:
    ang = t * 0.8
    p = [0.02 * math.cos(ang), 0.02 * math.sin(ang), 0.0]
    q = [0.0, 0.0, math.sin(ang / 4), math.cos(ang / 4)]
    return {
        "p": p,
        "r": q,
        "grip": grip,
        "trigger": 0.5 if grip else 0.0,
        "primary": False,
        "secondary": secondary,
        "thumbstick": False,
        "thumb": [0.0, 0.0],
        "raw": {"p": p, "r": q},
    }


async def _run(port: int, seconds: float, rate: float) -> None:
    import websockets

    url = f"ws://127.0.0.1:{port}/ws"
    async with websockets.connect(url, max_size=65536) as ws:
        print(f"[wsdriver] connected {url}", flush=True)
        t0 = time.monotonic()
        sent = 0

        async def reader() -> None:
            try:
                async for raw in ws:
                    msg = json.loads(raw)
                    if msg.get("type") == "gateway_status":
                        print(
                            f"[wsdriver] status state={msg.get('state')} "
                            f"session={str(msg.get('session'))[:8]} "
                            f"L={msg.get('left',{}).get('state')} "
                            f"R={msg.get('right',{}).get('state')}",
                            flush=True,
                        )
            except Exception:  # noqa: BLE001
                return

        reader_task = asyncio.ensure_future(reader())
        try:
            while True:
                t = time.monotonic() - t0
                if t > seconds:
                    break
                secondary = _scripted(t, "secondary")
                grip = _scripted(t, "grip")
                await ws.send(
                    json.dumps(
                        {
                            "t": t,
                            "left": _hand(t, grip, secondary),
                            "right": _hand(t, grip, False),
                        }
                    )
                )
                sent += 1
                await asyncio.sleep(1.0 / max(1.0, rate))
        finally:
            reader_task.cancel()
        print(f"[wsdriver] done: {sent} frames sent", flush=True)


def run_wsdriver(*, port: int = 8000, seconds: float = 15.0, rate: float = 20.0) -> None:
    try:
        asyncio.run(_run(port, seconds, rate))
    except KeyboardInterrupt:
        pass
