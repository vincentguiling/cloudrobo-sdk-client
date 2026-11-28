"""Mock teleop device: offsets the robot's current pose in a small circle.

Per the 2026-11-28 request: "mock 会按照 state/current 对应的这个末端位姿
上下左右稍微偏移返回给机器人" — the device reads the robot's latest
state/current end-effector pose (per arm), adds a small up/down/left/right
offset, and hands the resulting *target pose* to the gateway. With the
``passthrough`` mapping the pose is published as-is, so the robot visibly
follows the small wobble — a headset-free demo of the full teleop path
(session -> publish -> robot gating).

Params (teleop.input.params):
  side: "both" | "left" | "right"   (default both)
  amplitude_m: offset radius in metres (default 0.02)
  rate_hz: wobble frequency (default 0.3)
  interval_s: sampling interval (default 0.05)
"""

from __future__ import annotations

import asyncio
import math
import time
from typing import Any, Awaitable, Callable, Optional

from . import INPUT_MODE_VR, InputSample


class MockSource:
    name = "mock"
    input_mode = INPUT_MODE_VR
    ws_path = None
    wants_auto_session = True

    def __init__(self, params: Optional[dict] = None,
                 state_getter: Optional[Callable[[str], Optional[Any]]] = None) -> None:
        self.params = params or {}
        self.state_getter = state_getter
        self.side = str(self.params.get("side") or "both")
        self.amplitude_m = float(self.params.get("amplitude_m") or 0.02)
        self.rate_hz = float(self.params.get("rate_hz") or 0.3)
        self.interval_s = float(self.params.get("interval_s") or 0.05)
        self._task: Optional[asyncio.Task] = None
        self._stop: Optional[asyncio.Event] = None
        self._t0 = time.monotonic()

    def capabilities(self) -> dict:
        return {
            "bimanual": self.side == "both",
            "trigger": True,
            "deadman": "grip",
            "rate_hz": 1.0 / self.interval_s,
            "synthetic": True,
        }

    async def start(self, sink: Callable[[list[InputSample]], Awaitable[None]]) -> None:
        self._t0 = time.monotonic()
        self._stop = asyncio.Event()
        self._task = asyncio.ensure_future(self._loop(sink))

    async def stop(self) -> None:
        if self._stop is not None:
            self._stop.set()
        if self._task is not None:
            self._task.cancel()
            try:
                await self._task
            except (asyncio.CancelledError, Exception):  # noqa: BLE001
                pass
            self._task = None

    async def _loop(self, sink: Callable[[list[InputSample]], Awaitable[None]]) -> None:
        while self._stop is not None and not self._stop.is_set():
            samples = self._build_samples()
            if samples:
                await sink(samples)
            await asyncio.sleep(self.interval_s)

    def _build_samples(self) -> list[InputSample]:
        if self.state_getter is None:
            return []
        t = time.monotonic() - self._t0
        phase = 2.0 * math.pi * self.rate_hz * t
        # up/down/left/right: x = sin phase, y = cos phase (small circle)
        dx = self.amplitude_m * math.sin(phase)
        dy = self.amplitude_m * math.cos(phase)
        sides = ("left", "right") if self.side == "both" else (self.side,)
        samples: list[InputSample] = []
        for side in sides:
            base = self.state_getter(side)
            if base is None:
                continue
            pose = [
                base.p[0] + dx,
                base.p[1] + dy,
                base.p[2],
                base.q[0],
                base.q[1],
                base.q[2],
                base.q[3],
            ]
            samples.append(
                InputSample(
                    input_mode=self.input_mode,
                    side=side,
                    pose=pose,
                    grip=True,
                    trigger=0.5,
                    buttons={"primary": False, "secondary": False, "thumbstick": False},
                    axes={"x": 0.0, "y": 0.0},
                    raw={"mock": True, "phase": phase},
                )
            )
        return samples