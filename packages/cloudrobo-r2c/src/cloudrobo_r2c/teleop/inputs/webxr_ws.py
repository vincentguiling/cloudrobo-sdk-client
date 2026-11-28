"""WebXR HUD input source (WS JSON frames -> InputSample).

The gateway serves ``webxr/index.html`` statically and routes ``/ws`` here.
Frame contract (unchanged from the C# gateway, so the HUD page needs no
modification):

  HUD -> gateway, ~20 Hz:
    {"t": seconds,
     "left":  {"p": [x,y,z], "r": [qx,qy,qz,qw], "grip": bool,
               "trigger": 0..1, "primary": bool, "secondary": bool,
               "thumbstick": bool, "thumb": [x,y],
               "raw": {"p": [...], "r": [...]}},
     "right": {...}}

  gateway -> HUD (gateway_status, on state changes / 50-200 ms):
    {"type": "gateway_status", "state": "...", "session": "...",
     "cloud": bool, "cloud_adjust": {...}|null,
     "left":  {"state": "nodata|offline|online|following", "reason": "...",
               "pose": [7] | null},
     "right": {...}}
"""

from __future__ import annotations

import json
from typing import Any, Awaitable, Callable, Optional

from . import INPUT_MODE_VR, InputSample


class WebXrWsSource:
    name = "webxr"
    input_mode = INPUT_MODE_VR
    ws_path = "/ws"
    wants_auto_session = False

    def __init__(self, params: Optional[dict] = None,
                 state_getter: Optional[Callable[[str], Optional[Any]]] = None) -> None:
        self.params = params or {}
        self.state_getter = state_getter
        self._ws: Any = None
        self._sink: Optional[Callable[[list[InputSample]], Awaitable[None]]] = None

    def capabilities(self) -> dict:
        return {
            "bimanual": True,
            "trigger": True,
            "deadman": "grip",
            "buttons": ["primary", "secondary", "thumbstick"],
            "rate_hz": 20,
        }

    async def start(self, sink: Callable[[list[InputSample]], Awaitable[None]]) -> None:
        # The HUD drives the samples through handle_websocket(); keep the sink
        # for frames that arrive while the socket is open.
        self._sink = sink

    async def stop(self) -> None:
        self._sink = None

    # ── WebSocket serving ────────────────────────────────────────────────────
    async def handle_websocket(self, ws: Any, status_getter: Callable[[], dict]) -> None:
        """Serve one HUD connection until it closes."""
        self._ws = ws
        print("[gateway] headset connected", flush=True)
        try:
            await self._send_status(ws, status_getter())
            async for msg in ws:
                if msg.type.name != "TEXT":
                    continue
                samples = self._parse_frame(msg.data)
                if samples:
                    await self._emit(samples)
        except Exception:  # noqa: BLE001 - disconnect is normal
            pass
        finally:
            if self._ws is ws:
                self._ws = None
            print("[gateway] headset disconnected", flush=True)

    async def push_status(self, status: dict) -> None:
        ws = self._ws
        if ws is None:
            return
        try:
            await self._send_status(ws, status)
        except Exception:  # noqa: BLE001
            pass

    # ── internals ────────────────────────────────────────────────────────────
    async def _emit(self, samples: list[InputSample]) -> None:
        if self._sink is not None:
            await self._sink(samples)

    async def _send_status(self, ws: Any, status: dict) -> None:
        await ws.send_str(json.dumps(status))

    def _parse_frame(self, data: str) -> list[InputSample]:
        try:
            frame = json.loads(data)
        except (TypeError, ValueError):
            return []
        if not isinstance(frame, dict) or "left" not in frame and "right" not in frame:
            return []  # ping / keepalive
        samples: list[InputSample] = []
        for side in ("left", "right"):
            hand = frame.get(side)
            if not isinstance(hand, dict):
                continue
            pose = hand.get("p") if hand.get("p") is not None else None
            quat = hand.get("r") or [0.0, 0.0, 0.0, 1.0]
            pose7 = [float(pose[0]), float(pose[1]), float(pose[2]),
                     float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])] if pose else None
            thumb = hand.get("thumb") or [0.0, 0.0]
            samples.append(
                InputSample(
                    input_mode=self.input_mode,
                    side=side,
                    pose=pose7,
                    grip=bool(hand.get("grip")),
                    trigger=float(hand.get("trigger") or 0.0),
                    buttons={
                        "primary": bool(hand.get("primary")),
                        "secondary": bool(hand.get("secondary")),
                        "thumbstick": bool(hand.get("thumbstick")),
                    },
                    axes={"x": float(thumb[0]), "y": float(thumb[1])},
                    raw=hand,
                )
            )
        return samples
