"""Teleop operator gateway: HTTP/WS server + session + publishers.

Runtime pieces (all wired here, each unit-testable on its own):

  * aiohttp serves the WebXR HUD page and the input source's websocket
    endpoint on one port (the HUD page needs no change: same ws URL + JSON);
  * the input source (webxr / mock / third-party) feeds InputSamples;
  * OperatorSession applies the local B/Y or cloud command state machine;
  * RelativePoseMapping (or passthrough for the mock device) turns samples
    into per-hand target poses;
  * actions are published at teleop.actions_rate_hz on a dedicated timer
    (BEST_EFFORT), commands on RELIABLE, heartbeats on a background thread;
  * state/current provides the live base; a missing/stale base suppresses the
    affected hand (never a fake base);
  * gateway_status is pushed to the HUD with the same JSON shape as before.
"""

from __future__ import annotations

import asyncio
import time
from pathlib import Path
from typing import Any, Optional

from .commands import TeleopCommandReceiver
from .config import TeleopConfig
from .frame_builder import build_action_frame, build_command_frame, serialize
from .heartbeat import HeartbeatPublisher
from .inputs import InputSample, make_input_source
from .mapping import create_mapping
from .pose_math import Pose
from .session import FINISHED, IDLE, RUNNING, OperatorSession, SessionEvent
from .state_current import StateCurrentReceiver
from .transport import ZenohTransport

WEBXr_DIR = Path(__file__).resolve().parent.parent / "webxr"


class TeleopGateway:
    def __init__(
        self,
        cfg: TeleopConfig,
        *,
        transport: Optional[ZenohTransport] = None,
        cloud_mode: bool = False,
    ) -> None:
        self.cfg = cfg
        self.transport = transport or ZenohTransport(cfg)
        self.cloud_mode = cloud_mode
        self.frame_dir = WEBXr_DIR
        self._state = StateCurrentReceiver(
            self.transport, cfg.state_current_topic, cfg.state_current
        )
        # Publishers/heartbeat are declared in start(), after transport.open().
        self._action_pub: Any = None
        self._command_pub: Any = None
        self._heartbeat_pub: Any = None
        self.heartbeat: Optional[HeartbeatPublisher] = None
        self.session = OperatorSession(
            cloud_mode=cloud_mode,
            auto_start=cfg.auto_start,
            input_mode=1,
        )
        mapping_name = cfg.mapping or ("passthrough" if cfg.input_source == "mock" else "relative_pose")
        self.mapping_name = mapping_name
        self.source = make_input_source(
            cfg.input_source, cfg.input_params, state_getter=self._latest_base
        )
        self._session_mapper = create_mapping(
            mapping_name,
            reach_limit_m=cfg.reach_limit_m,
            smoothing_alpha=cfg.smoothing_alpha,
        )
        # per-hand mapping state (each hand owns an independent base/clutch)
        self._hand_mapping: dict[str, Any] = {}
        self._following: dict[str, bool] = {"left": False, "right": False}
        self._sent_pose: dict[str, Optional[Pose]] = {"left": None, "right": None}
        self._latest: dict[str, InputSample] = {}
        self._latest_buttons = {"primary": False, "secondary": False}
        self._last_status = 0.0
        self._last_status_state = ""
        self._publish_task: Optional[asyncio.Task] = None
        self._running = False
        self._warn_last: dict[str, float] = {}
        self._applied_cloud_seq = 0
        self._cloud_cmd: Optional[TeleopCommandReceiver] = None
        if cloud_mode:
            self._cloud_cmd = TeleopCommandReceiver(
                self.transport, cfg.actions_topic, cfg.operator_id
            )

    # ── lifecycle ────────────────────────────────────────────────────────────
    async def start(self) -> None:
        self.transport.open()
        self._action_pub = self.transport.declare_action_publisher(self.cfg.actions_topic)
        self._command_pub = self.transport.declare_command_publisher(self.cfg.actions_topic)
        self._heartbeat_pub = self.transport.declare_heartbeat_publisher(
            self.cfg.heartbeats_topic
        )
        self.heartbeat = HeartbeatPublisher(
            self._heartbeat_pub,
            f"{self.cfg.project_id}/{self.cfg.device_id}",
            self.session.teleop_mode,
        )
        self._state.start()
        if self._cloud_cmd is not None:
            self._cloud_cmd.start()
        await self.source.start(self._sink)
        self.heartbeat.start()
        self._publish_task = asyncio.ensure_future(self._publish_loop())
        print(
            f"[gateway] publishing {self.cfg.actions_topic} "
            f"@{self.cfg.actions_rate_hz:g} Hz, heartbeats on "
            f"{self.cfg.heartbeats_topic}; subscribing {self.cfg.state_current_topic}",
            flush=True,
        )
        print(
            f"[gateway] input={self.source.name} mapping={self.mapping_name} "
            f"cloud_mode={self.cloud_mode} operator={self.cfg.operator_id}",
            flush=True,
        )

    async def stop(self) -> None:
        if self._publish_task is not None:
            self._publish_task.cancel()
            try:
                await self._publish_task
            except (asyncio.CancelledError, Exception):  # noqa: BLE001
                pass
            self._publish_task = None
        if self.heartbeat is not None:
            self.heartbeat.stop()
        try:
            await self.source.stop()
        except Exception:  # noqa: BLE001
            pass
        if self._cloud_cmd is not None:
            self._cloud_cmd.stop()
        self._state.stop()
        self.transport.close()
        print("[gateway] stopped", flush=True)

    # ── input sink (called by the input source) ─────────────────────────────
    async def _sink(self, samples: list[InputSample]) -> None:
        now_ms = time.time() * 1000.0
        for sample in samples:
            if sample.side:
                self._latest[sample.side] = sample
            if sample.buttons:
                for name in ("primary", "secondary"):
                    if sample.buttons.get(name):
                        self._latest_buttons[name] = True
        primary = any(s.buttons.get("primary") for s in samples)
        secondary = any(s.buttons.get("secondary") for s in samples)

        # headset-free auto-start (mock / scripted sources)
        if (
            getattr(self.source, "wants_auto_session", False)
            and not self.cloud_mode
            and self.session.state in (IDLE, FINISHED)
        ):
            event = self.session.auto_start_now(now_ms)
            if event is not None:
                self._publish_event(event)

        # local B/Y edges
        for event in self.session.on_buttons(primary, secondary, now_ms):
            self._publish_event(event)

        # cloud commands (bundle mode)
        if self._cloud_cmd is not None:
            seq = self._cloud_cmd.latest_seq
            if seq != self._applied_cloud_seq:
                cmd = self._cloud_cmd.latest()
                if cmd is not None:
                    applied = self.session.on_cloud_command(
                        cmd.kind,
                        cmd.session_id,
                        cmd.seq,
                        now_ms,
                        start_matrices={
                            "pos_left": cmd.start_pos_left,
                            "rot_left": cmd.start_rot_left,
                            "pos_right": cmd.start_pos_right,
                            "rot_right": cmd.start_rot_right,
                        }
                        if cmd.has_start_matrices
                        else None,
                    )
                    self._applied_cloud_seq = seq
                    if applied:
                        print(
                            f"[gateway] cloud {cmd.kind.upper()} "
                            f"(session={cmd.session_id}) -> {self.session.state}",
                            flush=True,
                        )
                        self._sent_pose = {"left": None, "right": None}

        self._maybe_reset_mapping()
        await self._maybe_push_status(now_ms)

    def _maybe_reset_mapping(self) -> None:
        """Clear per-hand mapping/clutch on session edges (pause/finish/start)."""
        if self.session.state != RUNNING:
            for hand in ("left", "right"):
                self._sent_pose[hand] = None
                self._following[hand] = False

    # ── publishing ──────────────────────────────────────────────────────────
    async def _publish_loop(self) -> None:
        interval = 1.0 / max(1.0, self.cfg.actions_rate_hz)
        next_at = time.monotonic()
        while True:
            next_at += interval
            delay = next_at - time.monotonic()
            if delay > 0:
                await asyncio.sleep(delay)
            else:
                next_at = time.monotonic()
            try:
                self._publish_actions()
            except Exception as exc:  # noqa: BLE001 - keep pacing
                print(f"[gateway] action publish error: {exc}", flush=True)

    def _publish_actions(self) -> None:
        if self.session.state != RUNNING:
            return
        now_m = time.monotonic()
        snap = self._state.snapshot()
        fresh = (
            snap.has_data
            and (now_m - snap.last_sample_monotonic) * 1000.0
            <= self.cfg.grey_timeout_ms
        )
        hands: dict[str, tuple[Pose, float]] = {}
        for hand in ("left", "right"):
            sample = self._latest.get(hand)
            if sample is None or not sample.grip:
                self._sent_pose[hand] = None
                self._following[hand] = False
                continue
            base = self._state.latest_pose(hand, now_m, self.cfg.grey_timeout_ms)
            if base is None or not fresh:
                self._warn(
                    f"{hand.upper()} action suppressed: "
                    + (
                        "no fresh state/current pose for that arm"
                        if base is None
                        else "waiting for a state/current sample received after entering RUNNING"
                    )
                )
                self._sent_pose[hand] = None
                self._following[hand] = False
                continue
            ctrl = Pose(
                list(sample.pose[0:3]) if sample.pose else [0, 0, 0],
                list(sample.pose[3:7]) if sample.pose else [0, 0, 0, 1],
            )
            mapper = self._hand_mapping.get(hand)
            if mapper is None:
                mapper = create_mapping(
                    self.mapping_name,
                    reach_limit_m=self.cfg.reach_limit_m,
                    smoothing_alpha=self.cfg.smoothing_alpha,
                )
                self._hand_mapping[hand] = mapper
            if not self._following[hand]:
                mapper.begin(base, ctrl)
            pose = mapper.map(sample.grip, ctrl)
            self._sent_pose[hand] = pose
            self._following[hand] = True
            hands[hand] = (pose, sample.trigger)
        frame = build_action_frame(
            self.session.session_id,
            self.cfg.operator_id,
            hands,
            raw_inputs={
                side: {
                    "primary": self._latest[side].buttons.get("primary", False),
                    "thumbstick": self._latest[side].buttons.get("thumbstick", False),
                    "thumb": [
                        self._latest[side].axes.get("x", 0.0),
                        self._latest[side].axes.get("y", 0.0),
                    ],
                }
                for side in ("left", "right")
                if side in self._latest
            },
        )
        self.transport.put(self._action_pub, serialize(frame))

    def _publish_event(self, event: SessionEvent) -> None:
        frame = build_command_frame(
            event.kind,
            session_id=event.session_id,
            operator_id=self.cfg.operator_id,
            input_mode=event.input_mode,
            start=event.start_matrices,
        )
        self.transport.put(self._command_pub, serialize(frame))
        print(
            f"[gateway] -> {event.kind} session={event.session_id} -> {self.session.state}",
            flush=True,
        )

    # ── status ──────────────────────────────────────────────────────────────
    async def _maybe_push_status(self, now_ms: float) -> None:
        state = self.session.state
        heartbeat_ms = 50.0 if state == RUNNING else 200.0
        if state == self._last_status_state and now_ms - self._last_status < heartbeat_ms:
            return
        self._last_status = now_ms
        self._last_status_state = state
        status = self._status_json()
        push = getattr(self.source, "push_status", None)
        if push is not None:
            await push(status)

    def _status_json(self) -> dict:
        snap = self._state.snapshot()
        running = self.session.state == RUNNING
        now_m = time.monotonic()

        def hand_json(hand: str, sample: Optional[InputSample]) -> dict:
            fresh_pose = self._state.latest_pose(hand, now_m, self.cfg.grey_timeout_ms)
            if not snap.has_data:
                hs, reason = "nodata", "no_data"
            elif fresh_pose is None:
                hs, reason = "offline", "no_pose"
            elif sample is None:
                hs, reason = "offline", "controller"
            elif running and sample.grip and self._sent_pose.get(hand):
                hs, reason = "following", "ok"
            else:
                hs, reason = "online", "ok"
            pose = None
            if running and self._sent_pose.get(hand) is not None:
                pose = _pose7(self._sent_pose[hand])
            elif fresh_pose is not None:
                pose = _pose7(fresh_pose)
            out = {"state": hs, "reason": reason, "pose": pose}
            out["anchor"] = None
            return out

        status = {
            "type": "gateway_status",
            "state": self.session.state,
            "session": self.session.session_id,
            "cloud": self.cloud_mode,
            "cloud_adjust": self.session.status_payload()["cloud_adjust"],
            "left": hand_json("left", self._latest.get("left")),
            "right": hand_json("right", self._latest.get("right")),
        }
        return status

    def _warn(self, msg: str, throttle_s: float = 2.0) -> None:
        now = time.monotonic()
        if now - self._warn_last.get(msg, 0.0) >= throttle_s:
            self._warn_last[msg] = now
            print(f"[gateway] ERROR: {msg}", flush=True)

    # ── helpers ──────────────────────────────────────────────────────────────
    def _latest_base(self, hand: str) -> Optional[Pose]:
        return self._state.latest_pose(hand, time.monotonic(), self.cfg.grey_timeout_ms)

    def _webxr_handler(self) -> Optional[Any]:
        if getattr(self.source, "ws_path", None):
            return self._handle_ws
        return None

    async def _handle_ws(self, request: Any) -> Any:  # pragma: no cover - needs aiohttp
        from aiohttp import web

        ws = web.WebSocketResponse(max_msg_size=64 * 1024)
        await ws.prepare(request)
        handler = getattr(self.source, "handle_websocket", None)
        if handler is not None:
            await handler(ws, self._status_json)
        else:
            await ws.close()
        return ws


def _pose7(pose: Optional[Pose]) -> Optional[list[float]]:
    if pose is None:
        return None
    return [
        pose.p[0],
        pose.p[1],
        pose.p[2],
        pose.q[0],
        pose.q[1],
        pose.q[2],
        pose.q[3],
    ]


async def run_gateway(cfg: TeleopConfig, *, cloud_mode: bool = False) -> None:
    """Serve HTTP/WS until interrupted."""
    from aiohttp import web

    gateway = TeleopGateway(cfg, cloud_mode=cloud_mode)
    await gateway.start()

    app = web.Application()

    async def index(request: Any) -> Any:
        return web.FileResponse(WEBXr_DIR / "index.html")

    app.router.add_get("/", index)
    handler = gateway._webxr_handler()
    if handler is not None:
        app.router.add_get(getattr(gateway.source, "ws_path", "/ws"), handler)
    app.router.add_static("/", WEBXr_DIR)

    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, cfg.http_host, cfg.http_port)
    await site.start()
    print(
        f"[gateway] http://localhost:{cfg.http_port}  (static page + "
        f"{getattr(gateway.source, 'ws_path', 'no')})",
        flush=True,
    )
    try:
        await asyncio.Event().wait()
    finally:
        await gateway.stop()
        await runner.cleanup()
