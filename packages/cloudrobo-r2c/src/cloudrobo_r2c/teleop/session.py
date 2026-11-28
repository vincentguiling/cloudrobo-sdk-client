"""Operator session state machine (port of impl/webxr-gateway/Program.cs).

The operator keeps its own display/interaction state (the robot side only
knows IDLE/RUNNING/PAUSED):

  local mode:
    IDLE/FINISHED --B/Y release--> PAUSED --next B/Y release--> RUNNING
    RUNNING <-> PAUSED  (B/Y short press: press edges the decision, release settles)
    RUNNING/PAUSED --B/Y long press >=500ms--> FINISHED (terminal until new start/quit)
    FINISHED --B/Y release--> new start

  bundle mode (--bundle): start/finish/quit come from the cloud; local B/Y
  keeps pause/resume only. Cloud start enters PAUSED first (two-phase, same
  as local) and the operator's B/Y release confirms into RUNNING.

Two-phase start: entering PAUSED does not publish poses; state/current
samples keep flowing so the RUNNING base is the latest sample at confirmation.
"""

from __future__ import annotations

import time
import uuid
from dataclasses import dataclass, field
from typing import Optional

IDLE, PAUSED, RUNNING, FINISHED = "Idle", "Paused", "Running", "Finished"

LONG_PRESS_MS = 500.0


@dataclass
class SessionEvent:
    kind: str          # start | pause | resume | finish | quit
    session_id: str
    input_mode: int = 1  # INPUT_MODE_VR
    start_matrices: Optional[dict[str, list[float]]] = None
    seq: int = 0       # >0 for remote commands


class OperatorSession:
    def __init__(self, *, cloud_mode: bool = False, auto_start: bool = False,
                 input_mode: int = 1) -> None:
        self.cloud_mode = cloud_mode
        self.auto_start = auto_start
        self.input_mode = input_mode
        self.state = IDLE
        self.session_id = ""
        self.running_entered_ms = 0.0
        self.finished_once = False
        # B/Y edge tracking (local mode)
        self.prev_secondary = False
        self.has_pending_sec = False
        self.pending_sec_resume = False
        self.secondary_pressed_at = 0.0
        self.require_all_released = False
        # cloud command bookkeeping
        self.last_cloud_seq = 0
        self.cloud_adjust = {"seq": 0, "left": None, "right": None}

    # ── queries ──────────────────────────────────────────────────────────────
    @property
    def is_running(self) -> bool:
        return self.state == RUNNING

    @property
    def is_active(self) -> bool:
        return self.state in (RUNNING, PAUSED)

    def teleop_mode(self) -> str:
        return "CONTROLLING" if self.is_active else "AUTO"

    # ── local input events ───────────────────────────────────────────────────
    def on_buttons(self, primary: bool, secondary: bool, now_ms: float) -> list[SessionEvent]:
        """Feed per-frame button states; returns command frames to publish.

        ``secondary`` is the B/Y button; A/X (``primary``) never drives session
        control (its raw value rides along in extensions).
        """
        events: list[SessionEvent] = []
        if self.cloud_mode:
            return self._on_buttons_cloud_mode(primary, secondary, now_ms)

        secondary_edge = secondary and not self.prev_secondary
        secondary_released = (not secondary) and self.prev_secondary
        any_button = primary or secondary

        # finish: long press while running/paused settles immediately
        if (
            self.has_pending_sec
            and secondary
            and self.state in (RUNNING, PAUSED)
            and now_ms - self.secondary_pressed_at >= LONG_PRESS_MS
        ):
            self.state = FINISHED
            self.finished_once = True
            self.session_id = ""
            self.require_all_released = True
            self.has_pending_sec = False
            events.append(SessionEvent("finish", ""))

        # start: B/Y release edge in IDLE/FINISHED
        if (
            self.state in (IDLE, FINISHED)
            and secondary_released
            and not self.require_all_released
        ):
            self.session_id = uuid.uuid4().hex
            self.state = PAUSED
            self.running_entered_ms = now_ms
            events.append(
                SessionEvent("start", self.session_id, input_mode=self.input_mode)
            )

        # pause/resume: press edges the decision, release settles
        if secondary_edge:
            self.secondary_pressed_at = now_ms
            if self.state in (RUNNING, PAUSED):
                self.has_pending_sec = True
                self.pending_sec_resume = self.state == PAUSED
        if secondary_released and self.has_pending_sec and self.state in (RUNNING, PAUSED):
            self.has_pending_sec = False
            if self.pending_sec_resume:
                self.state = RUNNING
                self.running_entered_ms = now_ms
                events.append(SessionEvent("resume", self.session_id, input_mode=self.input_mode))
            else:
                self.state = PAUSED
                events.append(SessionEvent("pause", self.session_id, input_mode=self.input_mode))

        self.prev_secondary = secondary
        if self.require_all_released and not any_button:
            self.require_all_released = False
            self.prev_secondary = False
        return events

    def _on_buttons_cloud_mode(
        self, primary: bool, secondary: bool, now_ms: float
    ) -> list[SessionEvent]:
        """Cloud mode: B/Y short press toggles pause/resume only."""
        events: list[SessionEvent] = []
        secondary_edge = secondary and not self.prev_secondary
        secondary_released = (not secondary) and self.prev_secondary
        any_button = primary or secondary

        if secondary_edge and self.state in (RUNNING, PAUSED):
            self.has_pending_sec = True
            self.pending_sec_resume = self.state == PAUSED
            self.secondary_pressed_at = now_ms
        if (
            secondary and self.has_pending_sec and self.state in (RUNNING, PAUSED)
            and now_ms - self.secondary_pressed_at >= LONG_PRESS_MS
        ):
            # long press in cloud mode is not finish; treat as held (no-op)
            pass
        if secondary_released and self.has_pending_sec and self.state in (RUNNING, PAUSED):
            self.has_pending_sec = False
            if self.pending_sec_resume:
                self.state = RUNNING
                self.running_entered_ms = now_ms
                events.append(SessionEvent("resume", self.session_id, input_mode=self.input_mode))
            else:
                self.state = PAUSED
                events.append(SessionEvent("pause", self.session_id, input_mode=self.input_mode))

        self.prev_secondary = secondary
        if self.require_all_released and not any_button:
            self.require_all_released = False
            self.prev_secondary = False
        return events

    # ── cloud command application ────────────────────────────────────────────
    def on_cloud_command(self, kind: str, session_id: str, seq: int, now_ms: float,
                         start_matrices: Optional[dict[str, list[float]]] = None) -> bool:
        """Apply a remote command; returns whether it was accepted.

        Same rules as the C# gateway (§3.3.1.8): cloud start requires a
        session_id (else rejected), enters PAUSED first; pause/resume/finish
        with a mismatched session are stale (ignored); quit always resets.
        """
        if seq <= self.last_cloud_seq:
            return False
        self.last_cloud_seq = seq
        applied = True
        if kind == "start":
            if not session_id:
                applied = False
            else:
                self.state = PAUSED
                self.session_id = session_id
                self.running_entered_ms = now_ms
                self.finished_once = False
                self.require_all_released = False
                self.has_pending_sec = False
                if start_matrices and self._apply_matrices(start_matrices):
                    print(
                        f"[session] cloud transform matrices applied "
                        f"(seq={self.cloud_adjust['seq']})",
                        flush=True,
                    )
        elif kind == "pause":
            if session_id and session_id != self.session_id:
                applied = False
            elif self.state == RUNNING:
                self.state = PAUSED
            else:
                applied = False
            self.has_pending_sec = False
        elif kind == "resume":
            if session_id and session_id != self.session_id:
                applied = False
            elif self.state == PAUSED:
                self.state = RUNNING
                self.running_entered_ms = now_ms
            else:
                applied = False
            self.has_pending_sec = False
        elif kind == "finish":
            if self.state in (RUNNING, PAUSED):
                self.state = FINISHED
                self.finished_once = True
                self.session_id = ""
                self.require_all_released = True
            else:
                applied = False
            self.has_pending_sec = False
        elif kind == "quit":
            self.state = IDLE
            self.session_id = ""
            self.require_all_released = False
            self.has_pending_sec = False
        else:
            applied = False
        return applied

    def _apply_matrices(self, matrices: dict[str, list[float]]) -> bool:
        changed = False
        left = self.cloud_adjust["left"] or {}
        right = self.cloud_adjust["right"] or {}
        if matrices.get("pos_left"):
            left = dict(left, posR=matrices["pos_left"])
            changed = True
        if matrices.get("rot_left"):
            left = dict(left, rotR=matrices["rot_left"])
            changed = True
        if matrices.get("pos_right"):
            right = dict(right, posR=matrices["pos_right"])
            changed = True
        if matrices.get("rot_right"):
            right = dict(right, rotR=matrices["rot_right"])
            changed = True
        if changed:
            self.cloud_adjust = {
                "seq": self.cloud_adjust["seq"] + 1,
                "left": left or None,
                "right": right or None,
            }
        return changed

    # ── helpers ──────────────────────────────────────────────────────────────
    def auto_start_now(self, now_ms: float) -> Optional[SessionEvent]:
        """Headset-free sources (e.g. mock): start the session immediately.

        Only applies in local mode while IDLE/FINISHED. The operator is
        synthetic, so there is no B/Y confirmation: the session goes straight
        to RUNNING (the robot side only needs the start command frame).
        """
        if self.cloud_mode or self.state not in (IDLE, FINISHED):
            return None
        self.session_id = uuid.uuid4().hex
        self.state = RUNNING
        self.running_entered_ms = now_ms
        self.finished_once = False
        self.require_all_released = False
        self.has_pending_sec = False
        self.prev_secondary = False
        print(
            f"[session] auto-start (headset-free input) session={self.session_id}",
            flush=True,
        )
        return SessionEvent("start", self.session_id, input_mode=self.input_mode)

    def status_payload(self) -> dict:
        return {
            "state": self.state,
            "session": self.session_id,
            "cloud": self.cloud_mode,
            "cloud_adjust": None if self.cloud_adjust["seq"] == 0 else self.cloud_adjust,
        }