"""Relative pose mapping (port of core/RelativePoseMapping.cs).

Relative control (design §3.3.1.4):
  * base = latest state/current end-effector pose when the session starts;
  * while grip is held, the controller motion relative to the grip press is
    applied on top of that base: target = align(robot, ctrl@press) * ctrl(t);
  * clutch re-press re-anchors on the LAST COMMANDED OUTPUT (the arm stays
    where it is; only new motion is applied) — never jump to a stale
    feedback pose;
  * reach limit clamps around the last output; EMA smooths the output;
  * without grip the previous output is held (no new command for that hand).
"""

from __future__ import annotations

from typing import Optional

from ..pose_math import Pose, clamp_reach, compose, ema, inverse


class RelativePoseMapping:
    name = "relative_pose"

    def __init__(self, reach_limit_m: float = 0.5, smoothing_alpha: float = 1.0) -> None:
        self.reach_limit_m = reach_limit_m
        self.smoothing_alpha = smoothing_alpha
        self.has_base = False
        self.base = Pose()
        self._align: Optional[Pose] = None
        self._was_grip = False
        self._filtered = Pose()
        self._has_filtered = False
        self._last_output = Pose()
        self.following = False

    # ── lifecycle ────────────────────────────────────────────────────────────
    def begin(self, robot_pose: Pose, ctrl_pose: Pose) -> None:
        """(Re)arm at session entry / resume: anchor on the robot feedback pose."""
        self.base = robot_pose.copy()
        self.has_base = True
        self._align = compose(robot_pose, inverse(ctrl_pose))
        self._was_grip = False
        self._has_filtered = False
        self._last_output = robot_pose.copy()
        self.following = False

    def disengage(self) -> None:
        """Pause/finish: hold the last output; the next begin() re-anchors."""
        self._was_grip = False
        self.following = False

    def map(self, grip: bool, ctrl: Pose) -> Pose:
        if self._align is None:
            self.following = False
            return self._last_output
        if grip and not self._was_grip:
            # clutch: re-anchor on the last commanded output (continuity)
            self._align = compose(self._last_output, inverse(ctrl))
        self._was_grip = grip
        self.following = grip
        if not grip:
            return self._last_output
        raw = compose(self._align, ctrl)
        anchor = self._last_output.p if self.has_base else raw.p
        raw.p = clamp_reach(anchor, raw.p, self.reach_limit_m)
        self._filtered = (
            ema(self._filtered, raw, self.smoothing_alpha) if self._has_filtered else raw
        )
        self._has_filtered = True
        self._last_output = self._filtered
        return self._filtered

    # ── introspection ────────────────────────────────────────────────────────
    @property
    def last_output(self) -> Pose:
        return self._last_output
