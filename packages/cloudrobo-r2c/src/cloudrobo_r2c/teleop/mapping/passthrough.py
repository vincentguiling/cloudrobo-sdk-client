"""Passthrough mapping: the input sample's pose is already the target pose.

Used by devices whose reading IS a desired end-effector pose rather than a
controller offset — e.g. the mock device (state/current + small offset) or a
future "scripted trajectory" source. No base, clutch, reach limiting or
filtering is applied (the source is responsible for producing a sane target).
"""

from __future__ import annotations

from ..pose_math import Pose


class PassthroughMapping:
    name = "passthrough"

    def __init__(self, reach_limit_m: float = 0.5, smoothing_alpha: float = 1.0) -> None:
        self.reach_limit_m = reach_limit_m
        self.smoothing_alpha = smoothing_alpha
        self.following = False

    def begin(self, robot_pose: Pose, ctrl_pose: Pose) -> None:
        return

    def disengage(self) -> None:
        self.following = False

    def map(self, grip: bool, ctrl: Pose) -> Pose:
        self.following = grip
        return ctrl if grip else ctrl
