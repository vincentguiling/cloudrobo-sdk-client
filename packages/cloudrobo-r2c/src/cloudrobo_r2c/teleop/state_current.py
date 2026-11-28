"""state/current subscription: the relative-control base (port of
core/StateCurrentReceiver.cs + StateCurrentConfig.cs).

The robot publishes CurrentState (protos/current_state.proto) at ~10 Hz with
the orientation-aware field layout (orientation=3, location=4,
joint_states=5, end_effector_states=6, end_effector_poses=7). This module
extracts per-arm end-effector poses from either:

  * end_effector_poses: RuntimeEndEffectorPoses names[i] <-> pose[i],
    "quat" = [tx,ty,tz,qx,qy,qz,qw] or "euler" = [tx,ty,tz,rx,ry,rz] rad;
  * joint_states: the same pose values mirrored into the joint stream
    (agibot appends per-arm pose blocks after the arm joints), with
    "trailing" (joints first, then per-arm blocks in marker order) or
    "inline" layout.

Absent data must never become a fake base: callers get None.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from typing import Any, Mapping, Optional

from .config import StateCurrentConfig
from .pose_math import Pose, euler_to_quaternion


@dataclass
class PoseSnapshot:
    poses: dict[str, Pose] = field(default_factory=dict)
    last_sample_monotonic: float = 0.0
    last_sample_wall_ms: float = 0.0

    @property
    def has_data(self) -> bool:
        return self.last_sample_monotonic > 0.0


class StateCurrentReceiver:
    """Subscribe state/current and keep the latest per-arm pose snapshot."""

    def __init__(self, transport: Any, topic: str, cfg: StateCurrentConfig) -> None:
        self.transport = transport
        self.topic = topic
        self.cfg = cfg
        self._lock = threading.Lock()
        self._snapshot = PoseSnapshot()
        self._sub: Any = None

    def start(self) -> None:
        self._sub = self.transport.subscribe(self.topic, self._on_sample)

    def stop(self) -> None:
        if self._sub is not None:
            try:
                self._sub.undeclare()
            except Exception:  # noqa: BLE001
                pass
            self._sub = None

    def snapshot(self) -> PoseSnapshot:
        with self._lock:
            return self._snapshot

    def latest_pose(self, name: str, now_monotonic: Optional[float] = None,
                    grey_timeout_ms: float = 600.0) -> Optional[Pose]:
        with self._lock:
            snap = self._snapshot
            if not snap.has_data:
                return None
            if grey_timeout_ms > 0 and now_monotonic is not None:
                age_ms = (now_monotonic - snap.last_sample_monotonic) * 1000.0
                if age_ms > grey_timeout_ms:
                    return None
            return snap.poses.get(name)

    def _on_sample(self, sample: Any) -> None:
        try:
            from ..common.models.generated import current_state_pb2
        except Exception as exc:  # pragma: no cover - generated at build time
            print(
                "[state-current] protobuf generated modules missing; run the "
                f"package build (compile_protobuf) first: {exc}",
                flush=True,
            )
            return
        try:
            msg = current_state_pb2.CurrentState.FromString(sample.payload.to_bytes())
            poses = self._extract_poses_pb(msg)
        except Exception as exc:  # noqa: BLE001 - layout mismatch is explicit
            print(
                "[state-current] state/current layout mismatch (incompatible robot build): "
                f"{exc}. The robot side must publish CurrentState with the "
                "orientation field numbering (orientation=3, location=4, "
                "joint_states=5, end_effector_states=6, end_effector_poses=7).",
                flush=True,
            )
            return
        now_m = time.monotonic()
        with self._lock:
            self._snapshot = PoseSnapshot(
                poses=poses,
                last_sample_monotonic=now_m,
                last_sample_wall_ms=time.time() * 1000.0,
            )

    # ── extraction ───────────────────────────────────────────────────────────
    def _extract_poses_pb(self, msg: Any) -> dict[str, Pose]:
        poses: dict[str, Pose] = {}
        if self.cfg.source == "joint_states":
            for name in self.cfg.effector_names:
                pose = self._find_pose_in_joint_states(msg.joint_states, name)
                if pose is not None:
                    poses[name] = pose
            return poses
        for name in self.cfg.effector_names:
            pose = self._find_pose_in_end_effectors(msg.end_effector_poses, name)
            if pose is not None:
                poses[name] = pose
        return poses

    def _find_pose_in_end_effectors(self, ee: Any, match: str) -> Optional[Pose]:
        for i, name in enumerate(ee.names):
            if i >= len(ee.pose):
                break
            if self._name_matches(name, match):
                data = list(ee.pose[i].data)
                return self._pose_from_values(data)
        return None

    def _find_pose_in_joint_states(self, js: Any, match: str) -> Optional[Pose]:
        names = list(js.names)
        position = list(js.position.data) if js.position is not None else []
        markers = [i for i, n in enumerate(names) if self._is_marker(n)]
        if not markers:
            return None
        need = 6 if self._use_euler() else 7
        if self.cfg.joint_pose_layout != "inline":
            # trailing: per-arm pose blocks in marker order, after joint values
            joint_count = len(names) - len(markers)
            for k, marker_idx in enumerate(markers):
                if not self._name_matches(names[marker_idx], match):
                    continue
                start = joint_count + k * self.cfg.values_per_arm
                if start + need > len(position):
                    return None
                return self._pose_from_values(position[start:start + need])
            return None
        # inline: marker's names index i -> position[i+1 : i+1+per)
        for i, name in enumerate(names):
            if self._is_marker(name) and self._name_matches(name, match):
                start = i + 1
                if start + need > len(position):
                    return None
                return self._pose_from_values(position[start:start + need])
        return None

    def _pose_from_values(self, data: list[float]) -> Optional[Pose]:
        if self._use_euler():
            if len(data) < 6:
                return None
            return Pose(
                [data[0], data[1], data[2]],
                euler_to_quaternion(data[3], data[4], data[5], self.cfg.euler_order),
            )
        if len(data) < 7:
            return None
        return Pose([data[0], data[1], data[2]], [data[3], data[4], data[5], data[6]])

    # ── helpers ──────────────────────────────────────────────────────────────
    def _use_euler(self) -> bool:
        return self.cfg.format.lower() == "euler"

    def _is_marker(self, name: str) -> bool:
        low = (name or "").lower()
        return any(low.endswith(s.lower()) for s in self.cfg.joint_name_suffixes)

    def _name_matches(self, name: str, match: str) -> bool:
        n = (name or "").strip().lower()
        m = (match or "").strip().lower()
        if not n or not m:
            return False
        return n == m or n.endswith("_" + m) or n.endswith("/" + m) or n.endswith("." + m)
