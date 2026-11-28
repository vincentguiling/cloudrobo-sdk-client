"""Remote (cloud) teleop command receiver (port of core/TeleopCommandReceiver.cs).

In ``--bundle`` mode the session lifecycle is driven by the cloud console:
start/pause/resume/finish/quit command frames arrive on the same
``teleop/actions`` topic. The gateway subscribes there and must ignore its
own locally published frames — the cloud console is a different zenoh
session, and the gateway stamps its frames with its own ``operator_id``.

Each accepted remote command bumps a sequence number; the gateway applies
only new sequences (latest-command-wins, same as the C# receiver).
"""

from __future__ import annotations

import threading
from dataclasses import dataclass, field
from typing import Any, Optional


@dataclass
class CloudCommand:
    kind: str                      # start | pause | resume | finish | quit
    session_id: str = ""
    seq: int = 0
    start_pos_left: Optional[list[float]] = None
    start_rot_left: Optional[list[float]] = None
    start_pos_right: Optional[list[float]] = None
    start_rot_right: Optional[list[float]] = None

    @property
    def has_start_matrices(self) -> bool:
        return any(
            m is not None
            for m in (
                self.start_pos_left,
                self.start_rot_left,
                self.start_pos_right,
                self.start_rot_right,
            )
        )


class TeleopCommandReceiver:
    """Subscribe teleop/actions; keep the latest *remote* lifecycle command."""

    def __init__(self, transport: Any, topic: str, own_operator_id: str) -> None:
        self.transport = transport
        self.topic = topic
        self.own_operator_id = own_operator_id
        self._lock = threading.Lock()
        self._latest: Optional[CloudCommand] = None
        self._seq = 0
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

    @property
    def latest_seq(self) -> int:
        with self._lock:
            return self._seq

    def latest(self) -> Optional[CloudCommand]:
        with self._lock:
            return self._latest

    def _on_sample(self, sample: Any) -> None:
        try:
            from ..common.models.generated import teleop_pb2

            frame = teleop_pb2.TeleopFrame.FromString(sample.payload.to_bytes())
        except Exception:  # noqa: BLE001 - ignore undecodable frames
            return
        which = frame.WhichOneof("command")
        if not which:
            return
        if self.own_operator_id and frame.operator_id == self.own_operator_id:
            return  # our own local command frames never loop back as cloud input
        cmd = CloudCommand(kind=which, session_id=frame.session_id)
        if which == "start":
            start = frame.start
            cmd.start_pos_left = _matrix(start.pos_matrix_left)
            cmd.start_rot_left = _matrix(start.rot_matrix_left)
            cmd.start_pos_right = _matrix(start.pos_matrix_right)
            cmd.start_rot_right = _matrix(start.rot_matrix_right)
        with self._lock:
            self._seq += 1
            cmd.seq = self._seq
            self._latest = cmd


def _matrix(msg: Any) -> Optional[list[float]]:
    values = list(getattr(msg, "values", []) or [])
    if len(values) != 9:
        return None
    out = []
    for v in values:
        if v != v or v in (float("inf"), float("-inf")):  # NaN/Inf
            return None
        out.append(float(v))
    return out