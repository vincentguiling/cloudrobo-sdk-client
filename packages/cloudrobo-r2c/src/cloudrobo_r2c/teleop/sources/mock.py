"""mock_joints：无硬件遥操输入源（正弦波关节目标）。

用途：不接真实主臂/VR 时验证遥操链路（operator → 机器人 → 回显观测），
也是单元测试的输入源。生成的目标在 [base - amplitude, base + amplitude]
内以 ``frequency_hz`` 摆动，夹爪 0~1 往复。
"""

from __future__ import annotations

import math
import threading
import time
from typing import Any, Dict, List, Mapping, Optional

from ..base import TeleopSource, TeleopTarget, register_source


@register_source("mock_joints")
def create_mock_joints(config: Mapping[str, Any]) -> TeleopSource:
    return MockJointsSource(config)


class MockJointsSource(TeleopSource):
    """按配置的关节列表输出正弦波目标值。"""

    def __init__(self, config: Mapping[str, Any]) -> None:
        cfg = dict(config or {})
        joints: List[str] = [str(j) for j in cfg.get("joints") or ["joint_1", "joint_2"]]
        self._joints = joints
        self._amplitude = float(cfg.get("amplitude", 0.3))
        self._base = float(cfg.get("base", 0.0))
        self._frequency_hz = float(cfg.get("frequency_hz", 0.5))
        self._gripper_amplitude = float(cfg.get("gripper_amplitude", 0.5))
        self._start: Optional[float] = None
        self._lock = threading.Lock()

    @property
    def name(self) -> str:
        return "mock_joints"

    def start(self) -> None:
        with self._lock:
            self._start = time.monotonic()

    def read(self) -> Optional[TeleopTarget]:
        with self._lock:
            start = self._start
        if start is None:
            return None
        elapsed = time.monotonic() - start
        phase = 2.0 * math.pi * self._frequency_hz * elapsed
        target = TeleopTarget(source_timestamp_ms=int(time.time() * 1000))
        for index, joint in enumerate(self._joints):
            offset = index * 0.7  # 各关节错相，运动形态更接近真实遥操
            target.joint_positions[joint] = self._base + self._amplitude * math.sin(
                phase + offset
            )
        if self._gripper_amplitude > 0.0:
            target.gripper = 0.5 + 0.5 * self._gripper_amplitude * math.sin(phase)
        return target

    def stop(self) -> None:
        with self._lock:
            self._start = None

    def describe(self) -> Dict[str, Any]:
        return {
            "type": self.name,
            "joints": self._joints,
            "frequency_hz": self._frequency_hz,
        }
