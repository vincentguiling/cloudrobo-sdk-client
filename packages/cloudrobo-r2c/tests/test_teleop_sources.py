"""遥操输入源单元测试（mock / lerobot_leader）。

VR 输入源按 2026-11-05 决策移出实现（保留为设计阶段内容，见
docs/design/r2c-teleop-data-collection-design.md），对应测试一并移除。
"""

from __future__ import annotations

import pytest

from cloudrobo_r2c.teleop.base import create_source
from cloudrobo_r2c.teleop.sources import lerobot_leader, mock  # noqa: F401  触发注册
from cloudrobo_r2c.teleop.sources.lerobot_leader import LeRobotLeaderSource
from cloudrobo_r2c.teleop.sources.mock import MockJointsSource


# ── 源注册表 ─────────────────────────────────────────────────────────────


def test_create_source_registry_lookup():
    source = create_source({"type": "mock_joints", "config": {"joints": ["j1"]}})
    assert isinstance(source, MockJointsSource)


def test_create_source_unknown_type_rejected():
    with pytest.raises(ValueError, match="unknown teleop source type"):
        create_source({"type": "does_not_exist"})


def test_create_source_requires_type():
    with pytest.raises(ValueError, match="teleop.source.type is required"):
        create_source({})


# ── mock_joints ──────────────────────────────────────────────────────────


def test_mock_source_sine_targets():
    source = MockJointsSource(
        {"joints": ["joint_1", "joint_2"], "frequency_hz": 10.0, "gripper_amplitude": 0.0}
    )
    source.start()
    target = source.read()
    assert target is not None
    assert set(target.joint_positions) == {"joint_1", "joint_2"}
    for value in target.joint_positions.values():
        assert -0.301 <= value <= 0.301  # base 0 + amplitude 0.3（留相位误差）
    assert target.gripper is None  # gripper_amplitude=0 不输出夹爪
    source.stop()
    assert source.read() is None


def test_mock_source_gripper_oscillates():
    source = MockJointsSource({"joints": ["joint_1"], "gripper_amplitude": 0.5})
    source.start()
    target = source.read()
    assert target is not None
    assert target.gripper is not None
    assert 0.0 <= target.gripper <= 1.0
    source.stop()


# ── lerobot_leader ───────────────────────────────────────────────────────


class _FakeTeleoperator:
    """LeRobot teleoperator 测试替身（get_action 返回脚本值）。"""

    def __init__(self, script):
        self.script = list(script)
        self.connected = False
        self.disconnected = False

    def connect(self, **kwargs):
        self.connected = True
        self.connect_kwargs = kwargs

    def get_action(self):
        if len(self.script) > 1:
            return self.script.pop(0)
        return self.script[0]

    def disconnect(self):
        self.disconnected = True


def _leader_source(script, **extra_cfg):
    fake = _FakeTeleoperator(script)
    source = LeRobotLeaderSource(
        {"type": "so101_leader", "port": "COM9", **extra_cfg},
        teleoperator_factory=lambda cfg: fake,
    )
    return source, fake


def test_lerobot_leader_action_passthrough():
    source, fake = _leader_source(
        [{"shoulder_pan.pos": 1.5, "gripper.pos": 50.0}]
    )
    source.start()
    assert fake.connected
    target = source.read()
    assert target.joint_positions == {"shoulder_pan": 1.5, "gripper": 50.0}
    assert target.gripper is None  # 未配置 gripper_range 不输出归一化值
    source.stop()
    assert fake.disconnected


def test_lerobot_leader_gripper_normalization():
    source, _ = _leader_source(
        [{"gripper.pos": 25.0}], gripper_range=[0, 100]
    )
    source.start()
    target = source.read()
    assert target.gripper == pytest.approx(0.25)
    source.stop()


def test_lerobot_leader_connect_kwargs_passthrough():
    source, fake = _leader_source([{"a.pos": 0.0}], connect_kwargs={"calibrate": False})
    source.start()
    assert fake.connect_kwargs == {"calibrate": False}
    source.stop()


def test_lerobot_leader_read_failure_returns_none():
    class _Broken(_FakeTeleoperator):
        def get_action(self):
            raise RuntimeError("serial gone")

    fake = _Broken([])
    source = LeRobotLeaderSource(
        {"type": "so101_leader"}, teleoperator_factory=lambda cfg: fake
    )
    source.start()
    assert source.read() is None
    source.stop()


def test_lerobot_leader_requires_type():
    with pytest.raises(ValueError, match="type is required"):
        LeRobotLeaderSource({"port": "COM9"})
