"""遥操输入源单元测试（mock / vr_udp / lerobot_leader）。"""

from __future__ import annotations

import json
import socket
import time

import pytest
from cloudrobo_r2c.teleop.base import create_source
from cloudrobo_r2c.teleop.sources import lerobot_leader, mock, vr_udp  # noqa: F401  触发注册
from cloudrobo_r2c.teleop.sources.lerobot_leader import LeRobotLeaderSource
from cloudrobo_r2c.teleop.sources.mock import MockJointsSource
from cloudrobo_r2c.teleop.sources.vr_udp import VrUdpSource

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


# ── vr_udp ───────────────────────────────────────────────────────────────


def _free_udp_port() -> int:
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    probe.bind(("127.0.0.1", 0))
    port = probe.getsockname()[1]
    probe.close()
    return port


def _send_datagram(port: int, frame: dict) -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.sendto(json.dumps(frame).encode("utf-8"), ("127.0.0.1", port))
    finally:
        sock.close()


def _wait_for_frame(source: VrUdpSource, timeout_s: float = 2.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        target = source.read()
        if target is not None:
            return target
        time.sleep(0.02)
    raise AssertionError("vr_udp source did not receive any datagram in time")


def test_vr_udp_pose_mapping_and_gripper():
    config = {
        "listen_host": "127.0.0.1",
        "listen_port": 0,
        "stale_after_s": 0.5,
        "scale": 2.0,
        "workspace_offset": [0.1, 0.0, 0.0],
        "hands": ["left", "right"],
        "ee_name_map": {"left": "left_ee", "right": "right_ee"},
        "gripper": {"source": "pinch", "hand": "right"},
    }
    source = VrUdpSource(config)
    source.start()
    try:
        assert source.bound_port is not None and source.bound_port != 0
        _send_datagram(
            source.bound_port,
            {
                "ts": 1,
                "left": {"pos": [0.2, 0.3, 0.4], "quat": [0.0, 0.0, 0.0, 1.0]},
                "right": {"pos": [0.0, 0.0, 0.0], "pinch": 0.25},
            },
        )
        target = _wait_for_frame(source)
        assert target.stale is False
        left = target.end_effector_poses["left_ee"]
        assert len(left) == 7
        assert left[0] == pytest.approx(0.2 * 2.0 + 0.1)
        assert left[1] == pytest.approx(0.6)
        assert left[2] == pytest.approx(0.8)
        # identity 四元数 → (0,0,0,1)
        assert left[3:] == pytest.approx([0.0, 0.0, 0.0, 1.0])
        # pinch 0.25（1=捏合）→ 开度 0.75
        assert target.gripper == pytest.approx(0.75)
    finally:
        source.stop()


def test_vr_udp_axis_remap_and_quat_order():
    config = {
        "listen_host": "127.0.0.1",
        "listen_port": 0,
        "axis_remap": {"x": "+y", "y": "-x", "z": "+z"},
        "quat_order": "wxyz",
        "hands": ["right"],
        "ee_name_map": {"right": "arm_ee"},
    }
    source = VrUdpSource(config)
    source.start()
    try:
        # 源坐标 [1, 0, 0]，四元数 wxyz=(1,0,0,0) 即 xyzw=(0,0,0,1) 恒等
        _send_datagram(
            source.bound_port,
            {"right": {"pos": [1.0, 0.0, 0.0], "quat": [1.0, 0.0, 0.0, 0.0]}},
        )
        target = _wait_for_frame(source)
        pose = target.end_effector_poses["arm_ee"]
        # robot_x = +source_y = 0；robot_y = -source_x = -1；robot_z = source_z = 0
        assert pose[0] == pytest.approx(0.0)
        assert pose[1] == pytest.approx(-1.0)
        assert pose[2] == pytest.approx(0.0)
        # 恒等四元数经轴重映射旋转后仍为单位四元数
        norm = sum(v * v for v in pose[3:]) ** 0.5
        assert norm == pytest.approx(1.0, abs=1e-6)
    finally:
        source.stop()


def test_vr_udp_stale_and_empty():
    config = {
        "listen_host": "127.0.0.1",
        "listen_port": 0,
        "stale_after_s": 0.15,
        "hands": ["right"],
    }
    source = VrUdpSource(config)
    source.start()
    try:
        assert source.read() is None  # 未收到任何数据
        _send_datagram(source.bound_port, {"right": {"pos": [0.0, 0.0, 0.0]}})
        target = _wait_for_frame(source)
        assert target.stale is False
        time.sleep(0.3)  # 超过 stale_after_s
        target = source.read()
        assert target is not None and target.stale is True
    finally:
        source.stop()


def test_vr_udp_invalid_config_rejected():
    with pytest.raises(ValueError, match="quat_order"):
        VrUdpSource({"quat_order": "bogus"})
    with pytest.raises(ValueError, match="workspace_offset"):
        VrUdpSource({"workspace_offset": [1.0, 2.0]})
    with pytest.raises(ValueError, match="gripper.source"):
        VrUdpSource({"gripper": {"source": "lever"}})


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
