"""Unit tests for the AgiBot G01 hardware adapter (fake ``a2d_sdk``).

The fake ``a2d_sdk`` modules are installed *before* the adapter is imported so
the client module's import-time guard binds the fakes — no GDK environment or
robot required.
"""

from __future__ import annotations

import struct
import sys
import types
from pathlib import Path
from typing import Any, Dict, List

import numpy as np
import pytest
import yaml

# ---------------------------------------------------------------------------
# Fake a2d_sdk（必须在导入被测模块之前安装）
# ---------------------------------------------------------------------------


class FakeRobotDds:
    """记录所有 GDK 调用；返回确定性的关节状态。"""

    instances: List["FakeRobotDds"] = []

    def __init__(self) -> None:
        type(self).instances.append(self)
        self.calls: List[tuple] = []
        self.arm = [float(i + 1) / 10 for i in range(14)]
        self.gripper = [0.25, 0.75]
        self.hand = [0.0] * 12
        self.head = [0.1, -0.2]
        self.waist = [0.05, 30.0]
        self.body_pose = [0.0, 0.1, 0.2, 0.3]
        self.fail_next_state_query = False
        self.shutdown_count = 0

    # 状态查询（《指南》7.1：返回 (数据, 时间戳)）
    def arm_joint_states(self):
        if self.fail_next_state_query:
            self.fail_next_state_query = False
            raise RuntimeError("dds hiccup")
        return list(self.arm), 1_000_000_000

    def gripper_states(self):
        return list(self.gripper), 1_000_000_000

    def hand_joint_states(self):
        return list(self.hand), 1_000_000_000

    def head_joint_states(self):
        return list(self.head), 1_000_000_000

    def waist_joint_states(self):
        return list(self.waist), 1_000_000_000

    def body_pose_joint_states(self):
        return list(self.body_pose)

    # 运动控制（《指南》7.2）
    def move_arm(self, positions):
        self.calls.append(("move_arm", list(positions)))

    def move_gripper(self, positions):
        self.calls.append(("move_gripper", list(positions)))

    def move_hand(self, positions):
        self.calls.append(("move_hand", list(positions)))

    def move_head(self, positions):
        self.calls.append(("move_head", list(positions)))

    def move_waist(self, positions):
        self.calls.append(("move_waist", list(positions)))

    def move_wheel(self, linear, angular):
        self.calls.append(("move_wheel", float(linear), float(angular)))

    def reset(self, **targets):
        self.calls.append(("reset", dict(targets)))

    def shutdown(self):
        self.shutdown_count += 1
        self.calls.append(("shutdown",))


class FakeCosineCamera:
    instances: List["FakeCosineCamera"] = []

    def __init__(self, camera_group):
        self.camera_group = list(camera_group)
        self.frames = {name: np.full((4, 4, 3), 7, dtype=np.uint8) for name in self.camera_group}
        self.closed = False
        type(self).instances.append(self)

    def get_latest_image(self, camera_name):
        return self.frames.get(camera_name)

    def close(self):
        self.closed = True


def _install_fake_a2d_sdk() -> None:
    """真实 a2d_sdk 未安装时注入假模块（被测模块导入前调用）。"""
    try:
        import a2d_sdk  # noqa: F401
    except ImportError:
        pass
    else:
        return  # 真实 SDK 已安装，不注入假模块
    a2d = types.ModuleType("a2d_sdk")
    robot_mod = types.ModuleType("a2d_sdk.robot")
    robot_mod.RobotDds = FakeRobotDds
    robot_mod.CosineCamera = FakeCosineCamera
    a2d.robot = robot_mod
    sys.modules.setdefault("a2d_sdk", a2d)
    sys.modules.setdefault("a2d_sdk.robot", robot_mod)


_install_fake_a2d_sdk()

from cloudrobo_r2c.robots.agibot_g01 import agibot_g01_client  # noqa: E402
from cloudrobo_r2c.robots.agibot_g01.agibot_g01_hardware_adapter import (  # noqa: E402
    ARM_DOF,
    GRIPPER_DOF,
    AgiBotG01HardwareAdapter,
    create_agibot_g01_adapter,
)

CONFIG_PATH = (
    Path(agibot_g01_client.__file__).resolve().parents[2]
    / "config"
    / "robot_agibot_g01_config.yaml"
)


@pytest.fixture(autouse=True)
def _reset_fakes():
    FakeRobotDds.instances.clear()
    FakeCosineCamera.instances.clear()
    yield
    FakeRobotDds.instances.clear()
    FakeCosineCamera.instances.clear()


def _connect(config: Dict[str, Any] = None) -> AgiBotG01HardwareAdapter:
    adapter = create_agibot_g01_adapter(config or {})
    adapter.connect()
    return adapter


def _last_robot() -> FakeRobotDds:
    assert FakeRobotDds.instances, "FakeRobotDds was never constructed"
    return FakeRobotDds.instances[-1]


def _register_commands(adapter: AgiBotG01HardwareAdapter) -> None:
    """与工厂路径一致：按 ``hardware.config.commands`` 实例化已注册的命令类。"""
    from cloudrobo_r2c.robots.robot_factory import RobotFactory

    RobotFactory._register_adapter_commands(adapter, adapter._config)


# ---------------------------------------------------------------------------
# 生命周期
# ---------------------------------------------------------------------------


def test_factory_creates_adapter_with_defaults():
    adapter = create_agibot_g01_adapter({})
    assert isinstance(adapter, AgiBotG01HardwareAdapter)
    assert not adapter.is_connected
    # 默认启用相机并映射到 R2C 键
    assert adapter._camera_enabled is True
    assert adapter._camera_key_map["head"] == "cam_high"


def test_connect_disconnect_lifecycle():
    adapter = _connect()
    assert adapter.is_connected
    assert _last_robot() is not None
    camera = FakeCosineCamera.instances[-1]
    assert camera.camera_group == ["head", "hand_left", "hand_right"]
    adapter.disconnect()
    assert not adapter.is_connected
    assert _last_robot().shutdown_count == 1
    assert camera.closed


def test_connect_failure_cleans_up():
    # 相机构造失败时，机器人连接应被回滚
    from cloudrobo_r2c.robots.agibot_g01 import agibot_g01_client as client_mod

    original = client_mod._CosineCamera

    def _boom(group):
        raise RuntimeError("camera unavailable")

    client_mod._CosineCamera = _boom
    try:
        adapter = create_agibot_g01_adapter({})
        with pytest.raises(RuntimeError, match="camera unavailable"):
            adapter.connect()
        assert not adapter.is_connected
        assert _last_robot().shutdown_count == 1
    finally:
        client_mod._CosineCamera = original


def test_connect_raises_without_sdk(monkeypatch):
    from cloudrobo_r2c.robots.agibot_g01 import agibot_g01_client as client_mod

    monkeypatch.setattr(client_mod, "_RobotDds", None)
    adapter = create_agibot_g01_adapter({})
    with pytest.raises(RuntimeError, match="a2d_sdk"):
        adapter.connect()
    assert not adapter.is_connected


# ---------------------------------------------------------------------------
# 观测
# ---------------------------------------------------------------------------


def test_get_observation_assembles_flat_fields():
    adapter = _connect()
    obs = adapter.get_observation()

    assert obs["id"] == 1
    assert isinstance(obs["timestamp"], int)
    assert list(obs["joint_positions"]) == pytest.approx(_last_robot().arm)
    assert list(obs["gripper_positions"]) == pytest.approx([0.25, 0.75])
    assert len(obs["hand_joint_positions"]) == 12
    assert list(obs["head_joint_positions"]) == pytest.approx([0.1, -0.2])
    assert list(obs["waist_joint_positions"]) == pytest.approx([0.05, 30.0])
    assert list(obs["body_pose"]) == pytest.approx([0.0, 0.1, 0.2, 0.3])
    # 相机键按 camera_key_map 暴露
    assert obs["cam_high"].shape == (4, 4, 3)
    assert obs["cam_left_wrist"].shape == (4, 4, 3)
    assert obs["cam_right_wrist"].shape == (4, 4, 3)

    # 第二帧 id 自增
    assert adapter.get_observation()["id"] == 2


def test_get_observation_falls_back_on_state_failure():
    adapter = _connect()
    first = adapter.get_observation()
    robot = _last_robot()
    robot.fail_next_state_query = True
    second = adapter.get_observation()
    # 状态查询失败：沿用上一帧值，不抛异常
    assert list(second["joint_positions"]) == pytest.approx(first["joint_positions"])
    assert list(second["joint_positions"]) == pytest.approx(robot.arm)


def test_get_observation_skips_cameras_when_disabled():
    adapter = _connect({"camera_enabled": False})
    obs = adapter.get_observation()
    assert "cam_high" not in obs
    assert FakeCosineCamera.instances == []


def test_get_observation_maps_custom_camera_keys():
    adapter = _connect(
        {
            "camera_group": ["head", "head_depth"],
            "camera_key_map": {"head": "cam_high", "head_depth": "cam_high_depth"},
        }
    )
    obs = adapter.get_observation()
    assert "cam_high" in obs
    assert "cam_high_depth" in obs


# ---------------------------------------------------------------------------
# 动作
# ---------------------------------------------------------------------------


def test_send_action_dispatches_all_keys():
    adapter = _connect()
    adapter.send_action(
        {
            "arm_cmd_joint_pos": [0.0] * ARM_DOF,
            "gripper_cmd_pos": [0.5, 0.5],
            "hand_cmd_joint_pos": [0.0] * 12,
            "head_cmd_joint_pos": [0.1, 0.0],
            "waist_cmd_joint_pos": [0.0, 25.0],
            "wheel_cmd_speed": [0.2, 0.0],
        }
    )
    calls = {c[0]: c for c in _last_robot().calls}
    assert calls["move_arm"][1] == [0.0] * ARM_DOF
    assert calls["move_gripper"][1] == [0.5, 0.5]
    assert calls["move_head"][1] == [0.1, 0.0]
    assert calls["move_waist"][1] == [0.0, 25.0]
    assert calls["move_wheel"][1:] == (0.2, 0.0)


def test_send_action_skips_wrong_dims():
    adapter = _connect()
    adapter.send_action({"arm_cmd_joint_pos": [0.0] * 6})
    # 维度不匹配：记录错误并跳过，不应调用 GDK
    assert not any(c[0] == "move_arm" for c in _last_robot().calls)


def test_send_action_dry_run_skips_hardware():
    adapter = _connect({"dry_run": True})
    adapter.connect()
    adapter.send_action({"arm_cmd_joint_pos": [0.0] * ARM_DOF})
    robot = _last_robot()
    assert not any(c[0].startswith("move_") for c in robot.calls)
    adapter.disconnect()
    assert robot.shutdown_count == 1


def test_send_action_decodes_extension_value():
    adapter = _connect()
    payload = struct.pack("<2f", 0.1, 0.2)
    extension = {
        "dtype": "FLOAT32",
        "shape": [2],
        "data": payload,
        "mime_type": "",
    }
    adapter.send_action({"head_cmd_joint_pos": extension})
    assert _last_robot().calls[-1] == ("move_head", pytest.approx([0.1, 0.2]))


def test_send_action_ignores_unknown_command():
    adapter = _connect()
    adapter.send_action({"unknown_field": [1.0]})
    assert not any(c[0].startswith("move_") for c in _last_robot().calls)


def test_go_home_command_calls_reset():
    adapter = _connect({"commands": {"go_home": {"type": "go_home", "head_positions": [0.0, 0.0]}}})
    adapter.connect()
    _register_commands(adapter)
    assert adapter.execute_command("go_home") is True
    reset_calls = [c for c in _last_robot().calls if c[0] == "reset"]
    assert reset_calls == [("reset", {"head_positions": [0.0, 0.0]})]


def test_go_home_command_rejects_wrong_dims():
    adapter = _connect({"commands": {"go_home": {"type": "go_home", "arm_positions": [0.0, 1.0]}}})
    adapter.connect()
    _register_commands(adapter)
    with pytest.raises(ValueError, match="arm_positions"):
        adapter.execute_command("go_home")


def test_actions_require_connection():
    adapter = create_agibot_g01_adapter({})
    with pytest.raises(RuntimeError, match="not connected"):
        adapter.get_observation()
    with pytest.raises(RuntimeError, match="not connected"):
        adapter.send_action({"arm_cmd_joint_pos": [0.0] * ARM_DOF})


# ---------------------------------------------------------------------------
# 配置 YAML + 翻译链路（ConfigurableDeviceTranslator）
# ---------------------------------------------------------------------------


def _load_robot_config() -> Dict[str, Any]:
    assert CONFIG_PATH.exists(), f"missing shipped config: {CONFIG_PATH}"
    return yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))


def test_robot_config_yaml_passes_schema_validation():
    from cloudrobo_r2c.common.robot_config_schema import RobotConfig

    config = _load_robot_config()
    # hardware.type 校验依赖 entry point 注册（agibot_g01）
    RobotConfig.model_validate(config)


def test_robot_config_yaml_translates_observation_and_action():
    from cloudrobo_r2c.common.models import Observations
    from cloudrobo_r2c.translators.device_translator import (
        ConfigurableDeviceTranslator,
    )

    config = _load_robot_config()
    translator = ConfigurableDeviceTranslator.from_config(config)

    adapter = _connect()
    obs = adapter.get_observation()
    observation = translator.device_to_r2c(obs)
    assert isinstance(observation, Observations)
    assert len(observation.joint_states.position) == ARM_DOF
    assert observation.joint_states.names[0] == "left_arm_joint_1"
    assert len(observation.end_effector_states.position) == GRIPPER_DOF
    # 相机映射为 JPEG 字节
    jpeg = observation.images.color.get("cam_high")
    assert jpeg is not None and jpeg[:2] == b"\xff\xd8"
    # extensions：head/waist/hand 均以 FLOAT32 打包（body_pose 变长默认不映射）
    head_ext = observation.extensions["head_joint"]
    assert list(np.frombuffer(head_ext.data, dtype="<f4")) == pytest.approx([0.1, -0.2])
    assert observation.extensions["hand_joint"].shape == [12]

    # r2c_to_device：云端动作 → 适配器平铺字段
    from cloudrobo_r2c.common.models import ExtensionValue

    step = {
        "timestamp": 1,
        "id": 1,
        "chunk_index": 0,
        "joint_states": {
            "names": [],
            "position": [0.1] * ARM_DOF,
            "velocity": [],
            "torque": [],
        },
        "end_effector_states": {
            "names": [],
            "position": [0.4, 0.6],
            "velocity": [],
            "torque": [],
        },
        "end_effector_poses": {},
        "localization": {"odom_pose": [], "map_pose": []},
        "extensions": {
            "head_joint": ExtensionValue(
                dtype="FLOAT32",
                shape=[2],
                data=struct.pack("<2f", 0.3, -0.1),
                mime_type="",
            )
        },
    }
    device_command = translator.r2c_to_device(step)
    assert device_command["arm_cmd_joint_pos"] == [0.1] * ARM_DOF
    assert device_command["gripper_cmd_pos"] == [0.4, 0.6]
    # extension 源以 ExtensionValue 透传，由适配器解码
    assert AgiBotG01HardwareAdapter.as_float_list(
        device_command["head_cmd_joint_pos"]
    ) == pytest.approx([0.3, -0.1])
    # 云端未下发 waist/wheel 时自动跳过
    assert "waist_cmd_joint_pos" not in device_command
    assert "wheel_cmd_speed" not in device_command

    # 端到端：翻译结果可直接送入适配器
    adapter.send_action(device_command)
    calls = {c[0]: c for c in _last_robot().calls}
    assert calls["move_arm"][1] == [0.1] * ARM_DOF
    assert calls["move_head"][1] == pytest.approx([0.3, -0.1])
