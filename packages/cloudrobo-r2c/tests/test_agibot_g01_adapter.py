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
        # 真实 a2d_sdk 返回 (图像, 时间戳)（见《指南》9.5 使用示例）
        return self.frames.get(camera_name), 1_000_000_000

    def close(self):
        self.closed = True


class FakeRobotController:
    """记录第 8 章 set_end_effector_pose_control / set_joint_position_control 调用。

    ``get_motion_status`` 返回固定 frames（含左右末端位姿），用于观测侧测试。
    """

    instances: List["FakeRobotController"] = []

    def __init__(self):
        type(self).instances.append(self)
        self.pose_calls: List[tuple] = []
        self.joint_calls: List[tuple] = []
        self.motion_status: Any = _fake_motion_status()

    def get_motion_status(self):
        return self.motion_status

    def set_end_effector_pose_control(
        self, lifetime, control_group, left_pose=None, right_pose=None
    ):
        self.pose_calls.append((float(lifetime), list(control_group), left_pose, right_pose))

    def set_joint_position_control(self, lifetime, joint_group):
        self.joint_calls.append((float(lifetime), dict(joint_group)))


def _fake_motion_status():
    """《指南》8.2.3 get_motion_status 返回结构示例（含左右末端 frames 位姿）。"""
    return {
        "timestamp": 1,
        "mode": {"value": 1, "name": "SERVO"},
        "error": {"code": 0, "message": "", "has_error": False},
        "frames": {
            "arm_left_link7": {
                "position": {"x": 0.13, "y": 0.89, "z": 0.85},
                "orientation": {"quaternion": {"x": -0.7071, "y": 0.0, "z": 0.0, "w": 0.7071}},
            },
            "arm_right_link7": {
                "position": {"x": 0.13, "y": -0.89, "z": 0.85},
                "orientation": {"quaternion": {"x": 0.0, "y": -0.7071, "z": 0.7071, "w": 0.0}},
            },
        },
        "collisions": [],
    }


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
    robot_mod.RobotController = FakeRobotController
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
    FakeRobotController.instances.clear()
    yield
    FakeRobotDds.instances.clear()
    FakeCosineCamera.instances.clear()
    FakeRobotController.instances.clear()


def _last_controller() -> FakeRobotController:
    assert FakeRobotController.instances, "FakeRobotController was never constructed"
    return FakeRobotController.instances[-1]


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
# 末端位姿观测（device_to_r2c，第 8 章 get_motion_status）
# ---------------------------------------------------------------------------


def test_observation_exposes_ee_poses():
    adapter = _connect()
    obs = adapter.get_observation()
    assert obs["left_ee_pose"] == pytest.approx([0.13, 0.89, 0.85, -0.7071, 0.0, 0.0, 0.7071])
    assert obs["right_ee_pose"] == pytest.approx([0.13, -0.89, 0.85, 0.0, -0.7071, 0.7071, 0.0])
    # 合并字段 ee_poses：按 left、right 顺序
    assert obs["ee_poses"][0] == pytest.approx(obs["left_ee_pose"])
    assert obs["ee_poses"][1] == pytest.approx(obs["right_ee_pose"])
    # 镜像进 joint_states 所需的平铺字段（names 与动作位姿路由约定一致）
    assert obs["ee_pose_names"] == ["left_pose", "right_pose"]
    assert obs["ee_poses_flat"] == pytest.approx(obs["left_ee_pose"] + obs["right_ee_pose"])


def test_observation_ee_poses_absent_without_motion_status():
    adapter = _connect()
    _last_controller().motion_status = None
    obs = adapter.get_observation()
    assert "left_ee_pose" not in obs
    assert "right_ee_pose" not in obs
    assert "ee_poses" not in obs


def test_observation_ee_poses_absent_when_motion_controller_disabled():
    adapter = _connect({"enable_motion_controller": False})
    obs = adapter.get_observation()
    assert "ee_poses" not in obs
    # 第 7 章关节观测不受影响
    assert len(obs["joint_positions"]) == ARM_DOF


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
    # joint_states：14 关节 + 末端位姿镜像（left_pose/right_pose 各 7 维）
    assert len(observation.joint_states.position) == 2 * ARM_DOF
    assert observation.joint_states.names[:ARM_DOF][0] == "left_arm_joint_1"
    assert list(observation.joint_states.names[ARM_DOF:]) == ["left_pose", "right_pose"]
    assert observation.joint_states.position[ARM_DOF:ARM_DOF + 7] == pytest.approx(
        [0.13, 0.89, 0.85, -0.7071, 0.0, 0.0, 0.7071]
    )
    assert observation.joint_states.position[ARM_DOF + 7:] == pytest.approx(
        [0.13, -0.89, 0.85, 0.0, -0.7071, 0.7071, 0.0]
    )
    assert len(observation.end_effector_states.position) == GRIPPER_DOF
    # 相机映射为 JPEG 字节
    jpeg = observation.images.color.get("cam_high")
    assert jpeg is not None and jpeg[:2] == b"\xff\xd8"
    # extensions：head/waist/hand 均以 FLOAT32 打包（body_pose 变长默认不映射）
    head_ext = observation.extensions["head_joint"]
    assert list(np.frombuffer(head_ext.data, dtype="<f4")) == pytest.approx([0.1, -0.2])
    assert observation.extensions["hand_joint"].shape == [12]
    # 末端位姿观测 → end_effector_poses（顺序与 names 对应：left, right）
    assert list(observation.end_effector_poses.names) == ["left", "right"]
    assert len(observation.end_effector_poses.pose) == 2
    assert observation.end_effector_poses.pose[0] == pytest.approx(
        [0.13, 0.89, 0.85, -0.7071, 0.0, 0.0, 0.7071]
    )

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
    # joint_states.names 映射为 joint_names（第 7/8 章路由依据；此处为空 → 第 7 章）
    assert device_command["joint_names"] == []


# ---------------------------------------------------------------------------
# 第 7 / 8 章路由（按 joint_states.names 区分，《指南》8.1）
# ---------------------------------------------------------------------------


LEFT_POSE = [0.2, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0]
RIGHT_POSE = [0.2, 0.0, 0.3, 0.0, 0.0, 0.0, -1.0]


def test_route_pose_via_joint_states_names():
    adapter = _connect()
    adapter.send_action(
        {
            "joint_names": ["left_pose", "right_pose"],
            "arm_cmd_joint_pos": LEFT_POSE + RIGHT_POSE,
        }
    )
    controller = _last_controller()
    lifetime, control_group, left_pose, right_pose = controller.pose_calls[0]
    assert lifetime == 1.0
    assert control_group == ["left_arm", "right_arm"]
    assert left_pose == pytest.approx(
        {"x": 0.2, "y": 0.0, "z": 0.3, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0}
    )
    assert right_pose == pytest.approx(
        {"x": 0.2, "y": 0.0, "z": 0.3, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": -1.0}
    )
    # 位姿路由不走第 7 章 move_arm
    assert not any(c[0] == "move_arm" for c in _last_robot().calls)


def test_route_pose_priority_joint_states_over_ee():
    # joint_states 位姿与 end_effector_poses.pose 同时存在：joint_states 优先
    adapter = _connect()
    adapter.send_action(
        {
            "joint_names": ["left_pose", "right_pose"],
            "arm_cmd_joint_pos": LEFT_POSE + RIGHT_POSE,
            "left_pose_cmd_ee": [9.0, 9.0, 9.0, 0.0, 0.0, 0.0, 1.0],
            "right_pose_cmd_ee": [9.0, 9.0, 9.0, 0.0, 0.0, 0.0, 1.0],
        }
    )
    _, _, left_pose, right_pose = _last_controller().pose_calls[0]
    assert left_pose == pytest.approx(
        {"x": 0.2, "y": 0.0, "z": 0.3, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0}
    )
    assert right_pose == pytest.approx(
        {"x": 0.2, "y": 0.0, "z": 0.3, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": -1.0}
    )


def test_route_pose_fallback_to_ee_channel():
    # joint_states 不含位姿名，末端位姿经 end_effector_poses.pose 下发
    adapter = _connect()
    adapter.send_action(
        {
            "joint_names": [f"left_arm_joint_{i}" for i in range(1, 8)]
            + [f"right_arm_joint_{i}" for i in range(1, 8)],
            "arm_cmd_joint_pos": [0.1] * ARM_DOF,
            "left_pose_cmd_ee": LEFT_POSE,
            "right_pose_cmd_ee": RIGHT_POSE,
        }
    )
    # 关节走第 7 章，位姿走第 8 章
    assert any(c[0] == "move_arm" for c in _last_robot().calls)
    _, _, left_pose, right_pose = _last_controller().pose_calls[0]
    assert left_pose == pytest.approx(
        {"x": 0.2, "y": 0.0, "z": 0.3, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0}
    )
    assert right_pose is not None


def test_route_joints8_via_group_names():
    # names 均为第 8 章部位组名（每组 7 关节）→ set_joint_position_control
    adapter = _connect()
    left_group = [0.1] * 7
    right_group = [0.2] * 7
    adapter.send_action(
        {
            "joint_names": ["left_arm", "right_arm"],
            "arm_cmd_joint_pos": left_group + right_group,
        }
    )
    controller = _last_controller()
    lifetime, joint_group = controller.joint_calls[0]
    assert lifetime == 1.0
    assert joint_group["left_arm"] == pytest.approx(left_group)
    assert joint_group["right_arm"] == pytest.approx(right_group)
    assert not any(c[0] == "move_arm" for c in _last_robot().calls)
    assert controller.pose_calls == []


def test_route_joints7_with_arm_joint_names():
    # names 为双臂关节名 → 第 7 章 move_arm（默认路径）
    adapter = _connect()
    arm = [0.1] * ARM_DOF
    adapter.send_action(
        {
            "joint_names": [f"left_arm_joint_{i}" for i in range(1, 8)]
            + [f"right_arm_joint_{i}" for i in range(1, 8)],
            "arm_cmd_joint_pos": arm,
        }
    )
    assert not _last_controller().pose_calls
    assert not _last_controller().joint_calls
    assert any(c[0] == "move_arm" for c in _last_robot().calls)


def test_route_pose_rejects_wrong_dims():
    adapter = _connect()
    adapter.send_action(
        {
            "joint_names": ["left_pose"],
            "arm_cmd_joint_pos": [0.1, 0.2, 0.3],  # 不足 7 维
        }
    )
    assert _last_controller().pose_calls == []


def test_translator_maps_ee_pose_and_joint_names():
    from cloudrobo_r2c.common.models import Pose7D
    from cloudrobo_r2c.translators.device_translator import ConfigurableDeviceTranslator

    config = _load_robot_config()
    translator = ConfigurableDeviceTranslator.from_config(config)
    step = {
        "timestamp": 1,
        "id": 1,
        "chunk_index": 0,
        "joint_states": {
            "names": ["left_pose", "right_pose"],
            "position": LEFT_POSE + RIGHT_POSE,
            "velocity": [],
            "torque": [],
        },
        "end_effector_states": {"names": [], "position": [], "velocity": [], "torque": []},
        "end_effector_poses": {
            # 与 sync_client._extract_action_step 的 per-step 结构一致（pose 为 Pose7D）
            "left": {"pose": Pose7D(list(LEFT_POSE))},
            "right": {"pose": Pose7D(list(RIGHT_POSE))},
        },
        "localization": {"odom_pose": [], "map_pose": []},
        "extensions": {},
    }
    device_command = translator.r2c_to_device(step)
    assert device_command["joint_names"] == ["left_pose", "right_pose"]
    # joint_states 位姿按 names 选取（source_name_ref 语义等价：索引 0/1 各 7 维）
    assert AgiBotG01HardwareAdapter.as_float_list(device_command["arm_cmd_joint_pos"]) == (
        pytest.approx(LEFT_POSE + RIGHT_POSE)
    )
    # ee 通道：per-step pose 为 7 元素列表，映射为 *_pose_cmd_ee
    assert AgiBotG01HardwareAdapter.as_float_list(device_command["left_pose_cmd_ee"]) == (
        pytest.approx(LEFT_POSE)
    )
    assert AgiBotG01HardwareAdapter.as_float_list(device_command["right_pose_cmd_ee"]) == (
        pytest.approx(RIGHT_POSE)
    )
    # 端到端：joint_states 位姿优先于 ee 通道
    adapter = _connect()
    adapter.send_action(device_command)
    _, _, left_pose, right_pose = _last_controller().pose_calls[0]
    assert left_pose == pytest.approx(
        {"x": 0.2, "y": 0.0, "z": 0.3, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0}
    )
    assert right_pose is not None
    assert not any(c[0] == "move_arm" for c in _last_robot().calls)


# ---------------------------------------------------------------------------
# 样例 cloud-adapter（examples/agibot_g01_cloud_adapter.py）
# ---------------------------------------------------------------------------


def _load_example_module():
    pytest.importorskip("zenoh")
    examples_dir = Path(agibot_g01_client.__file__).resolve().parents[4] / "examples"
    examples_dir_str = str(examples_dir)
    if examples_dir_str not in sys.path:
        sys.path.insert(0, examples_dir_str)
    import agibot_g01_cloud_adapter as demo

    return demo


def _demo_observation():
    return types.SimpleNamespace(
        joint_states=types.SimpleNamespace(position=[0.1] * ARM_DOF),
        end_effector_states=types.SimpleNamespace(position=[0.3, 0.3]),
        end_effector_poses=types.SimpleNamespace(
            names=["left", "right"],
            pose=[[0.2, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0]] * 2,
        ),
    )


def test_example_ch8_demo_groups():
    """--include-ch8：第 8 章三种示例组（names 位姿 / ee 通道 / 组名关节）。"""
    demo = _load_example_module()
    cycler = demo.ThreeGroupActionCycler(
        steps=5,
        sequence=demo.CH8_CYCLE_SEQUENCE,
        include_ch8=True,
        pose_offset_z=0.05,
    )
    current = demo._extract_current(_demo_observation())
    now = 1000

    def next_action():
        return cycler.build_action(now, current["joints"], current["gripper"], current["poses"])

    # 前 4 步为第 7 章：0 → 1 → 2 → 1
    for _ in range(3):
        next_action()
    a4 = next_action()
    assert a4.joint_states.names == [f"left_arm_joint_{i}" for i in range(1, 8)] + [
        f"right_arm_joint_{i}" for i in range(1, 8)
    ]
    # 组 3：末端位姿经 joint_states names（当前位姿 + z 偏移）
    a5 = next_action()
    assert a5.joint_states.names == ["left_pose", "right_pose"]
    row = a5.joint_states.position[0]
    assert row[:7] == pytest.approx([0.2, 0.0, 0.35, 0.0, 0.0, 0.0, 1.0])
    assert row[7:] == pytest.approx([0.2, 0.0, 0.35, 0.0, 0.0, 0.0, 1.0])
    # 组 4：ee 通道（joint_states 保持当前关节）
    a6 = next_action()
    assert list(a6.end_effector_poses["left"].pose[0]) == pytest.approx(
        [0.2, 0.0, 0.35, 0.0, 0.0, 0.0, 1.0]
    )
    assert a6.joint_states.position[0] == pytest.approx([0.1] * ARM_DOF)
    # 组 5：组名关节（复用 GROUP_C 真机值）
    a7 = next_action()
    assert a7.joint_states.names == ["left_arm", "right_arm"]
    assert a7.joint_states.position[0][:7] == pytest.approx(demo.GROUP_C["joints"][:7])
    # 往复：5 → 4 → 3 → 2 ...
    a8 = next_action()
    assert list(a8.end_effector_poses["left"].pose[0]) == pytest.approx(
        [0.2, 0.0, 0.35, 0.0, 0.0, 0.0, 1.0]
    )
    a9 = next_action()
    assert a9.joint_states.position[-1] == pytest.approx(demo.GROUP_A["joints"])


def test_example_ch8_pose_group_skips_without_ee_poses():
    demo = _load_example_module()
    cycler = demo.ThreeGroupActionCycler(
        steps=3,
        sequence=demo.CH8_CYCLE_SEQUENCE,
        include_ch8=True,
        pose_offset_z=0.05,
    )
    observation = _demo_observation()
    observation.end_effector_poses.pose = []
    current = demo._extract_current(observation)

    def next_action():
        return cycler.build_action(1000, current["joints"], current["gripper"], current["poses"])

    # 走到组 3（joint_states 位姿）时缺观测位姿 → 返回 None 跳过该轮
    for _ in range(4):
        assert next_action() is not None
    assert next_action() is None
    # 后续组 4（ee 通道）同样缺位姿 → None；组 5 正常
    assert next_action() is None
    assert next_action() is not None


def test_example_cycler_cycles_three_groups():
    demo = _load_example_module()
    cycler = demo.ThreeGroupActionCycler(steps=5)
    current = demo._extract_current(_demo_observation())
    now = 1000

    def next_action():
        return cycler.build_action(now, current["joints"], current["gripper"])

    # 组 0：关节 A（当前 → 目标插值 5 步，最后一步到达目标；夹爪任意角度插值）
    a1 = next_action()
    assert a1.chunk_size == 5
    assert a1.joint_states.position[-1] == pytest.approx(demo.GROUP_A["joints"])
    assert a1.end_effector_states.position[-1] == pytest.approx([0.15, 0.15])
    # 组 1：关节 B
    a2 = next_action()
    assert a2.joint_states.position[-1] == pytest.approx(demo.GROUP_B["joints"])
    assert a2.end_effector_states.position[-1] == pytest.approx([0.85, 0.35])
    # 组 2：真机调试后为纯关节组（目标为真机采集值，不带 head/waist extensions）
    a3 = next_action()
    assert a3.chunk_size == 5
    assert a3.joint_states.position[-1] == pytest.approx(demo.GROUP_C["joints"])
    assert a3.end_effector_states.position[-1] == pytest.approx([0.5, 0.8])
    assert "head_joint" not in a3.extensions
    assert "waist_joint" not in a3.extensions
    # 往复：2 → 1 → 0
    a4 = next_action()
    assert a4.joint_states.position[-1] == pytest.approx(demo.GROUP_B["joints"])
    a5 = next_action()
    assert a5.joint_states.position[-1] == pytest.approx(demo.GROUP_A["joints"])
