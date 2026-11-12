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
        self.motion_mode_calls: List[int] = []
        self.motion_status: Any = _fake_motion_status()

    def get_motion_status(self):
        return self.motion_status

    def set_motion_control_mode(self, mode):
        self.motion_mode_calls.append(int(mode))
        self.motion_status["mode"] = {
            "value": int(mode),
            "name": {1: "SERVO", 2: "PLANNING"}.get(int(mode), "?"),
        }

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
    # 镜像进 joint_states 所需的逐维 names（四元数 + 欧拉角两种表示同时携带）
    assert obs["ee_pose_names"] == (
        _pose_dim_names("left", "quat")
        + _pose_dim_names("left", "euler")[3:]
        + _pose_dim_names("right", "quat")
        + _pose_dim_names("right", "euler")[3:]
    )
    from cloudrobo_r2c.robots.agibot_g01.agibot_g01_hardware_adapter import (
        _quat_to_euler_rpy,
    )

    left_rpy = _quat_to_euler_rpy(-0.7071, 0.0, 0.0, 0.7071)
    right_rpy = _quat_to_euler_rpy(0.0, -0.7071, 0.7071, 0.0)
    assert obs["ee_poses_flat"][:7] == pytest.approx(obs["left_ee_pose"])
    assert obs["ee_poses_flat"][7:10] == pytest.approx(left_rpy)
    assert obs["ee_poses_flat"][10:17] == pytest.approx(obs["right_ee_pose"])
    assert obs["ee_poses_flat"][17:20] == pytest.approx(right_rpy)


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
    # joint_states：14 关节 + 位姿镜像（quat 7 + euler 3 每侧，共 20） + 夹爪 2
    assert len(observation.joint_states.position) == ARM_DOF + 20 + GRIPPER_DOF
    assert observation.joint_states.names[:ARM_DOF][0] == "left_arm_joint_1"
    assert list(observation.joint_states.names[ARM_DOF : ARM_DOF + 7]) == _pose_dim_names("left")
    assert list(observation.joint_states.names[ARM_DOF + 7 : ARM_DOF + 10]) == [
        "left_pose_roll",
        "left_pose_pitch",
        "left_pose_yaw",
    ]
    assert list(observation.joint_states.names[ARM_DOF + 10 : ARM_DOF + 17]) == _pose_dim_names(
        "right"
    )
    assert list(observation.joint_states.names[ARM_DOF + 17 : ARM_DOF + 20]) == [
        "right_pose_roll",
        "right_pose_pitch",
        "right_pose_yaw",
    ]
    assert observation.joint_states.position[ARM_DOF : ARM_DOF + 7] == pytest.approx(
        [0.13, 0.89, 0.85, -0.7071, 0.0, 0.0, 0.7071]
    )
    from cloudrobo_r2c.robots.agibot_g01.agibot_g01_hardware_adapter import (
        _quat_to_euler_rpy,
    )

    left_rpy = _quat_to_euler_rpy(-0.7071, 0.0, 0.0, 0.7071)
    right_rpy = _quat_to_euler_rpy(0.0, -0.7071, 0.7071, 0.0)
    assert observation.joint_states.position[ARM_DOF + 7 : ARM_DOF + 10] == pytest.approx(
        list(left_rpy)
    )
    assert observation.joint_states.position[ARM_DOF + 10 : ARM_DOF + 17] == pytest.approx(
        [0.13, -0.89, 0.85, 0.0, -0.7071, 0.7071, 0.0]
    )
    assert observation.joint_states.position[ARM_DOF + 17 : ARM_DOF + 20] == pytest.approx(
        list(right_rpy)
    )
    assert list(observation.joint_states.names[-GRIPPER_DOF:]) == [
        "left_gripper",
        "right_gripper",
    ]
    assert observation.joint_states.position[-GRIPPER_DOF:] == pytest.approx([0.25, 0.75])
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


def _pose_dim_names(side: str, pose_repr: str = "quat") -> List[str]:
    """逐维位姿 names：四元数 x/y/z/rx/ry/rz/rw 或欧拉角姿态 x/y/z/roll/pitch/yaw。"""
    dims = (
        ["x", "y", "z", "rx", "ry", "rz", "rw"]
        if pose_repr == "quat"
        else ["x", "y", "z", "roll", "pitch", "yaw"]
    )
    return [f"{side}_pose_{d}" for d in dims]


def test_route_pose_via_joint_states_names():
    adapter = _connect()
    adapter.send_action(
        {
            "joint_names": _pose_dim_names("left") + _pose_dim_names("right"),
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


def test_route_pose_euler_names_converted_to_quaternion():
    # 欧拉角表示（roll/pitch/yaw，ZYX）：适配器内部转换为四元数下发

    from cloudrobo_r2c.robots.agibot_g01.agibot_g01_hardware_adapter import (
        _euler_rpy_to_quat,
    )

    adapter = _connect()
    roll, pitch, yaw = 0.1, -0.2, 0.3
    qx, qy, qz, qw = _euler_rpy_to_quat(roll, pitch, yaw)
    names = []
    values = []
    for side in ("left", "right"):
        names += [f"{side}_pose_x", f"{side}_pose_y", f"{side}_pose_z"]
        names += [f"{side}_pose_roll", f"{side}_pose_pitch", f"{side}_pose_yaw"]
        values += [0.2, 0.0, 0.3, roll, pitch, yaw]
    adapter.send_action({"joint_names": names, "arm_cmd_joint_pos": values})
    _, _, left_pose, right_pose = _last_controller().pose_calls[0]
    expected_quat = {"x": 0.2, "y": 0.0, "z": 0.3, "qx": qx, "qy": qy, "qz": qz, "qw": qw}
    assert left_pose == pytest.approx(expected_quat)
    assert right_pose == pytest.approx(expected_quat)


def test_route_gripper_via_joint_states():
    # 夹爪经 joint_states 逐维 names（left_gripper/right_gripper）→ move_gripper
    adapter = _connect()
    adapter.send_action(
        {
            "joint_names": ["left_gripper", "right_gripper"],
            "arm_cmd_joint_pos": [0.25, 0.9],
        }
    )
    assert any(
        c[0] == "move_gripper" and c[1] == pytest.approx([0.25, 0.9]) for c in _last_robot().calls
    )


def test_route_reject_pose_in_both_channels():
    # 末端位姿同时出现在 joint_states 与 end_effector_poses → 拒绝执行
    adapter = _connect()
    adapter.send_action(
        {
            "joint_names": _pose_dim_names("left") + _pose_dim_names("right"),
            "arm_cmd_joint_pos": LEFT_POSE + RIGHT_POSE,
            "left_pose_cmd_ee": [9.0, 9.0, 9.0, 0.0, 0.0, 0.0, 1.0],
            "right_pose_cmd_ee": [9.0, 9.0, 9.0, 0.0, 0.0, 0.0, 1.0],
        }
    )
    assert _last_controller().pose_calls == []


def test_route_joints8_per_joint_names():
    # 第 8 章关节控制：left_arm_joint1..7 / right_arm_joint1..7（不带下划线）
    adapter = _connect()
    left_group = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
    right_group = [0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4]
    names = [f"left_arm_joint{i}" for i in range(1, 8)]
    names += [f"right_arm_joint{i}" for i in range(1, 8)]
    adapter.send_action({"joint_names": names, "arm_cmd_joint_pos": left_group + right_group})
    controller = _last_controller()
    lifetime, joint_group = controller.joint_calls[0]
    assert lifetime == 1.0
    assert joint_group["left_arm"] == pytest.approx(left_group)
    assert joint_group["right_arm"] == pytest.approx(right_group)
    assert not any(c[0] == "move_arm" for c in _last_robot().calls)
    # 第 8 章下发前自动切 planning 模式
    assert controller.motion_mode_calls == [2]


def test_motion_mode_switches_between_ch7_and_ch8():
    # 第 7 章(servo=1) ↔ 第 8 章(planning=2) 切换时自动 set_motion_control_mode
    adapter = _connect()
    pose_names = _pose_dim_names("left") + _pose_dim_names("right")
    adapter.send_action({"joint_names": pose_names, "arm_cmd_joint_pos": LEFT_POSE + RIGHT_POSE})
    controller = _last_controller()
    assert controller.motion_mode_calls == [2]
    # 切回第 7 章：下发 servo 模式
    adapter.send_action(
        {
            "joint_names": [f"left_arm_joint_{i}" for i in range(1, 8)]
            + [f"right_arm_joint_{i}" for i in range(1, 8)],
            "arm_cmd_joint_pos": [0.1] * ARM_DOF,
        }
    )
    assert controller.motion_mode_calls == [2, 1]
    # 同章节重复下发不再重复切换
    adapter.send_action(
        {
            "joint_names": [f"left_arm_joint_{i}" for i in range(1, 8)]
            + [f"right_arm_joint_{i}" for i in range(1, 8)],
            "arm_cmd_joint_pos": [0.1] * ARM_DOF,
        }
    )
    assert controller.motion_mode_calls == [2, 1]


def test_motion_mode_auto_disabled():
    # auto_motion_control_mode: false → 不自动切换模式
    adapter = _connect({"auto_motion_control_mode": False})
    pose_names = _pose_dim_names("left") + _pose_dim_names("right")
    adapter.send_action({"joint_names": pose_names, "arm_cmd_joint_pos": LEFT_POSE + RIGHT_POSE})
    assert _last_controller().motion_mode_calls == []


def test_route_reject_partial_joints8():
    # 第 8 章关节名不全（缺 joint7）→ 不硬执行
    adapter = _connect()
    names = [f"left_arm_joint{i}" for i in range(1, 7)]  # 缺 7
    names += [f"right_arm_joint{i}" for i in range(1, 8)]
    adapter.send_action({"joint_names": names, "arm_cmd_joint_pos": [0.1] * 13})
    assert _last_controller().joint_calls == []


def test_route_reject_mixed_ch7_ch8_joint_names():
    # 第 7 章（带下划线）与第 8 章（不带下划线）关节名混用 → 拒绝
    adapter = _connect()
    names = ["left_arm_joint_1", "left_arm_joint1"]
    adapter.send_action({"joint_names": names, "arm_cmd_joint_pos": [0.1, 0.2]})
    assert not any(c[0] == "move_arm" for c in _last_robot().calls)
    assert _last_controller().joint_calls == []


def test_route_pose_fallback_to_ee_channel():
    # joint_states 仅夹爪分量（无关节/位姿分量）→ ee 通道位姿兜底 + 夹爪下发
    adapter = _connect()
    adapter.send_action(
        {
            "joint_names": ["left_gripper", "right_gripper"],
            "arm_cmd_joint_pos": [0.3, 0.3],
            "left_pose_cmd_ee": LEFT_POSE,
            "right_pose_cmd_ee": RIGHT_POSE,
        }
    )
    assert any(c[0] == "move_gripper" for c in _last_robot().calls)
    _, _, left_pose, right_pose = _last_controller().pose_calls[0]
    assert left_pose == pytest.approx(
        {"x": 0.2, "y": 0.0, "z": 0.3, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0}
    )
    assert right_pose is not None


def test_route_reject_joint_and_ee_pose_together():
    # 关节与末端互斥：joint_states 含关节分量时不得经 ee 通道下发位姿
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
    assert not any(c[0] == "move_arm" for c in _last_robot().calls)
    assert _last_controller().pose_calls == []


def test_route_reject_mixed_joint_and_pose_names():
    # 关节与末端互斥：同一动作不得同时包含关节分量与位姿分量
    adapter = _connect()
    adapter.send_action(
        {
            "joint_names": [f"left_arm_joint_{i}" for i in range(1, 8)]
            + [f"right_arm_joint_{i}" for i in range(1, 8)]
            + ["left_pose_x", "left_pose_y", "left_pose_z"]
            + ["left_pose_rx", "left_pose_ry", "left_pose_rz", "left_pose_rw"]
            + ["right_pose_x", "right_pose_y", "right_pose_z"]
            + ["right_pose_rx", "right_pose_ry", "right_pose_rz", "right_pose_rw"],
            "arm_cmd_joint_pos": [0.1] * ARM_DOF + LEFT_POSE + RIGHT_POSE,
        }
    )
    assert not any(c[0] == "move_arm" for c in _last_robot().calls)
    assert _last_controller().pose_calls == []
    assert _last_controller().joint_calls == []


def test_route_reject_mixed_quat_and_euler():
    # 四元数与欧拉角互斥：同一侧同时出现两种表示 → 拒绝
    adapter = _connect()
    names = ["left_pose_x", "left_pose_y", "left_pose_z", "left_pose_rx"]
    names += ["left_pose_roll", "left_pose_pitch", "left_pose_yaw"]
    adapter.send_action({"joint_names": names, "arm_cmd_joint_pos": [0.1] * 7})
    assert _last_controller().pose_calls == []

    # 跨侧表示不一致同样拒绝
    names = _pose_dim_names("left") + [
        "right_pose_x",
        "right_pose_y",
        "right_pose_z",
        "right_pose_roll",
        "right_pose_pitch",
        "right_pose_yaw",
    ]
    adapter.send_action({"joint_names": names, "arm_cmd_joint_pos": LEFT_POSE + [0.1] * 6})
    assert _last_controller().pose_calls == []


def test_route_reject_partial_joints():
    # 关节不全（仅 6 个关节名）→ 不硬执行（不做补帧兜底）
    adapter = _connect()
    adapter.send_action(
        {
            "joint_names": [f"left_arm_joint_{i}" for i in range(1, 7)],
            "arm_cmd_joint_pos": [0.1] * 6,
        }
    )
    assert not any(c[0] == "move_arm" for c in _last_robot().calls)


def test_route_reject_partial_group():
    # 组关节数不全（每组须 7 维）→ 拒绝
    adapter = _connect()
    adapter.send_action(
        {
            "joint_names": ["left_arm", "right_arm"],
            "arm_cmd_joint_pos": [0.1] * 7 + [0.2] * 5,  # right_arm 仅 5 维
        }
    )
    assert _last_controller().joint_calls == []


def test_route_reject_partial_gripper():
    # 夹爪仅一侧 → 不硬执行
    adapter = _connect()
    adapter.send_action(
        {
            "joint_names": ["left_gripper"],
            "arm_cmd_joint_pos": [0.25],
        }
    )
    assert not any(c[0] == "move_gripper" for c in _last_robot().calls)


def test_route_reject_unknown_names():
    # 未知 names 视为协议不一致 → 拒绝执行
    adapter = _connect()
    adapter.send_action(
        {
            "joint_names": ["left_pose_w", "right_pose_w"],
            "arm_cmd_joint_pos": [0.1, 0.1],
        }
    )
    assert _last_controller().pose_calls == []
    assert not any(c[0] == "move_arm" for c in _last_robot().calls)


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
    # 位姿分量不完整（只有 x/y，缺 z 与姿态）→ 不下发
    adapter.send_action(
        {
            "joint_names": ["left_pose_x", "left_pose_y"],
            "arm_cmd_joint_pos": [0.1, 0.2],
        }
    )
    assert _last_controller().pose_calls == []


def test_translator_maps_ee_pose_and_joint_names():
    from cloudrobo_r2c.common.models import Pose7D
    from cloudrobo_r2c.translators.device_translator import ConfigurableDeviceTranslator

    config = _load_robot_config()
    translator = ConfigurableDeviceTranslator.from_config(config)
    pose_names = _pose_dim_names("left") + _pose_dim_names("right")
    step = {
        "timestamp": 1,
        "id": 1,
        "chunk_index": 0,
        "joint_states": {
            "names": pose_names,
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
    assert device_command["joint_names"] == pose_names
    # joint_states 位姿逐维值全量透传（适配器按 names 解析重组）
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
    # 端到端：末端位姿同时出现在 joint_states 与 end_effector_poses → 拒绝执行
    adapter = _connect()
    adapter.send_action(device_command)
    assert _last_controller().pose_calls == []
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
    )


# 第 8 章示例组（与样例中注释掉的 GROUP_POSE / GROUP_JOINTS8 同构）
_CH8_POSE_GROUP = {
    "name": "ch8_pose(绝对位姿)",
    "kind": "ch8_pose",
    "left_pose": LEFT_POSE,
    "right_pose": RIGHT_POSE,
}
_CH8_JOINTS8_GROUP = {
    "name": "ch8_joints(组名关节)",
    "kind": "ch8_joints",
    "joints": [0.2] * ARM_DOF,
    "gripper": [0.5, 0.8],
}


def test_example_ch8_pose_group_joint_states_channel():
    """ch8_pose 组：绝对位姿经 joint_states 逐维 names（四元数）+ 夹爪。"""
    demo = _load_example_module()
    cycler = demo.ActionGroupCycler(
        groups=[dict(_CH8_POSE_GROUP)],
        steps=5,
        pose_channel="joint_states",
        pose_repr="quat",
    )
    current = demo._extract_current(_demo_observation())
    a = cycler.build_action(1000, current["joints"], current["gripper"])
    assert a.joint_states.names == (
        demo._pose_dim_names("left")
        + demo._pose_dim_names("right")
        + ["left_gripper", "right_gripper"]
    )
    row = a.joint_states.position[0]
    assert row[:7] == pytest.approx(LEFT_POSE)
    assert row[7:14] == pytest.approx(RIGHT_POSE)
    assert row[14:] == pytest.approx([0.3, 0.3])  # 夹爪保持当前角度
    assert a.end_effector_poses == {}


def test_example_ch8_pose_group_ee_channel_and_euler_repr():
    """ch8_pose 组：--pose-channel ee --pose-repr euler。

    关节与末端互斥 → joint_states 仅携带夹爪分量；ee 通道为四元数原生格式
    （欧拉角在云端先转换为四元数）。
    """
    demo = _load_example_module()
    cycler = demo.ActionGroupCycler(
        groups=[dict(_CH8_POSE_GROUP)],
        steps=5,
        pose_channel="ee",
        pose_repr="euler",
    )
    current = demo._extract_current(_demo_observation())
    a = cycler.build_action(1000, current["joints"], current["gripper"])
    assert a.joint_states.names == ["left_gripper", "right_gripper"]
    assert a.joint_states.position[0] == pytest.approx([0.3, 0.3])
    left_pose = list(a.end_effector_poses["left"].pose[0])
    assert left_pose == pytest.approx(LEFT_POSE)
    assert list(a.end_effector_poses["right"].pose[0]) == pytest.approx(RIGHT_POSE)


def test_example_ch8_joints_group_per_joint_names():
    """ch8_joints 组：left_arm_joint1..7 / right_arm_joint1..7 逐关节名。"""
    demo = _load_example_module()
    cycler = demo.ActionGroupCycler(
        groups=[dict(_CH8_JOINTS8_GROUP)],
        steps=5,
    )
    current = demo._extract_current(_demo_observation())
    a = cycler.build_action(1000, current["joints"], current["gripper"])
    assert a.joint_states.names == [f"left_arm_joint{i}" for i in range(1, 8)] + [
        f"right_arm_joint{i}" for i in range(1, 8)
    ]
    assert a.joint_states.position[-1] == pytest.approx(_CH8_JOINTS8_GROUP["joints"])
    assert a.end_effector_poses == {}


def test_example_cycler_cycles_groups_ping_pong():
    """GROUPS dict 数组驱动：默认往复序列 0 → 1 → 2 → 1 → 0。"""
    demo = _load_example_module()
    cycler = demo.ActionGroupCycler(groups=demo.GROUPS, steps=5)
    current = demo._extract_current(_demo_observation())
    now = 1000

    def next_action():
        return cycler.build_action(now, current["joints"], current["gripper"])

    # 组 0：关节 A（当前 → 目标插值 5 步，最后一步到达目标；夹爪任意角度）
    a1 = next_action()
    assert a1.chunk_size == 5
    assert a1.joint_states.position[-1] == pytest.approx(demo.GROUP_A["joints"])
    assert a1.end_effector_states.position[-1] == pytest.approx([0.15, 0.15])
    # 组 1：关节 B
    a2 = next_action()
    assert a2.joint_states.position[-1] == pytest.approx(demo.GROUP_B["joints"])
    assert a2.end_effector_states.position[-1] == pytest.approx([0.85, 0.35])
    # 组 2：全身姿态（真机值，不带 head/waist extensions）
    a3 = next_action()
    assert a3.joint_states.position[-1] == pytest.approx(demo.GROUP_C["joints"])
    assert a3.end_effector_states.position[-1] == pytest.approx([0.5, 0.8])
    assert "head_joint" not in a3.extensions
    assert "waist_joint" not in a3.extensions
    # 往复：2 → 1 → 0
    a4 = next_action()
    assert a4.joint_states.position[-1] == pytest.approx(demo.GROUP_B["joints"])
    a5 = next_action()
    assert a5.joint_states.position[-1] == pytest.approx(demo.GROUP_A["joints"])
    # 夹爪全程插值：首步从当前角度开始（0.3 → 0.15 五步插值）
    assert a1.end_effector_states.position[0] == pytest.approx([0.27, 0.27])


def test_example_ch8_pose_interp_joint_states_channel():
    """ch8_pose 组（joint_states 通道）：位姿全程插值（平移线性 + SLERP）。"""
    demo = _load_example_module()
    target_left = [0.3, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0]
    group_pose = {
        "name": "ch8_pose(绝对位姿)",
        "kind": "ch8_pose",
        "left_pose": target_left,
        "right_pose": [0.3, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0],
    }
    cycler = demo.ActionGroupCycler(groups=[group_pose], steps=5)
    observation = _demo_observation()
    observation.end_effector_poses = types.SimpleNamespace(
        names=["left", "right"],
        pose=[[0.2, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0]] * 2,
    )
    current = demo._extract_current(observation)
    a = cycler.build_action(1000, current["joints"], current["gripper"], current["poses"])
    assert a is not None
    # 逐步 z 从 0.3 线性逼近 0.3（start z=0.3 → target z=0.3，此处验证 x：0.2→0.3）
    xs = [row[0] for row in a.joint_states.position]
    assert xs == pytest.approx([0.22, 0.24, 0.26, 0.28, 0.30])
    # 每步姿态为连续四元数（无跳变）
    for i in range(4):
        assert abs(a.joint_states.position[i][5] - a.joint_states.position[i + 1][5]) < 0.05
    # 最后一步到达目标
    assert a.joint_states.position[-1][:7] == pytest.approx(target_left)


def test_example_ch8_joints_group_interpolates():
    """ch8_joints 组：关节与夹爪均从当前值线性插值到目标。"""
    demo = _load_example_module()
    cycler = demo.ActionGroupCycler(groups=[dict(_CH8_JOINTS8_GROUP)], steps=5)
    current = demo._extract_current(_demo_observation())
    a = cycler.build_action(1000, current["joints"], current["gripper"])
    # 首步 0.1 → 0.2 五步插值
    assert a.joint_states.position[0] == pytest.approx([0.12] * ARM_DOF)
    assert a.joint_states.position[-1] == pytest.approx([0.2] * ARM_DOF)
    # 夹爪同样插值（当前 0.3 → 目标 0.5）
    assert a.end_effector_states.position[0] == pytest.approx([0.34, 0.4])
    assert a.end_effector_states.position[-1] == pytest.approx([0.5, 0.8])
