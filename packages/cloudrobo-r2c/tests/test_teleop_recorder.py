"""TeleopSessionRecorder 与遥操配置加载测试。"""

from __future__ import annotations

import textwrap

import pytest
from cloudrobo_r2c.common.utils.safe_serialization import load_frames
from cloudrobo_r2c.teleop.cli import load_teleop_config
from cloudrobo_r2c.teleop.recording import TeleopSessionRecorder


def test_recorder_roundtrip_with_bytes_and_nested_structures(tmp_path):
    path = str(tmp_path / "teleop.r2cr")
    recorder = TeleopSessionRecorder(path, source_name="lerobot_leader")
    observation = {
        "timestamp": 12345,
        "task": "pick place",
        "joint_states": {"names": ["joint_1", "joint_2"], "position": [0.1, -0.2]},
        "images": {
            "color": {"front": b"\xff\xd8front-jpeg", "wrist": b"\xff\xd8wrist-jpeg"},
            "depth": {},
        },
    }
    action = {
        "timestamp": 12346,
        "id": 0,
        "chunk_size": 1,
        "joint_states": {"names": ["gripper"], "position": [[50.0]]},
        "end_effector_poses": {"left_ee": [[0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0]]},
    }
    source = {"joint_positions": {}, "gripper": 0.5}
    recorder.record(observation, action, source)
    recorder.record({"timestamp": 12347}, {"id": 1}, None)
    recorder.close()

    frames = load_frames(path)
    assert len(frames) == 2
    first = frames[0]
    assert first["timestamp_ms"] > 0
    assert first["source_name"] == "lerobot_leader"
    assert first["observation"]["images"]["color"]["front"] == b"\xff\xd8front-jpeg"
    assert first["observation"]["joint_states"]["names"] == ["joint_1", "joint_2"]
    assert first["action"]["end_effector_poses"]["left_ee"][0][0] == pytest.approx(0.1)
    assert first["source"]["gripper"] == pytest.approx(0.5)
    assert frames[1]["observation"] == {"timestamp": 12347}
    assert frames[1]["action"]["id"] == 1


def test_recorder_record_after_close_raises(tmp_path):
    recorder = TeleopSessionRecorder(str(tmp_path / "teleop.r2cr"))
    recorder.close()
    with pytest.raises(RuntimeError, match="closed"):
        recorder.record({}, {})


def test_recorder_close_is_idempotent(tmp_path):
    path = str(tmp_path / "teleop.r2cr")
    recorder = TeleopSessionRecorder(path)
    recorder.record({"a": 1}, {"b": 2})
    recorder.close()
    recorder.close()  # 第二次 close 不应报错或重复写文件
    assert len(load_frames(path)) == 1


# ── 配置加载 ─────────────────────────────────────────────────────────────


def _write(tmp_path, content: str) -> str:
    config_path = tmp_path / "teleop_config.yaml"
    config_path.write_text(textwrap.dedent(content), encoding="utf-8")
    return str(config_path)


def test_load_teleop_config_returns_teleop_section(tmp_path):
    path = _write(
        tmp_path,
        """
        teleop:
          source:
            type: mock_joints
          action:
            mode: joint
            joint_map: {"a": "joint_1"}
          recording:
            enabled: false
        """,
    )
    cfg = load_teleop_config(path)
    assert cfg["source"]["type"] == "mock_joints"
    assert cfg["action"]["joint_map"] == {"a": "joint_1"}


def test_load_teleop_config_missing_section_rejected(tmp_path):
    path = _write(tmp_path, "something: else\n")
    with pytest.raises(ValueError, match="teleop"):
        load_teleop_config(path)


def test_load_teleop_config_non_mapping_rejected(tmp_path):
    path = _write(tmp_path, "- a\n- b\n")
    with pytest.raises(ValueError, match="mapping"):
        load_teleop_config(path)


def test_shipped_operator_configs_are_valid():
    """随包发布的两份遥操配置必须能被 OperatorConfig 正确解析。"""
    import os

    from cloudrobo_r2c.teleop.operator import OperatorConfig

    config_dir = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "src",
        "cloudrobo_r2c",
        "config",
    )
    for name in ("teleop_so101_leader.yaml",):
        cfg = load_teleop_config(os.path.join(config_dir, name))
        operator_config = OperatorConfig.from_cfg(cfg)
        operator_config.validate()

