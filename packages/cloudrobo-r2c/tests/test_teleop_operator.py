"""TeleopOperator 单元测试（FakeSession + 脚本化输入源）。"""

from __future__ import annotations

from typing import Any, Dict, List, Optional

import pytest
from cloudrobo_r2c.teleop.base import TeleopSource, TeleopTarget
from cloudrobo_r2c.teleop.operator import OperatorConfig, TeleopOperator
from cloudrobo_r2c.teleop.recording import TeleopSessionRecorder


class FakeSession:
    """记录 publish_actions 并持有观测回调的测试会话。"""

    def __init__(self):
        self.published: List[Any] = []
        self.observation_callback = None
        self.decode_images = None

    def subscribe_observations(self, callback, decode_images: bool = True):
        self.observation_callback = callback
        self.decode_images = decode_images

    def publish_actions(self, action):
        self.published.append(action)

    def feed_observation(self, observation: Dict[str, Any]) -> None:
        assert self.observation_callback is not None
        self.observation_callback(observation)


class ScriptedSource(TeleopSource):
    """按脚本依次返回 TeleopTarget；脚本耗尽返回 None。"""

    def __init__(self, targets):
        self._targets = list(targets)

    @property
    def name(self) -> str:
        return "scripted"

    def start(self) -> None:
        pass

    def read(self) -> Optional[TeleopTarget]:
        if self._targets:
            return self._targets.pop(0)
        return None

    def stop(self) -> None:
        pass


class FakeClock:
    """可控单调时钟。"""

    def __init__(self) -> None:
        self.now = 100.0

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


def _joint_config(**overrides) -> OperatorConfig:
    cfg = {
        "source": {"type": "mock_joints"},
        "action": {
            "mode": "joint",
            "action_hz": 30.0,
            "joint_map": {"a": "joint_1", "b": "joint_2"},
        },
        "safety": {"require_observation": True, "observation_timeout_s": 0.0},
    }
    cfg.update(overrides)
    return OperatorConfig.from_cfg(cfg)


def _target(joints: Dict[str, float], **kwargs) -> TeleopTarget:
    return TeleopTarget(source_timestamp_ms=1, joint_positions=dict(joints), **kwargs)


# ── joint 模式 ───────────────────────────────────────────────────────────


def test_joint_mode_publishes_mapped_action():
    session = FakeSession()
    clock = FakeClock()
    operator = TeleopOperator(
        session,
        _joint_config(),
        source=ScriptedSource([_target({"a": 1.0, "b": 2.0})]),
        clock=clock,
    )
    operator.start()
    session.feed_observation({"timestamp": 1, "joint_states": {"names": ["joint_1"], "position": [0.1]}})

    action = operator.tick()
    assert action is not None
    assert action.chunk_size == 1
    assert action.id == 0
    assert action.joint_states.names == ["joint_1", "joint_2"]
    assert action.joint_states.position == [[1.0, 2.0]]
    assert len(session.published) == 1
    assert session.decode_images is False


def test_joint_mode_ids_increment():
    session = FakeSession()
    operator = TeleopOperator(
        session,
        _joint_config(),
        source=ScriptedSource([_target({"a": 1.0, "b": 2.0}) for _ in range(3)]),
        clock=FakeClock(),
    )
    operator.start()
    session.feed_observation({"timestamp": 1})
    for expected_id in (0, 1, 2):
        action = operator.tick()
        assert action is not None and action.id == expected_id


def test_joint_mode_joint_scale_applied():
    session = FakeSession()
    config = _joint_config()
    config.joint_scale = {"a": 10.0}
    operator = TeleopOperator(
        session,
        config,
        source=ScriptedSource([_target({"a": 1.5, "b": -2.0})]),
        clock=FakeClock(),
    )
    operator.start()
    session.feed_observation({"timestamp": 1})
    action = operator.tick()
    assert action is not None
    assert action.joint_states.position == [[15.0, -2.0]]


def test_joint_mode_missing_joint_skips_tick():
    session = FakeSession()
    operator = TeleopOperator(
        session,
        _joint_config(),
        source=ScriptedSource([_target({"a": 1.0})]),  # 缺 b
        clock=FakeClock(),
    )
    operator.start()
    session.feed_observation({"timestamp": 1})
    assert operator.tick() is None
    assert session.published == []


def test_joint_mode_gripper_appended():
    session = FakeSession()
    config = _joint_config()
    config.gripper_joint = "gripper"
    operator = TeleopOperator(
        session,
        config,
        source=ScriptedSource([_target({"a": 0.0, "b": 0.0}, gripper=0.5)]),
        clock=FakeClock(),
    )
    operator.start()
    session.feed_observation({"timestamp": 1})
    action = operator.tick()
    assert action is not None
    assert action.joint_states.names == ["joint_1", "joint_2", "gripper"]
    assert action.joint_states.position == [[0.0, 0.0, 0.5]]


# ── 观测安全策略 ─────────────────────────────────────────────────────────


def test_no_observation_before_first_frame_skips():
    session = FakeSession()
    operator = TeleopOperator(
        session,
        _joint_config(),
        source=ScriptedSource([_target({"a": 1.0, "b": 2.0})]),
        clock=FakeClock(),
    )
    operator.start()
    assert operator.tick() is None
    assert session.published == []


def test_require_observation_false_allows_early_publish():
    session = FakeSession()
    config = _joint_config()
    config.require_observation = False
    operator = TeleopOperator(
        session,
        config,
        source=ScriptedSource([_target({"a": 1.0, "b": 2.0})]),
        clock=FakeClock(),
    )
    operator.start()
    action = operator.tick()
    assert action is not None
    assert len(session.published) == 1


def test_observation_timeout_stops_publishing():
    session = FakeSession()
    clock = FakeClock()
    config = _joint_config()
    config.observation_timeout_s = 1.0
    operator = TeleopOperator(
        session,
        config,
        source=ScriptedSource([_target({"a": 1.0, "b": 2.0}) for _ in range(3)]),
        clock=clock,
    )
    operator.start()
    session.feed_observation({"timestamp": 1})
    assert operator.tick() is not None
    clock.advance(2.0)  # 超过 observation_timeout_s
    assert operator.tick() is None
    assert operator.tick() is None
    assert len(session.published) == 1


def test_stale_source_skips():
    session = FakeSession()
    stale_target = _target({"a": 1.0, "b": 2.0}, stale=True)
    operator = TeleopOperator(
        session,
        _joint_config(),
        source=ScriptedSource([stale_target, None]),
        clock=FakeClock(),
    )
    operator.start()
    session.feed_observation({"timestamp": 1})
    assert operator.tick() is None
    assert operator.tick() is None
    assert session.published == []


# ── end_effector_pose 模式 ───────────────────────────────────────────────


def _ee_config() -> OperatorConfig:
    return OperatorConfig.from_cfg(
        {
            "source": {"type": "mock_joints"},
            "action": {
                "mode": "end_effector_pose",
                "action_hz": 30.0,
                "ee_pose_map": {"left": "left_ee", "right": "right_ee"},
                "gripper": {"emit": "joint", "joint_name": "gripper", "range": [0.0, 100.0]},
            },
            "safety": {"require_observation": True, "observation_timeout_s": 0.0},
        }
    )


def test_ee_mode_publishes_poses_and_gripper():
    session = FakeSession()
    target = TeleopTarget(
        source_timestamp_ms=1,
        end_effector_poses={
            "left": [0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0],
            "right": [0.4, 0.5, 0.6, 0.0, 0.0, 0.0, 1.0],
        },
        gripper=0.5,
    )
    operator = TeleopOperator(session, _ee_config(), source=ScriptedSource([target]), clock=FakeClock())
    operator.start()
    session.feed_observation({"timestamp": 1})
    action = operator.tick()
    assert action is not None
    assert set(action.end_effector_poses) == {"left_ee", "right_ee"}
    assert list(action.end_effector_poses["left_ee"].pose[0].data) == pytest.approx(
        [0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0]
    )
    # 夹爪 0.5 → [0, 100] 区间中点
    assert action.joint_states.names == ["gripper"]
    assert action.joint_states.position == [[50.0]]


def test_ee_mode_partial_hand_still_publishes():
    session = FakeSession()
    target = TeleopTarget(
        source_timestamp_ms=1,
        end_effector_poses={"left": [0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0]},  # 缺 right
    )
    operator = TeleopOperator(session, _ee_config(), source=ScriptedSource([target]), clock=FakeClock())
    operator.start()
    session.feed_observation({"timestamp": 1})
    action = operator.tick()
    assert action is not None
    assert set(action.end_effector_poses) == {"left_ee"}


def test_ee_mode_no_poses_skips():
    session = FakeSession()
    operator = TeleopOperator(
        session,
        _ee_config(),
        source=ScriptedSource([TeleopTarget(source_timestamp_ms=1)]),
        clock=FakeClock(),
    )
    operator.start()
    session.feed_observation({"timestamp": 1})
    assert operator.tick() is None


# ── 配置校验 ─────────────────────────────────────────────────────────────


def test_joint_mode_requires_joint_map():
    with pytest.raises(ValueError, match="joint_map is required"):
        OperatorConfig.from_cfg(
            {"source": {"type": "mock_joints"}, "action": {"mode": "joint"}}
        )


def test_ee_mode_requires_ee_pose_map():
    with pytest.raises(ValueError, match="ee_pose_map is required"):
        OperatorConfig.from_cfg(
            {"source": {"type": "mock_joints"}, "action": {"mode": "end_effector_pose"}}
        )


def test_invalid_mode_rejected():
    with pytest.raises(ValueError, match="mode"):
        OperatorConfig.from_cfg(
            {"source": {"type": "mock_joints"}, "action": {"mode": "telepathy"}}
        )


def test_invalid_action_hz_rejected():
    with pytest.raises(ValueError, match="action_hz"):
        OperatorConfig.from_cfg(
            {
                "source": {"type": "mock_joints"},
                "action": {"mode": "joint", "action_hz": 0, "joint_map": {"a": "j1"}},
            }
        )


# ── 录制配对 ─────────────────────────────────────────────────────────────


def test_recorder_pairs_observation_with_action(tmp_path):
    session = FakeSession()
    recorder = TeleopSessionRecorder(str(tmp_path / "session.r2cr"), source_name="scripted")
    operator = TeleopOperator(
        session,
        _joint_config(),
        source=ScriptedSource([_target({"a": 1.0, "b": 2.0}) for _ in range(2)]),
        recorder=recorder,
        clock=FakeClock(),
    )
    operator.start()
    observation = {
        "timestamp": 42,
        "joint_states": {"names": ["joint_1"], "position": [0.25]},
        "images": {"color": {"front": b"\xff\xd8jpeg"}},
    }
    session.feed_observation(observation)
    operator.tick()
    session.feed_observation({"timestamp": 43})
    operator.tick()
    assert recorder.frame_count == 2
    recorder.close()

    from cloudrobo_r2c.common.utils.safe_serialization import load_frames

    frames = load_frames(str(tmp_path / "session.r2cr"))
    assert len(frames) == 2
    assert frames[0]["observation"]["joint_states"]["position"] == [0.25]
    assert frames[0]["observation"]["images"]["color"]["front"] == b"\xff\xd8jpeg"
    assert frames[0]["action"]["joint_states"]["names"] == ["joint_1", "joint_2"]
    assert frames[0]["source_name"] == "scripted"
    assert frames[1]["observation"]["timestamp"] == 43
