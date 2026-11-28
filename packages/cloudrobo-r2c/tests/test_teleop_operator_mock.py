"""Mock teleop device tests: state/current-based small offsets."""

import asyncio

import pytest

from cloudrobo_r2c.teleop.inputs import make_input_source
from cloudrobo_r2c.teleop.pose_math import Pose


def _pose(x: float = 0.5, y: float = 0.0) -> Pose:
    return Pose([x, y, 0.3], [0.0, 0.0, 0.0, 1.0])


def test_mock_builds_samples_around_current_pose():
    source = make_input_source(
        "mock",
        {"amplitude_m": 0.02, "rate_hz": 0.3, "interval_s": 0.01},
        state_getter=lambda hand: _pose(0.5, 0.0),
    )
    samples = source._build_samples()
    assert {s.side for s in samples} == {"left", "right"}
    for s in samples:
        assert s.grip is True
        assert s.trigger == pytest.approx(0.5)
        # offset stays within the wobble radius of the current pose
        dx = abs(s.pose[0] - 0.5)
        dy = abs(s.pose[1] - 0.0)
        assert dx <= 0.0201
        assert dy <= 0.0201


def test_mock_missing_state_pose_yields_no_samples():
    source = make_input_source("mock", {}, state_getter=lambda hand: None)
    assert source._build_samples() == []


def test_mock_capabilities_and_auto_session():
    source = make_input_source("mock", {}, state_getter=lambda hand: _pose())
    caps = source.capabilities()
    assert caps["deadman"] == "grip"
    assert caps["synthetic"] is True
    assert source.wants_auto_session is True


def test_mock_streams_via_sink():
    collected: list = []

    async def scenario() -> None:
        source = make_input_source(
            "mock",
            {"interval_s": 0.01, "amplitude_m": 0.01},
            state_getter=lambda hand: _pose(),
        )

        async def sink(samples):
            collected.append(samples)

        await source.start(sink)
        await asyncio.sleep(0.05)
        await source.stop()

    asyncio.run(scenario())
    assert collected, "mock source should push samples"
    assert all(len(batch) == 2 for batch in collected)