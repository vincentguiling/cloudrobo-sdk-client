"""Operator mapping tests: relative pose (clutch/reach/EMA) + passthrough."""

import math

import pytest

from cloudrobo_r2c.teleop.mapping import create_mapping
from cloudrobo_r2c.teleop.mapping.passthrough import PassthroughMapping
from cloudrobo_r2c.teleop.pose_math import Pose, compose, inverse


def test_relative_pose_initial_output_equals_base():
    m = create_mapping("relative_pose", reach_limit_m=0.5, smoothing_alpha=1.0)
    base = Pose([0.5, 0.0, 0.3], [0, 0, 0, 1])
    ctrl = Pose([0.1, 0.1, 0.1], [0, 0, 0, 1])
    m.begin(base, ctrl)
    out = m.map(True, ctrl)
    assert out.p == pytest.approx(base.p, abs=1e-6)
    assert out.q == pytest.approx(base.q, abs=1e-6)


def test_relative_pose_follows_motion_delta():
    m = create_mapping("relative_pose", reach_limit_m=1.0, smoothing_alpha=1.0)
    base = Pose([0.5, 0.0, 0.3], [0, 0, 0, 1])
    ctrl0 = Pose([0.1, 0.0, 0.0], [0, 0, 0, 1])
    m.begin(base, ctrl0)
    first = m.map(True, ctrl0)  # press frame anchors on the base
    assert first.p == pytest.approx(base.p, abs=1e-6)
    ctrl1 = Pose([0.2, 0.0, 0.0], [0, 0, 0, 1])  # moved +0.1 in x
    out = m.map(True, ctrl1)
    assert out.p[0] == pytest.approx(0.6, abs=1e-6)


def test_relative_pose_hold_without_grip():
    m = create_mapping("relative_pose", reach_limit_m=1.0, smoothing_alpha=1.0)
    base = Pose([0.5, 0.0, 0.3], [0, 0, 0, 1])
    ctrl = Pose([0.1, 0.0, 0.0], [0, 0, 0, 1])
    m.begin(base, ctrl)
    moved = Pose([0.2, 0.0, 0.0], [0, 0, 0, 1])
    first = m.map(True, moved)
    held = m.map(False, moved)
    assert held.p == pytest.approx(first.p, abs=1e-6)
    # clutch re-press anchors on the last output (no jump)
    again = m.map(True, moved)
    assert again.p == pytest.approx(first.p, abs=1e-6)


def test_relative_pose_reach_limit_clamps():
    m = create_mapping("relative_pose", reach_limit_m=0.05, smoothing_alpha=1.0)
    base = Pose([0.0, 0.0, 0.0], [0, 0, 0, 1])
    ctrl0 = Pose([0.0, 0.0, 0.0], [0, 0, 0, 1])
    m.begin(base, ctrl0)
    far = Pose([1.0, 0.0, 0.0], [0, 0, 0, 1])
    out = m.map(True, far)
    assert abs(out.p[0]) <= 0.05 + 1e-6


def test_relative_pose_ema_smoothing():
    m = create_mapping("relative_pose", reach_limit_m=1.0, smoothing_alpha=0.5)
    base = Pose([0.0, 0.0, 0.0], [0, 0, 0, 1])
    ctrl0 = Pose([0.0, 0.0, 0.0], [0, 0, 0, 1])
    m.begin(base, ctrl0)
    m.map(True, ctrl0)  # press frame
    step = Pose([0.1, 0.0, 0.0], [0, 0, 0, 1])
    out = m.map(True, step)
    assert out.p[0] == pytest.approx(0.05, abs=1e-6)  # half of 0.1 target


def test_passthrough_returns_input_pose():
    m = PassthroughMapping()
    base = Pose([0.5, 0.0, 0.3], [0, 0, 0, 1])
    ctrl = Pose([0.52, 0.01, 0.3], [0, 0, 0, 1])
    m.begin(base, ctrl)
    out = m.map(True, ctrl)
    assert out.p == pytest.approx(ctrl.p, abs=1e-6)


def test_mapping_registry_rejects_unknown():
    with pytest.raises(ValueError):
        create_mapping("no-such-mapping")


def test_mapping_registry_lists_builtins():
    from cloudrobo_r2c.teleop.mapping import available_mappings

    names = available_mappings()
    assert "relative_pose" in names
    assert "passthrough" in names