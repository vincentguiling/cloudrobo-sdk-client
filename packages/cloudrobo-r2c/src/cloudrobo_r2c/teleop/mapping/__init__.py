"""Mapping strategies: InputSample -> teleop action payload (Pose + trigger).

Strategies are pure functions over per-hand mapping state, so they can be
unit-tested without Zenoh or aiohttp. Third parties register their own
strategy via the ``r2c_sdk.teleop_mappings`` entry point group.
"""

from __future__ import annotations

from .registry import (  # noqa: F401
    available_mappings,
    create_mapping,
    register_mapping,
)