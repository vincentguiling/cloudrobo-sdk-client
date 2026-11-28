"""Teleop operator runtime (WebXR HUD + mock device), merged from cloudrobo-teleop.

Design reference: docs (Feishu "R2C 遥操特性设计" v3.0, 并入 r2c 仓库方案).
Package layout:

  teleop/config.py        operator config (zenoh/teleop/input/state_current)
  teleop/transport.py     Zenoh pub/sub with the same QoS split as the C# gateway
  teleop/state_current.py state/current subscription (relative-control base)
  teleop/frame_builder.py TeleopFrame construction (actions + command + extensions)
  teleop/heartbeat.py     5s±0.5s heartbeat (AUTO/CONTROLLING)
  teleop/commands.py      remote (cloud) command receiver
  teleop/session.py       operator session state machine (local + bundle modes)
  teleop/inputs/          InputSource abstraction + built-in sources (webxr/mock)
  teleop/mapping/         mapping strategies (relative_pose / isomorphic)
  teleop/gateway.py       aiohttp gateway (static HUD page + /ws + publishers)
  teleop/cli.py           `cloudrobo r2c teleop ...` command group

The robot side (sync_client teleop gating, translators.teleop_to_device,
teleop.proto) already lives in this package; this module only adds the
operator side.
"""

from __future__ import annotations

__all__ = ["__version__"]

__version__ = "0.1.0"
