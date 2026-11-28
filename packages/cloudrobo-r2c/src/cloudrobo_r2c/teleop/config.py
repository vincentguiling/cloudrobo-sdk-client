"""Operator-side configuration for the teleop runtime.

All keys are optional; defaults reproduce the previous gateway behaviour
(project r2c/local, 20 Hz actions, BEST_EFFORT actions / RELIABLE commands /
Background heartbeats, relative pose control with reach limit + EMA).

Example (teleop-operator.yaml):

    zenoh:
      mode: client              # client | peer
      endpoint_role: connect    # connect | listen
      endpoints: ["tcp/127.0.0.1:7447"]
      tls: {enabled: false}
    teleop:
      project_id: r2c
      device_id: local
      operator_id: quest3
      actions_rate_hz: 20
      reach_limit_m: 0.5
      smoothing_alpha: 1.0
      input:
        source: webxr           # webxr | mock | <entry point name> | "module:Class"
        params: {}
      mapping: relative_pose    # relative_pose | isomorphic | <entry point name>
    state_current:
      source: end_effector_poses   # end_effector_poses | joint_states
      effector_names: [left, right]
      format: quat                 # quat | euler
      euler_order: xyz
      joint_name_suffixes: [left, right]
      values_per_arm: 7
      joint_pose_layout: trailing  # trailing | inline
    gateway:
      http_host: 0.0.0.0
      http_port: 8000
    session:
      auto_start: false          # mock source defaults to true

A credential bundle (--bundle zip|dir, same format as `r2c client`) overrides
device_id and the zenoh/tls sections.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping


@dataclass
class ZenohTlsConfig:
    enabled: bool = False
    root_ca: str = ""
    private_key: str = ""
    public_cert: str = ""

    @classmethod
    def from_mapping(cls, d: Mapping[str, Any]) -> "ZenohTlsConfig":
        if not isinstance(d, Mapping):
            return cls()
        return cls(
            enabled=bool(d.get("enabled", False)),
            root_ca=str(d.get("root_ca") or ""),
            private_key=str(d.get("private_key") or ""),
            public_cert=str(d.get("public_cert") or ""),
        )


@dataclass
class ZenohConfig:
    mode: str = "client"          # client | peer
    endpoint_role: str = "connect"  # connect | listen
    endpoints: list[str] = field(default_factory=lambda: ["tcp/127.0.0.1:7447"])
    tls: ZenohTlsConfig = field(default_factory=ZenohTlsConfig)

    @classmethod
    def from_mapping(cls, d: Mapping[str, Any]) -> "ZenohConfig":
        if not isinstance(d, Mapping):
            return cls()
        eps = d.get("endpoints")
        endpoints = [str(e) for e in eps] if isinstance(eps, (list, tuple)) and eps else ["tcp/127.0.0.1:7447"]
        return cls(
            mode=str(d.get("mode") or "client"),
            endpoint_role=str(d.get("endpoint_role") or "connect"),
            endpoints=endpoints,
            tls=ZenohTlsConfig.from_mapping(d.get("tls") or {}),
        )


@dataclass
class StateCurrentConfig:
    """Where the robot publishes its live end-effector pose (relative base)."""

    source: str = "end_effector_poses"   # end_effector_poses | joint_states
    effector_names: list[str] = field(default_factory=lambda: ["left", "right"])
    format: str = "quat"                 # quat (7 values) | euler (6 values, rad)
    euler_order: str = "xyz"
    joint_name_suffixes: list[str] = field(default_factory=lambda: ["left", "right"])
    values_per_arm: int = 7
    joint_pose_layout: str = "trailing"  # trailing | inline

    @classmethod
    def from_mapping(cls, d: Mapping[str, Any]) -> "StateCurrentConfig":
        if not isinstance(d, Mapping):
            return cls()
        names = d.get("effector_names")
        suffixes = d.get("joint_name_suffixes")
        return cls(
            source=str(d.get("source") or "end_effector_poses"),
            effector_names=[str(n) for n in names] if isinstance(names, (list, tuple)) and names else ["left", "right"],
            format=str(d.get("format") or "quat"),
            euler_order=str(d.get("euler_order") or "xyz"),
            joint_name_suffixes=[str(n) for n in suffixes] if isinstance(suffixes, (list, tuple)) and suffixes else ["left", "right"],
            values_per_arm=int(d.get("values_per_arm") or 7),
            joint_pose_layout=str(d.get("joint_pose_layout") or "trailing"),
        )


@dataclass
class TeleopConfig:
    project_id: str = "r2c"
    device_id: str = "local"
    operator_id: str = "python-gateway"
    actions_rate_hz: float = 20.0
    reach_limit_m: float = 0.5
    smoothing_alpha: float = 1.0
    grey_timeout_ms: float = 600.0
    input_source: str = "webxr"
    input_params: dict[str, Any] = field(default_factory=dict)
    mapping: str = ""  # empty = auto: passthrough for mock, relative_pose otherwise
    auto_start: bool = False
    zenoh: ZenohConfig = field(default_factory=ZenohConfig)
    state_current: StateCurrentConfig = field(default_factory=StateCurrentConfig)
    http_host: str = "0.0.0.0"
    http_port: int = 8000

    @property
    def actions_topic(self) -> str:
        return f"{self.project_id}/{self.device_id}/teleop/actions"

    @property
    def state_current_topic(self) -> str:
        return f"{self.project_id}/{self.device_id}/state/current"

    @property
    def heartbeats_topic(self) -> str:
        return f"{self.project_id}/{self.device_id}/state/heartbeats"

    @classmethod
    def from_mapping(cls, d: Mapping[str, Any]) -> "TeleopConfig":
        d = d or {}
        teleop = d.get("teleop") or {}
        if not isinstance(teleop, Mapping):
            teleop = {}
        inp = teleop.get("input") or {}
        if not isinstance(inp, Mapping):
            inp = {}
        session = d.get("session") or {}
        if not isinstance(session, Mapping):
            session = {}
        gateway = d.get("gateway") or {}
        if not isinstance(gateway, Mapping):
            gateway = {}
        return cls(
            project_id=str(teleop.get("project_id") or "r2c"),
            device_id=str(teleop.get("device_id") or "local"),
            operator_id=str(teleop.get("operator_id") or "python-gateway"),
            actions_rate_hz=float(teleop.get("actions_rate_hz") or 20.0),
            reach_limit_m=float(teleop.get("reach_limit_m") or 0.5),
            smoothing_alpha=float(teleop.get("smoothing_alpha") if teleop.get("smoothing_alpha") is not None else 1.0),
            grey_timeout_ms=float(teleop.get("grey_timeout_ms") or 600.0),
            input_source=str(inp.get("source") or "webxr"),
            input_params=dict(inp.get("params") or {}),
            mapping=str(teleop.get("mapping") or ""),
            auto_start=bool(session.get("auto_start", False)),
            zenoh=ZenohConfig.from_mapping(d.get("zenoh") or {}),
            state_current=StateCurrentConfig.from_mapping(d.get("state_current") or {}),
            http_host=str(gateway.get("http_host") or "0.0.0.0"),
            http_port=int(gateway.get("http_port") or 8000),
        )

    @classmethod
    def load(cls, path: str | Path) -> "TeleopConfig":
        import json

        import yaml

        p = Path(path)
        text = p.read_text(encoding="utf-8")
        if p.suffix.lower() in (".yaml", ".yml"):
            data = yaml.safe_load(text) or {}
        else:
            data = json.loads(text or "{}")
        if not isinstance(data, Mapping):
            raise ValueError(f"teleop config must be a mapping: {path}")
        return cls.from_mapping(data)


def apply_bundle(cfg: TeleopConfig, bundle_path: str | Path) -> TeleopConfig:
    """Overlay device_id / zenoh+tls from an r2c credential bundle.

    Reuses the robot-side bundle parser (same --bundle semantics as
    `cloudrobo r2c client`); keeps project_id and the local operator config
    unless the bundle provides authoritative values.
    """
    from ..common.credential_bundle import CredentialBundleLoader

    bundle, _ctx = CredentialBundleLoader.load(bundle_path)
    robot_id = getattr(bundle.identity, "robot_id", "") or ""
    if robot_id:
        cfg.device_id = robot_id
    mode = getattr(bundle.zenoh, "mode", "") or ""
    if mode:
        cfg.zenoh.mode = mode
    eps = getattr(bundle.zenoh, "connect_endpoints", None)
    if eps:
        cfg.zenoh.endpoints = [str(e) for e in eps]
    if getattr(bundle.zenoh, "enable_mtls", False):
        cfg.zenoh.tls.enabled = True
        cfg.zenoh.tls.root_ca = getattr(bundle.zenoh, "root_ca_certificate", "") or cfg.zenoh.tls.root_ca
        cfg.zenoh.tls.private_key = getattr(bundle.zenoh, "connect_private_key", "") or cfg.zenoh.tls.private_key
        cfg.zenoh.tls.public_cert = getattr(bundle.zenoh, "connect_certificate", "") or cfg.zenoh.tls.public_cert
    return cfg
