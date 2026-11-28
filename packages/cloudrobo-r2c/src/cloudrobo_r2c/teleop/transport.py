"""Zenoh transport for the operator gateway (port of core/ZenohTransport.cs).

QoS split (same as the C# gateway and the r2c SDK conventions):
  * teleop actions  -> BEST_EFFORT + DataHigh   (freshest frame wins)
  * teleop commands -> RELIABLE + InteractiveHigh (delivery matters)
  * heartbeats      -> BEST_EFFORT + Background (5 s ± 0.5 s)

TLS/credential-bundle values come from TeleopConfig.zenoh (see config.py).
"""

from __future__ import annotations

import json
from typing import Any, Callable, Optional

from .config import TeleopConfig


class ZenohTransport:
    def __init__(self, cfg: TeleopConfig) -> None:
        self.cfg = cfg
        self._session: Any = None
        self._subs: list[Any] = []
        self._publishers: dict[str, Any] = {}

    # ── lifecycle ────────────────────────────────────────────────────────────
    def open(self) -> None:
        import zenoh

        zcfg = zenoh.Config()
        zcfg.insert_json5("mode", json.dumps(self.cfg.zenoh.mode))
        if self.cfg.zenoh.mode == "client":
            zcfg.insert_json5(
                "connect/endpoints", json.dumps(self.cfg.zenoh.endpoints)
            )
        elif self.cfg.zenoh.endpoint_role == "listen":
            zcfg.insert_json5(
                "listen/endpoints", json.dumps(self.cfg.zenoh.endpoints)
            )
        else:
            zcfg.insert_json5(
                "connect/endpoints", json.dumps(self.cfg.zenoh.endpoints)
            )
        tls = self.cfg.zenoh.tls
        if tls.enabled:
            zcfg.insert_json5("transport/link/tls/enabled", "true")
            if tls.root_ca:
                zcfg.insert_json5(
                    "transport/link/tls/root_ca_certificate",
                    json.dumps(tls.root_ca),
                )
            if tls.private_key:
                zcfg.insert_json5(
                    "transport/link/tls/connect_private_key",
                    json.dumps(tls.private_key),
                )
            if tls.public_cert:
                zcfg.insert_json5(
                    "transport/link/tls/connect_certificate",
                    json.dumps(tls.public_cert),
                )
        self._session = zenoh.open(zcfg)

    def close(self) -> None:
        for sub in self._subs:
            try:
                sub.undeclare()
            except Exception:  # noqa: BLE001 - best-effort cleanup
                pass
        self._subs.clear()
        for pub in self._publishers.values():
            try:
                pub.undeclare()
            except Exception:  # noqa: BLE001
                pass
        self._publishers.clear()
        if self._session is not None:
            try:
                self._session.close()
            finally:
                self._session = None

    @property
    def session(self) -> Any:
        if self._session is None:
            raise RuntimeError("ZenohTransport.open() has not been called")
        return self._session

    # ── publishers / subscribers ─────────────────────────────────────────────
    def declare_action_publisher(self, topic: str) -> Any:
        import zenoh

        return self._declare_publisher(
            topic,
            reliability=zenoh.Reliability.BEST_EFFORT,
            priority=zenoh.Priority.DATA_HIGH,
        )

    def declare_command_publisher(self, topic: str) -> Any:
        import zenoh

        return self._declare_publisher(
            topic,
            reliability=zenoh.Reliability.RELIABLE,
            priority=zenoh.Priority.INTERACTIVE_HIGH,
        )

    def declare_heartbeat_publisher(self, topic: str) -> Any:
        import zenoh

        return self._declare_publisher(
            topic,
            reliability=zenoh.Reliability.BEST_EFFORT,
            priority=zenoh.Priority.BACKGROUND,
        )

    def _declare_publisher(self, topic: str, *, reliability: Any, priority: Any) -> Any:
        key = f"{topic}|{reliability}|{priority}"
        if key not in self._publishers:
            self._publishers[key] = self.session.declare_publisher(
                topic, reliability=reliability, priority=priority
            )
        return self._publishers[key]

    def subscribe(self, topic: str, handler: Callable[[Any], None]) -> Any:
        sub = self.session.declare_subscriber(topic, handler)
        self._subs.append(sub)
        return sub

    def put(self, publisher: Any, payload: bytes) -> Optional[Any]:
        try:
            return publisher.put(payload)
        except Exception as exc:  # noqa: BLE001 - publisher failure must not kill loops
            print(f"[transport] publish failed on {publisher}: {exc}", flush=True)
            return None
