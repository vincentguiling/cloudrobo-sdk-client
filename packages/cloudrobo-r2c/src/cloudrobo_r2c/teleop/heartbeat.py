"""Operator heartbeat (port of core/HeartbeatPublisher.cs).

Publishes {project}/{device}/state/heartbeats at 5 s ± 0.5 s with
BEST_EFFORT + Background priority (publisher declared by transport).
``source`` = "{project}/{device}", ``status`` = "ONLINE", ``mode`` follows the
teleop takeover state: AUTO when idle, CONTROLLING while RUNNING/PAUSED.

The loop is a daemon thread so it keeps running independently of the event
loop; publish failures are throttled and never kill the loop.
"""

from __future__ import annotations

import random
import threading
import time
from typing import Any, Callable, Optional


class HeartbeatPublisher:
    def __init__(
        self,
        publisher: Any,
        source: str,
        mode_getter: Callable[[], str],
        *,
        period_s: float = 5.0,
        jitter_s: float = 0.5,
    ) -> None:
        self.publisher = publisher
        self.source = source
        self.mode_getter = mode_getter
        self.period_s = period_s
        self.jitter_s = jitter_s
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._last_error_log = 0.0

    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._loop, name="teleop-heartbeat", daemon=True
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                self._publish()
            except Exception as exc:  # noqa: BLE001 - keep the loop alive
                now = time.monotonic()
                if now - self._last_error_log > 5.0:
                    self._last_error_log = now
                    print(f"[heartbeat] publish failed: {exc}", flush=True)
            delay = self.period_s + random.uniform(-self.jitter_s, self.jitter_s)
            self._stop.wait(max(0.5, delay))

    def _publish(self) -> None:
        from ..common.models.generated import heartbeat_pb2

        hb = heartbeat_pb2.Heartbeat()
        hb.timestamp.GetCurrentTime()
        hb.source = self.source
        hb.status = "ONLINE"
        hb.mode = "CONTROLLING" if self.mode_getter() == "CONTROLLING" else "AUTO"
        self.publisher.put(hb.SerializeToString())
