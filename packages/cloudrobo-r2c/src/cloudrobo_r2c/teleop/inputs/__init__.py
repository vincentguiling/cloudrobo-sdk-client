"""Input source abstraction + registry (design §3.3.5.4).

An InputSource normalizes a physical device into ``InputSample`` values; the
gateway owns session state, action pacing, heartbeat and publishing. Sources
never touch Zenoh or the TeleopFrame protocol.

Built-in sources:
  * ``webxr`` — Quest browser HUD over WebSocket (the HUD page is served by
    the gateway; the same JSON protocol as the previous C# gateway);
  * ``mock``  — synthetic operator: offsets the robot's latest
    state/current end-effector pose a little (up/down/left/right) and
    publishes it back as teleop actions; useful without a headset and for
    end-to-end demos/validation.

Third parties register sources via the ``r2c_sdk.teleop_inputs`` entry point
group, or configure a ``module:Class`` path directly.
"""

from __future__ import annotations

import importlib.metadata
from dataclasses import dataclass, field
from typing import Any, Awaitable, Callable, Mapping, Optional, Protocol

# INPUT_MODE values (teleop.proto InputMode enum)
INPUT_MODE_VR = 1
INPUT_MODE_ISOMORPHIC = 2

ENTRY_POINT_GROUP = "r2c_sdk.teleop_inputs"


@dataclass
class InputSample:
    """One normalized device reading."""

    input_mode: int = INPUT_MODE_VR
    side: Optional[str] = None          # "left" / "right" / None (single device)
    pose: Optional[list[float]] = None  # 7D [tx,ty,tz,qx,qy,qz,qw]
    joints: Optional[list[float]] = None
    grip: bool = False                  # deadman
    trigger: float = 0.0
    buttons: dict[str, bool] = field(default_factory=dict)
    axes: dict[str, float] = field(default_factory=dict)
    raw: dict[str, Any] = field(default_factory=dict)


class InputSource(Protocol):
    name: str
    input_mode: int
    ws_path: Optional[str]          # "/ws" when the source serves the HUD page
    wants_auto_session: bool

    def capabilities(self) -> dict: ...

    async def start(self, sink: Callable[[list[InputSample]], Awaitable[None]]) -> None: ...

    async def stop(self) -> None: ...


def make_input_source(
    name: str,
    params: Optional[Mapping[str, Any]] = None,
    *,
    state_getter: Optional[Callable[[str], Optional[Any]]] = None,
) -> Any:
    """Create a built-in or third-party input source.

    ``name`` may be a built-in (webxr/mock), an entry point name, or a
    ``module:Class`` path. ``state_getter(hand) -> Pose|None`` lets sources
    read the latest state/current pose without owning the subscription.
    """
    params = dict(params or {})
    builtin = {"webxr": "webxr_ws:WebXrWsSource", "mock": "mock:MockSource"}
    target = builtin.get(name)
    if target is not None:
        module_name, _, attr = target.partition(":")
        module = __import__(f"{__name__}.{module_name}", fromlist=[attr])
        cls = getattr(module, attr)
        return cls(params, state_getter=state_getter)
    # entry point group
    factory = _load_entry_point(name)
    if factory is not None:
        try:
            return factory(params, state_getter=state_getter)
        except TypeError:
            return factory(params)
    # module:Class fallback
    if ":" in name:
        module_name, _, attr = name.partition(":")
        module = __import__(module_name, fromlist=[attr])
        cls = getattr(module, attr)
        try:
            return cls(params, state_getter=state_getter)
        except TypeError:
            return cls(params)
    raise ValueError(
        f"unknown teleop input source {name!r}; available: webxr, mock "
        "(or an r2c_sdk.teleop_inputs entry point / module:Class path)"
    )


def available_input_sources() -> list[str]:
    names = {"webxr", "mock"}
    try:
        eps = importlib.metadata.entry_points()
        group = eps.select(group=ENTRY_POINT_GROUP) if hasattr(eps, "select") else eps.get(ENTRY_POINT_GROUP, [])
        names.update(ep.name for ep in group)
    except Exception:  # noqa: BLE001
        pass
    return sorted(names)


def _load_entry_point(name: str) -> Optional[Callable[..., Any]]:
    try:
        eps = importlib.metadata.entry_points()
        group = eps.select(group=ENTRY_POINT_GROUP) if hasattr(eps, "select") else eps.get(ENTRY_POINT_GROUP, [])
        for ep in group:
            if ep.name == name:
                return ep.load()
    except Exception:  # noqa: BLE001
        return None
    return None
