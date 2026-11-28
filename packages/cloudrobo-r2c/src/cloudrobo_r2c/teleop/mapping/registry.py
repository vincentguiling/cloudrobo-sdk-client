"""Mapping strategy registry.

Built-ins:
  * ``relative_pose`` — VR / 3D-mouse style incremental control (default);
  * ``isomorphic``    — joint-space pass-through for master-arm input
                        (reserved; implemented when the isomorphic source lands).

Third parties register strategies with the same entry point group used by
the r2c SDK conventions:

    [project.entry-points."r2c_sdk.teleop_mappings"]
    my_mapping = "my_pkg.mapping:MyMapping"

Strategies receive ``(reach_limit_m, smoothing_alpha)`` if their constructor
accepts them; otherwise a no-arg constructor is used.
"""

from __future__ import annotations

import importlib.metadata
from typing import Any, Callable, Optional

ENTRY_POINT_GROUP = "r2c_sdk.teleop_mappings"

_REGISTRY: dict[str, Callable[..., Any]] = {}


def register_mapping(name: str, factory: Callable[..., Any]) -> None:
    _REGISTRY[name] = factory


def _load_entry_point(name: str) -> Optional[Callable[..., Any]]:
    try:
        eps = importlib.metadata.entry_points()
        group = eps.select(group=ENTRY_POINT_GROUP) if hasattr(eps, "select") else eps.get(ENTRY_POINT_GROUP, [])
        for ep in group:
            if ep.name == name:
                return ep.load()
    except Exception:  # noqa: BLE001 - entry points are best-effort
        return None
    return None


def available_mappings() -> list[str]:
    names = set(_REGISTRY)
    try:
        eps = importlib.metadata.entry_points()
        group = eps.select(group=ENTRY_POINT_GROUP) if hasattr(eps, "select") else eps.get(ENTRY_POINT_GROUP, [])
        names.update(ep.name for ep in group)
    except Exception:  # noqa: BLE001
        pass
    return sorted(names)


def create_mapping(
    name: str,
    *,
    reach_limit_m: float = 0.5,
    smoothing_alpha: float = 1.0,
) -> Any:
    from .relative_pose import RelativePoseMapping

    if name in ("", "relative_pose"):
        return RelativePoseMapping(reach_limit_m, smoothing_alpha)
    factory: Optional[Callable[..., Any]] = _REGISTRY.get(name) or _load_entry_point(name)
    if factory is None and ":" in name:
        module_name, _, attr = name.partition(":")
        try:
            module = __import__(module_name, fromlist=[attr])
            factory = getattr(module, attr)
        except Exception as exc:  # noqa: BLE001
            raise ValueError(f"cannot import mapping strategy {name!r}: {exc}") from exc
    if factory is None:
        raise ValueError(
            f"unknown mapping strategy {name!r}; available: {available_mappings()}"
        )
    try:
        return factory(reach_limit_m, smoothing_alpha)
    except TypeError:
        return factory()


def _register_builtins() -> None:
    from .passthrough import PassthroughMapping
    from .relative_pose import RelativePoseMapping

    _REGISTRY.setdefault("relative_pose", RelativePoseMapping)
    _REGISTRY.setdefault("passthrough", PassthroughMapping)


_register_builtins()
