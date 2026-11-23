"""lerobot_leader：LeRobot teleoperator 主臂输入源（主从臂遥操）。

把 LeRobot 主臂（如 ``so101_leader``/``koch_leader``）的 ``get_action()``
读数转为遥操目标：键 ``<joint>.pos`` → ``TeleopTarget.joint_positions[<joint>]``。
主从臂同构时关节值直接同单位透传（机器人侧 ``r2c_to_device`` 映射负责
落到对应关节）；夹爪可用 ``gripper_range`` 额外归一化输出。

lerobot 为可选依赖，导入与解码全部延迟到 ``start()``；解码方式与
``robots.lerobot_hardware_adapter`` 保持一致（draccus.decode + 内置
teleoperator 子模块预导入）。
"""

from __future__ import annotations

import logging
import time
from typing import Any, Callable, Dict, Mapping, Optional

from ..base import TeleopSource, TeleopTarget, register_source

logger = logging.getLogger(__name__)

TeleoperatorFactory = Callable[[Mapping[str, Any]], Any]


@register_source("lerobot_leader")
def create_lerobot_leader(config: Mapping[str, Any]) -> TeleopSource:
    return LeRobotLeaderSource(config)


class LeRobotLeaderSource(TeleopSource):
    """LeRobot 主臂输入源。

    配置（``teleop.source.config``）::

        type: so101_leader        # LeRobot TeleoperatorConfig 类型
        port: /dev/ttyACM1        # 主臂串口
        id: L12345678             # 可选；同型多臂区分
        calibration_dir: null     # 可选；标定目录
        connect_kwargs: {}        # 透传 teleoperator.connect()，如 calibrate: false
        gripper_key: gripper      # 夹爪关节基名（信息性，默认 gripper）
        gripper_range: [0, 100]   # 可选；[闭合值, 全开值]，设置后输出归一化开度
    """

    def __init__(
        self,
        config: Mapping[str, Any],
        teleoperator_factory: Optional[TeleoperatorFactory] = None,
    ) -> None:
        cfg = dict(config or {})
        self._teleop_type = str(cfg.get("type", "")).strip()
        if not self._teleop_type:
            raise ValueError("lerobot_leader.source.config.type is required (e.g. so101_leader)")
        self._device_cfg: Dict[str, Any] = {
            k: v for k, v in cfg.items() if k not in ("connect_kwargs", "gripper_key", "gripper_range")
        }
        self._connect_kwargs = dict(cfg.get("connect_kwargs") or {})
        self._gripper_key = str(cfg.get("gripper_key", "gripper"))
        gripper_range = cfg.get("gripper_range")
        if gripper_range is not None:
            values = [float(v) for v in gripper_range]
            if len(values) != 2 or values[1] == values[0]:
                raise ValueError("lerobot_leader.gripper_range must be [closed, open] with distinct values")
            self._gripper_range = (values[0], values[1])
        else:
            self._gripper_range = None
        self._teleoperator_factory = teleoperator_factory or self._default_teleoperator_factory
        self._teleoperator: Any = None
        self._joint_suffix = ".pos"
        self._last_error_log = 0.0

    @property
    def name(self) -> str:
        return "lerobot_leader"

    def start(self) -> None:
        if self._teleoperator is not None:
            return
        teleoperator = self._teleoperator_factory(self._device_cfg)
        connect = getattr(teleoperator, "connect", None)
        if not callable(connect):
            raise TypeError("LeRobot teleoperator instance must expose connect()")
        if self._connect_kwargs:
            try:
                connect(**self._connect_kwargs)
            except TypeError:
                logger.debug("teleoperator.connect(**kwargs) unsupported; retrying without kwargs")
                connect()
        else:
            connect()
        self._teleoperator = teleoperator
        logger.info("LeRobot leader teleoperator connected: %s", self.describe())

    def read(self) -> Optional[TeleopTarget]:
        teleoperator = self._teleoperator
        if teleoperator is None:
            return None
        try:
            action = teleoperator.get_action()
        except Exception as exc:
            now = time.monotonic()
            if now - self._last_error_log > 5.0:
                self._last_error_log = now
                logger.warning("LeRobot leader get_action failed: %s", exc)
            return None
        if not isinstance(action, Mapping) or not action:
            return None
        target = TeleopTarget(source_timestamp_ms=int(time.time() * 1000))
        for key, value in action.items():
            base = str(key)
            if base.endswith(self._joint_suffix):
                base = base[: -len(self._joint_suffix)]
            try:
                target.joint_positions[base] = float(value)
            except (TypeError, ValueError):
                logger.debug("lerobot_leader: non-numeric action value for %s: %r", key, value)
        if self._gripper_range is not None:
            raw = target.joint_positions.get(self._gripper_key)
            if raw is not None:
                closed, opened = self._gripper_range
                span = opened - closed
                normalized = (raw - closed) / span if span else 0.0
                target.gripper = 0.0 if normalized < 0.0 else (1.0 if normalized > 1.0 else normalized)
        return target

    def stop(self) -> None:
        teleoperator = self._teleoperator
        self._teleoperator = None
        if teleoperator is None:
            return
        disconnect = getattr(teleoperator, "disconnect", None)
        if callable(disconnect):
            try:
                disconnect()
            except Exception:
                logger.warning("LeRobot leader disconnect failed", exc_info=True)

    def describe(self) -> Dict[str, Any]:
        return {"type": self.name, "teleoperator": self._teleop_type}

    # ── lerobot 装配（与 robots.lerobot_hardware_adapter 同一模式） ──────

    @staticmethod
    def _default_teleoperator_factory(device_cfg: Mapping[str, Any]) -> Any:
        """按 lerobot 0.5.x API 构建 teleoperator 实例（延迟导入）。"""
        try:
            import draccus
            from lerobot.teleoperators import TeleoperatorConfig, make_teleoperator_from_config
        except ImportError as exc:  # pragma: no cover - optional runtime dependency
            raise RuntimeError(
                "LeRobot is not installed. Please `pip install lerobot` to use the "
                "lerobot_leader teleop source."
            ) from exc

        LeRobotLeaderSource._import_builtin_lerobot_teleoperators()

        try:
            from lerobot.utils.import_utils import register_third_party_plugins
        except ImportError:  # pragma: no cover - older lerobot without plugin discovery
            register_third_party_plugins = None  # type: ignore[assignment]
        if register_third_party_plugins is not None:
            register_third_party_plugins()

        if hasattr(draccus, "decode"):
            return make_teleoperator_from_config(
                draccus.decode(TeleoperatorConfig, dict(device_cfg))
            )
        raise RuntimeError(
            "draccus.decode is unavailable; cannot decode TeleoperatorConfig"
        )

    @staticmethod
    def _import_builtin_lerobot_teleoperators() -> None:
        """导入内置 teleoperator 子模块以触发 register_subclass 注册。

        与 ``lerobot_hardware_adapter._import_builtin_lerobot_robots`` 相同
        的机制：teleoperator 类型由子模块导入时注册，不导入则
        draccus.decode 报 "Couldn't find a choice class"。
        """
        import importlib
        import pkgutil

        try:
            import lerobot.teleoperators as lerobot_teleop_pkg
        except ImportError:  # pragma: no cover - optional runtime dependency
            return

        for module_info in pkgutil.iter_modules(lerobot_teleop_pkg.__path__):
            if not module_info.ispkg:
                continue
            try:
                importlib.import_module(f"lerobot.teleoperators.{module_info.name}")
            except ImportError:
                logger.debug(
                    "Skipped lerobot.teleoperators.%s (optional dependency missing).",
                    module_info.name,
                )
            except Exception:  # pragma: no cover - defensive
                logger.debug(
                    "Failed to import lerobot.teleoperators.%s; its type may be unavailable.",
                    module_info.name,
                    exc_info=True,
                )
