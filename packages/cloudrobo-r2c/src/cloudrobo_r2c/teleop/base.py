"""遥操（Teleop）核心抽象：输入源、目标值与源注册表。

遥操链路（operator 侧）::

    TeleopSource（主从臂 leader / VR / mock）
        └─ read() → TeleopTarget（本 tick 的关节/末端/夹爪目标）
    TeleopOperator
        └─ 目标值 → Actions → session.publish_actions()（发给机器人）
        └─ 最新 Observations + 已发布 Actions → TeleopSessionRecorder（数据采集）

输入源只负责"读设备"，不做任何 R2C 语义转换；把源输出映射为
Actions（关节名/末端名/夹爪）统一在 ``operator``/``mapping`` 中完成。
"""

from __future__ import annotations

import logging
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, Mapping, Optional, Sequence

logger = logging.getLogger(__name__)


@dataclass
class TeleopTarget:
    """一个遥操 tick 的输入目标值（设备坐标系/单位，未映射）。

    - ``joint_positions``：按源设备关节键名索引的目标值（如 leader 臂的
      ``shoulder_pan`` → 弧度/度，保持设备原始单位，映射交给 operator）。
    - ``end_effector_poses``：源键（如 VR 的 ``left``/``right``）→
      ``[tx, ty, tz, qx, qy, qz, qw]``（Pose7D 顺序，已转到目标工作坐标系）。
    - ``gripper``：归一化夹爪开度 0.0（闭合）~ 1.0（张开）；无夹爪数据为 None。
    - ``stale``：输入数据超时（如 VR 数据流中断）。operator 对 stale 的
      处理是停发动作，而不是发布旧值。
    """

    source_timestamp_ms: int = 0
    joint_positions: Dict[str, float] = field(default_factory=dict)
    end_effector_poses: Dict[str, Sequence[float]] = field(default_factory=dict)
    gripper: Optional[float] = None
    stale: bool = False


class TeleopSource(ABC):
    """遥操输入源接口（主从臂 leader / VR 设备 / mock 等）。

    生命周期：``start()`` → 多次 ``read()`` → ``stop()``。``read()`` 必须
    非阻塞（设备读取在源内部线程完成），无新数据时返回上一次目标或 None。
    """

    @property
    @abstractmethod
    def name(self) -> str:
        """源类型名（与注册表一致，如 ``vr_udp``）。"""

    @abstractmethod
    def start(self) -> None:
        """启动源（打开设备/监听端口）。"""

    @abstractmethod
    def read(self) -> Optional[TeleopTarget]:
        """读取当前目标值；输入不可用时返回 None 或 stale=True 的目标。"""

    @abstractmethod
    def stop(self) -> None:
        """停止源并释放设备资源。"""

    def describe(self) -> Dict[str, Any]:
        """人类可读的源描述（启动日志用）。"""
        return {"type": self.name}


SourceFactory = Callable[[Mapping[str, Any]], TeleopSource]

SOURCE_REGISTRY: Dict[str, SourceFactory] = {}


def register_source(source_type: str) -> Callable[[SourceFactory], SourceFactory]:
    """把输入源工厂注册进 ``SOURCE_REGISTRY``（源类型名 → 工厂）。"""

    def _decorator(factory: SourceFactory) -> SourceFactory:
        if source_type in SOURCE_REGISTRY:
            raise ValueError(f"teleop source type already registered: {source_type!r}")
        SOURCE_REGISTRY[source_type] = factory
        logger.debug("registered teleop source: %s", source_type)
        return factory

    return _decorator


def create_source(source_cfg: Mapping[str, Any]) -> TeleopSource:
    """按 ``teleop.source`` 配置节创建输入源。

    配置形态::

        source:
          type: vr_udp          # 注册表中的源类型
          config: {...}         # 源自己的配置（缺省 {}）
    """
    if not isinstance(source_cfg, Mapping):
        raise ValueError(f"teleop.source must be a mapping, got {type(source_cfg).__name__}")
    source_type = str(source_cfg.get("type", "")).strip()
    if not source_type:
        raise ValueError(
            "teleop.source.type is required "
            f"(available sources: {sorted(SOURCE_REGISTRY) or '(none loaded)'})"
        )
    factory = SOURCE_REGISTRY.get(source_type)
    if factory is None:
        raise ValueError(
            f"unknown teleop source type: {source_type!r}. "
            f"Available sources: {sorted(SOURCE_REGISTRY)}"
        )
    return factory(source_cfg.get("config") or {})
