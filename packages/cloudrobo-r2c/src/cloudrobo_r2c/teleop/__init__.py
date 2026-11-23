"""CloudRobo R2C 遥操（Teleop）操作端。

组成：

- :class:`TeleopOperator`：读输入源 → 映射为 Actions → 发布到机器人，
  订阅机器人观测做安全看门狗。
- :class:`TeleopSource` / ``SOURCE_REGISTRY``：输入源抽象与注册表
  （内置 ``lerobot_leader`` 主从臂、``mock_joints`` 联调）。

入口：``cloudrobo r2c teleop --teleop-config <yaml> ...``，详见
``docs/teleop.md``。（数据采集/录制功能已于 2026-11-06 裁撤）
"""

from . import sources as _sources  # noqa: F401  (导入触发内置输入源注册)
from .base import (
    SOURCE_REGISTRY,
    TeleopSource,
    TeleopTarget,
    create_source,
    register_source,
)
from .operator import OperatorConfig, TeleopOperator

__all__ = [
    "SOURCE_REGISTRY",
    "TeleopSource",
    "TeleopTarget",
    "TeleopOperator",
    "OperatorConfig",
    "create_source",
    "register_source",
]
