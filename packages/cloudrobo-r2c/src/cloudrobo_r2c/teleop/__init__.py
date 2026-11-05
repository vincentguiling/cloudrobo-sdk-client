"""CloudRobo R2C 遥操（Teleop）与数据采集操作端。

组成：

- :class:`TeleopOperator`：读输入源 → 映射为 Actions → 发布到机器人，
  并把（最新观测, 已发布动作）成对录制（遥操数据采集）。
- :class:`TeleopSource` / ``SOURCE_REGISTRY``：输入源抽象与注册表
  （内置 ``lerobot_leader`` 主从臂、``mock_joints`` 联调）。
- :class:`TeleopSessionRecorder`：R2CR 格式会话录制。

入口：``cloudrobo r2c teleop --teleop-config <yaml> ...``，详见
``docs/teleop.md``。
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
from .recording import TeleopSessionRecorder

__all__ = [
    "SOURCE_REGISTRY",
    "TeleopSource",
    "TeleopTarget",
    "TeleopOperator",
    "OperatorConfig",
    "TeleopSessionRecorder",
    "create_source",
    "register_source",
]
