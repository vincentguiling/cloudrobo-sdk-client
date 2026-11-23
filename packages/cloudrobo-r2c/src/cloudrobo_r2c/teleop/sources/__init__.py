"""遥操输入源实现包。

导入本包即注册内置源：

- ``mock_joints``：无硬件正弦波关节目标（链路联调 / 测试）。
- ``lerobot_leader``：LeRobot teleoperator 主臂（主从臂遥操）。

VR 输入源按 2026-11-05 决策移出实现（保留为设计阶段内容，见
``docs/design/r2c-teleop-data-collection-design.md``）。
"""

from . import lerobot_leader, mock  # noqa: F401  (import 触发注册)
