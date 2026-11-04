"""遥操输入源实现包。

导入本包即注册内置源：

- ``mock_joints``：无硬件正弦波关节目标（链路联调 / 测试）。
- ``vr_udp``：VR 姿态 UDP JSON 数据流（Vision Pro / Quest 等桥接程序
  按文档协议发送即可接入）。
- ``lerobot_leader``：LeRobot teleoperator 主臂（主从臂遥操）。
"""

from . import lerobot_leader, mock, vr_udp  # noqa: F401  (import 触发注册)
