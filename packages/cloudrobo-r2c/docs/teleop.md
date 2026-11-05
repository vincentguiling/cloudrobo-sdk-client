# 遥操与数据采集（Teleop）

> 本文件是使用指南（怎么跑、怎么配）；特性的设计文档（目标、架构决策、安全设计、决策点，与设备树设计同模板）见 [docs/design/r2c-teleop-data-collection-design.md](../../docs/design/r2c-teleop-data-collection-design.md)（仓库根 docs/design/）。

基于 R2C 数据面的遥操操作端：把**主从臂（leader 臂）**的输入映射为
`Actions` 下发给机器人，同时把（最新观测, 已发布动作）成对录制为遥操数据集文件。
现场遥操（局域网）已实现；远程遥操（互联网）与 VR 输入为设计阶段内容（见设计文档 v0.8），实现尚未包含。

```
┌────────────────────────── 操作端（本模块） ──────────────────────────┐
│  TeleopSource                       TeleopOperator                  │
│  ├─ lerobot_leader（主从臂）  ──►  读源 → 映射 → Actions ──► 发布    │
│  └─ mock_joints（联调）             │ 订阅 Observations（安全看门狗）│
│                                     ▼                               │
│                     TeleopSessionRecorder：(观测, 动作) 成对落盘      │
└──────────────────────────────┬───────────────────────────────────────┘
                               │ zenoh（LAN 直连 / 云端接入）
                        ┌──────▼──────┐
                        │  机器人端     │  cloudrobo r2c client
                        │  r2c_to_device 映射执行动作                 │
                        │  camera_streaming（LiveKit 推流，可选）      │
                        └─────────────┘
```

## 快速开始

### 1. 主从臂遥操（LeRobot 主臂）

```bash
# 机器人端（从臂，如 SO101 follower）
cloudrobo r2c client --client-config config/client_config.yaml \
    --robot-config config/robot_so101_lerobot_config.yaml

# 操作端（主臂，局域网直连）
cloudrobo r2c teleop --teleop-config config/teleop_so101_leader.yaml \
    --endpoints tcp/<机器人IP>:7447 \
    --record teleop_session.r2cr
```

### 2. VR 遥操（设计中，未实现）

VR 输入（`vr_udp` UDP 姿态流协议、相对位姿遥操）为设计阶段内容，见设计文档 v0.8 §3.3.1.4；实现尚未包含。

### 3. 操作端配置（`--teleop-config`）

```yaml
teleop:
  source:                     # 输入源
    type: lerobot_leader      # lerobot_leader | mock_joints（VR 源为设计阶段内容）
    config:
      type: so101_leader
      port: /dev/ttyACM1
      connect_kwargs: {calibrate: false}
  action:                     # 目标映射
    mode: joint               # joint | end_effector_pose
    action_hz: 30.0
    joint_map:                # joint 模式：源关节键 → 机器人 R2C 关节名
      shoulder_pan: joint_1
      gripper: joint_6
    # joint_scale:            # 可选：按关节缩放（镜像臂/单位差异）
    #   shoulder_pan: -1.0
    # ee_pose_map:            # ee 模式：源键(手) → Actions.end_effector_poses 键
    #   left: left_ee
    # gripper:                # ee 模式夹爪输出
    #   emit: joint           # joint | end_effector_state | none
    #   joint_name: gripper
    #   range: [0.0, 100.0]
  safety:
    require_observation: true   # 未收到机器人观测前不发动作
    observation_timeout_s: 2.0  # 观测中断超时后停发动作（恢复自动继续）
  recording:
    enabled: true
    path: teleop_session.r2cr
```

`--record <path>` 覆盖 `recording.path` 并强制开启；`--no-record` 强制关闭。

## 输入源

### lerobot_leader（主从臂）

LeRobot teleoperator 主臂（`so101_leader`/`koch_leader` 等，与
`robots/lerobot_hardware_adapter.py` 相同的 draccus 解码机制）。
`get_action()` 的 `<joint>.pos` 键去掉 `.pos` 后进入 `joint_positions`，
经 `joint_map` 映射为机器人关节名，**同单位同序透传**（机器人侧
`r2c_to_device` 映射按 `source_index` 落到对应关节）。

| 配置项 | 说明 |
|--------|------|
| `type` | LeRobot `TeleoperatorConfig` 类型（必填） |
| `port` / `id` / `calibration_dir` | 主臂串口与标定 |
| `connect_kwargs` | 透传 `connect()`，无人值守场景必须 `calibrate: false` |
| `gripper_key` | 夹爪关节基名（默认 `gripper`） |
| `gripper_range: [闭合值, 全开值]` | 设置后额外输出归一化开度 `TeleopTarget.gripper` |

### mock_joints（联调）

无硬件正弦波关节目标（`joints`/`amplitude`/`base`/`frequency_hz`），用于
不接主臂时验证操作端 → 机器人 → 观测回显的全链路。

## 现场（局域网）与远程（互联网）

数据面执行路径一致，差别只在 zenoh 接入与相机画面路径。**当前实现覆盖局域网现场遥操**（操作端 `--endpoints tcp/<机器人IP>:7447`，机器人端 `--endpoint-role listen`）；**远程遥操（互联网，经云端数据面 + LiveKit 推拉流画面）为设计阶段内容**，见设计文档 v0.8 §4.3，实现尚未包含。

安全策略：观测中断超过 `observation_timeout_s`、或源缺关节值时，操作端**停发动作**；恢复后自动继续。机器人侧的 `runtime.mode_control`（`teleop_enter`/`teleop_exit` 命令）可做接管/归还模式切换（见 `client/mode_control.py`）。

## 数据采集（遥操录制）

`TeleopSessionRecorder` 以 R2CR 格式（`common/utils/safe_serialization`，
无 pickle、支持 ndarray/bytes）成对记录：

```python
{
  "timestamp_ms": 1700000000000,   # 动作发布时刻
  "observation": {...},            # 当时最新机器人观测（图像为编码 bytes）
  "action": {...},                 # 本 tick 发布的 Actions（dict 快照）
  "source": {...},                 # 输入源原始目标（对齐/调试用）
  "source_name": "lerobot_leader"
}
```

回读：

```python
from cloudrobo_r2c.common.utils.safe_serialization import load_frames
frames = load_frames("teleop_session.r2cr")
```

机器人端另有观测-only 录制：`cloudrobo r2c client --record rec.r2cr`（观测
发布路径，见 `cloudroboclient`）；遥操动作+观测成对数据以操作端录制为准。

## CLI（`cloudrobo r2c teleop`）

| 选项 | 默认值 | 说明 |
|------|--------|------|
| `--teleop-config` | （必填） | 遥操配置 YAML |
| `--bundle` | - | 平台证书（远程遥操） |
| `--client-config` | - | client YAML（局域网/显式端点） |
| `--project-id` / `--device-id` | r2c / local | 目标机器人 |
| `--endpoints` | - | 逗号分隔 zenoh 端点（LAN：`tcp/<机器人IP>:7447`） |
| `--mode` / `--endpoint-role` | peer / connect | zenoh 连接模式与角色 |
| `--duration` | 0 | 运行时长（秒），0 = 直到 Ctrl+C |
| `--record` / `--no-record` | 配置为准 | 录制覆盖 |
| `--log-level` / `--log-file` | INFO / - | 日志 |

会话参数与 `cloudrobo r2c client` 一致（同一套 `build_session`）。

## 扩展输入源

实现 `TeleopSource`（`start`/`read`/`stop`，`read()` 非阻塞）并注册：

```python
from cloudrobo_r2c.teleop import register_source, TeleopSource, TeleopTarget

@register_source("my_device")
def create_my_device(config):
    return MyDeviceSource(config)
```

`create_source` 按配置 `source.type` 查注册表实例化；测试可直接注入
`teleoperator_factory`（`lerobot_leader`）或脚本源。