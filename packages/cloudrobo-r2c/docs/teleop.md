# 遥操与数据采集（Teleop）

基于 R2C 数据面的遥操操作端：把**主从臂（leader 臂）**或 **VR 设备**的输入映射为
`Actions` 下发给机器人，同时把（最新观测, 已发布动作）成对录制为遥操数据集文件。
现场遥操（局域网）与远程遥操（互联网）共用同一套操作端，仅连接方式不同。

```
┌────────────────────────── 操作端（本模块） ──────────────────────────┐
│  TeleopSource                       TeleopOperator                  │
│  ├─ lerobot_leader（主从臂）  ──►  读源 → 映射 → Actions ──► 发布    │
│  ├─ vr_udp（VR 姿态流）             ▲                               │
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

### 2. VR 遥操（UDP 姿态流）

```bash
# 无 VR 设备联调：先跑 mock 发送端
python examples/vr_pose_mock_sender.py --port 9870

# 操作端
cloudrobo r2c teleop --teleop-config config/teleop_vr_udp.yaml \
    --endpoints tcp/<机器人IP>:7447
```

VR 桥接程序（Vision Pro / Quest 等侧的 tracking 桥）按下方协议发送 UDP JSON
数据报即可接入。

### 3. 操作端配置（`--teleop-config`）

```yaml
teleop:
  source:                     # 输入源
    type: lerobot_leader      # lerobot_leader | vr_udp | mock_joints
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

### vr_udp（VR）

监听 UDP 端口接收 JSON 姿态流（每数据报一帧，字段均可选）：

```json
{
  "ts": 1700000000000,
  "left":  {"pos": [x,y,z], "quat": [x,y,z,w], "grip": 0.8, "pinch": 0.4},
  "right": {"pos": [x,y,z], "quat": [x,y,z,w], "grip": 0.8, "pinch": 0.4},
  "head":  {"pos": [x,y,z], "quat": [x,y,z,w]}
}
```

约定：`pos` 单位米、右手系；`quat` 顺序由 `quat_order`（`xyzw` 默认 / `wxyz`）；
`grip` 0~1（1=张开）；`pinch` 0~1（1=捏合），同时存在优先 `grip`。

坐标变换按顺序应用：`scale` 缩放 → `workspace_offset` 平移 →
`axis_remap` 轴重映射（机器人轴 ← 源轴+符号，如 `{x: "+y", y: "-x", z: "+z"}`；
四元数按同一旋转矩阵同步变换）。无新数据超过 `stale_after_s` 判 stale。

| 配置项 | 默认 | 说明 |
|--------|------|------|
| `listen_host` / `listen_port` | `0.0.0.0` / `9870` | UDP 监听 |
| `hands` | `[left, right]` | 参与的手 |
| `ee_name_map` | 手名即末端名 | 源键 → `end_effector_poses` 键 |
| `scale` / `workspace_offset` | `1.0` / `[0,0,0]` | 工作区缩放与平移 |
| `axis_remap` | 恒等 | 轴重映射（含四元数） |
| `quat_order` | `xyzw` | 输入四元数顺序 |
| `stale_after_s` | `0.5` | 数据超时判 stale |
| `gripper.source` | `grip` | `grip` / `pinch`（取 1−x）/ `none` |
| `gripper.hand` | 第一只手 | 取哪只手的捏合/开合值 |

### mock_joints（联调）

无硬件正弦波关节目标（`joints`/`amplitude`/`base`/`frequency_hz`），用于
不接主臂/VR 时验证操作端 → 机器人 → 观测回显的全链路。

## 现场遥操（局域网）与远程遥操（互联网）

数据面完全一致，差别只在 zenoh 接入与相机画面路径：

| | 现场遥操（局域网） | 远程遥操（互联网） |
|---|---|---|
| zenoh | 操作端直连机器人：`--endpoints tcp/<机器人IP>:7447`（机器人端 `--endpoint-role listen --endpoints tcp/0.0.0.0:7447`） | 两端均连云端：操作端 `--bundle <平台证书zip>`（或云端 zenoh 接入点的 client-config） |
| 相机画面 | Observations 内嵌图像（jpeg/webp/png/h264）随数据面回传 | LiveKit WebRTC 推拉流：机器人端 `camera_streaming` 配置（`config/robot_so101_lerobot_config.yaml` 已含）经云端下发 `CAMERA_START` 后向 SFU 推流，操作者从云端控制台/客户端拉流观看；Observations 仍可走云端数据面 |
| 观测回传 | 直连 | 经云端数据面转发 |

安全策略（两种模式一致）：观测中断超过 `observation_timeout_s`、VR 数据流
stale、或源缺关节值时，操作端**停发动作**；恢复后自动继续。机器人侧的
`runtime.mode_control`（`teleop_enter`/`teleop_exit` 命令）可做接管/归还
模式切换（见 `client/mode_control.py`）。

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
