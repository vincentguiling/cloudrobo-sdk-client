# 基于 R2C 的遥操与数据采集：特性设计文档

> 本文件负责"遥操与数据采集"特性的设计真源：目标、需求范围、总体架构、数据面、输入源、安全、数据采集与开放问题。不属于本文件的内容：使用方法与配置参考见 [packages/cloudrobo-r2c/docs/teleop.md](../packages/cloudrobo-r2c/docs/teleop.md)；多设备管理/设备树特性另立设计文档；业界调研背景见 `docs/research/r2c-multi-device-industry-survey.md`（research 分支）。

- 需求来源：2026-11-04 遥操与数据采集需求（实现基于 r2c 的遥操&数据采集，包含主从臂与 VR 两种方式，支持现场遥操（局域网）与远程遥操（互联网），远程相机画面基于 dev1103 的相机推拉流）。
- 设计日期：2026-11-05
- 状态：设计定稿；实现已随 [PR #2](https://github.com/vincentguiling/cloudrobo-sdk-client/pull/2) 提交，与设计的映射见 §9，未决事项见 §11。

## 1. 背景与目标

R2C 数据面（Zenoh pub/sub + Protobuf）此前只服务于"云端模型推理"闭环：机器人端发布 Observation、云端订阅后下发 Action。遥操（人操作机器人）与遥操数据采集（为训练收集 (观测, 动作) 对）缺少 SDK 级的**操作端**形态：

- 机器人侧的接管/模式切换（`ModeControl`）、相机推流（LiveKit）、录制回放等底座已随 r2c_sdk 0.2.8 同步落地；
- 操作端此前只有零散示例（键盘/摇杆云适配器），没有主从臂、VR 输入源抽象，也没有遥操会话的数据采集。

本特性目标：提供 SDK 级遥操操作端，使**同一套操作端配置**能在现场（局域网直连）与远程（互联网经云端）两种拓扑下工作，并在此过程中以 (观测, 动作) 成对形式完成数据采集。

## 2. 需求与范围

### 2.1 功能需求

| # | 需求 | 设计落点 |
|---|------|----------|
| R1 | 主从臂遥操：LeRobot 主臂（leader）关节值映射为机器人动作 | 输入源 `lerobot_leader`（§5.2）+ joint 映射（§6.1） |
| R2 | VR 遥操：VR 手柄/头部姿态映射为机器人末端动作 | 输入源 `vr_udp`（§5.3）+ EE 映射（§6.1） |
| R3 | 现场遥操（局域网）：操作端与机器人直连 | 会话拓扑（§4.3），`--endpoints tcp/<机器人IP>:7447` |
| R4 | 远程遥操（互联网）：两端经云端数据面，相机画面走推拉流 | 会话拓扑（§4.3）；相机画面由机器人端 `camera_streaming`（dev1103 底座）承担，操作端不重复实现 |
| R5 | 数据采集：遥操过程录制 (观测, 动作) 对 | `TeleopSessionRecorder`（§7） |
| R6 | 安全：失联/输入中断不得盲发动作 | 看门狗矩阵（§6.2） |
| R7 | 可扩展：新输入设备（手柄、动捕等）不改核心 | `TeleopSource` 注册表（§5.1） |

### 2.2 非目标（本期不做）

- 云端控制台/VR 端的 UI（相机拉流观看端由云端平台承担，SDK 只提供机器人端推流底座）；
- 机器人端新硬件适配器（遥操复用既有 `r2c_to_device` 映射执行动作）；
- 多设备/设备树建模（另立特性）；多操作端控制权仲裁（见 §11）；
- LeRobot 数据集格式（parquet+video）转换器（录制格式为其上游输入，见 §7.3）。

## 3. 总体架构

三角色分工，操作端为本特性的新增面：

```text
┌───────────────────────── 操作端（本特性） ─────────────────────────┐
│ TeleopSource（输入源：主臂 / VR / mock）                            │
│      └─ read() → TeleopTarget（本 tick 关节/末端/夹爪目标）         │
│ TeleopOperator                                                     │
│      ├─ 目标映射（joint_map / ee_pose_map）→ Actions（chunk_size=1）│
│      ├─ 发布 Actions；订阅 Observations（安全看门狗）               │
│      └─ TeleopSessionRecorder：(最新观测, 已发布动作) 成对落盘      │
└──────────────────────────────┬─────────────────────────────────────┘
                               │ zenoh（LAN 直连 / 云端接入，同一套 topic）
                        ┌──────▼──────────┐
                        │ 机器人端（既有）   │ cloudrobo r2c client
                        │  · r2c_to_device 映射执行动作（复用，不新增）│
                        │  · ModeControl 遥操接管（既有）             │
                        │  · camera_streaming LiveKit 推流（既有）    │
                        └─────────────────┘
```

设计原则：

1. **操作端只说 R2C 语义**：输入源输出设备原始值（设备坐标系/单位），到 `Actions` 的语义映射集中在操作端配置，机器人侧零改动。
2. **拓扑正交**：LAN 与互联网只影响 zenoh 接入方式与相机画面路径，遥操配置不变（§4.3）。
3. **安全默认开**：观测看门狗、输入 stale 判定默认启用，显式配置才能关闭。
4. **底座复用**：相机推拉流、模式切换、事件上报全部复用 dev1103 已同步能力，本特性不重复建设。

## 4. 数据面设计

### 4.1 Topic 复用

不新增任何 topic，完全复用推理闭环的数据面（保证与云端控制台、既有云适配器互通）：

| 方向 | Topic | 用途 |
|------|-------|------|
| 下行 | `{project}/{device}/inference/actions` | 操作端发布遥操动作（与模型下发同 topic，机器人侧同一执行路径） |
| 上行 | `{project}/{device}/inference/observations` | 操作端订阅观测：安全看门狗 + 数据采集的观测侧 |

### 4.2 动作语义

- 动作以 **`chunk_size=1` 单步**发布（遥操是连续流式控制，无 chunk 融合需求），`id` 单调递增，`timestamp` 为发布时刻墙钟毫秒。
- 关节动作：`joint_states.names` 与 `position[0]` 一一对应；**顺序即语义**——机器人侧 `r2c_to_device` 映射按 `source_index` 取值，因此操作端映射配置的键序必须与机器人配置对齐（§6.1）。
- 末端动作：`end_effector_poses` 按 map 键路由（`Pose7D`，7 元素 [tx,ty,tz,qx,qy,qz,qw]），单手缺位姿只影响该手（map 天然支持部分发布）。
- 夹爪：归一化开度 0~1（1=张开），由操作端按 `gripper.range` 映射为目标区间，按 `emit` 输出为附加关节（`joint`）或 `end_effector_states` 字段（`end_effector_state`）。

### 4.3 现场与远程拓扑

| | 现场遥操（局域网） | 远程遥操（互联网） |
|---|---|---|
| zenoh | 操作端 `--endpoints tcp/<机器人IP>:7447` 直连；机器人端 `--endpoint-role listen` 监听 | 两端均连云端 zenoh 接入点：操作端 `--bundle <平台证书>`（或云端 client-config） |
| 相机画面 | Observations 内嵌图像（jpeg/webp/png/h264）随数据面回传 | LiveKit WebRTC 推拉流：机器人端 `camera_streaming` 配置经云端下发 `CAMERA_START` 后向 SFU 推流，操作者从云端控制台/客户端拉流；Observations 仍走云端数据面（供看门狗与采集） |
| 遥操配置 | 不变 | 不变 |

设计取舍：远程相机画面**不**走 Observations 大图回传（互联网带宽下不可控），复用机器人端已实现的 LiveKit 推流底座（`camera/`，帧槽共享、设备独占持有，云端命令驱动）；操作端对相机的唯一依赖是"观测流活着"（看门狗），与画面路径解耦。

## 5. 输入源设计

### 5.1 抽象与注册表

```text
TeleopSource（抽象）          TeleopTarget（一个 tick 的目标值）
  start() / read() / stop()     ├─ joint_positions: Dict[str, float]   # 设备原始单位
  read() 非阻塞                 ├─ end_effector_poses: Dict[str, 7D]   # 已转目标坐标系
  SOURCE_REGISTRY 注册表        ├─ gripper: Optional[float]            # 归一化 0~1（1=张开）
                                └─ stale: bool                         # 输入超时（停发信号）
```

- 源只负责"读设备"，不做 R2C 语义转换；设备值 → 动作语义的映射全部在 operator 配置中。
- `read()` 必须非阻塞（设备读取在源内部线程完成），无数据返回 None / stale 目标，保证操作循环节拍稳定。
- 注册表 `SOURCE_REGISTRY` + `register_source("类型名")`：新设备以工厂函数注册即可接入，核心零改动（R7）。

### 5.2 主从臂：`lerobot_leader`

- 复用 LeRobot teleoperator 体系（`so101_leader`/`koch_leader` 等），解码机制与既有 `lerobot_hardware_adapter` 一致（draccus.decode + 内置子模块预导入 + 第三方插件注册），主臂与从臂在 SDK 层是同一套设备抽象。
- `get_action()` 的 `<joint>.pos` 键去 `.pos` 后进入 `TeleopTarget.joint_positions`；**同单位同序透传**（SO101 主/从臂标定同构），异构主从臂用 `joint_scale` 按关节缩放（镜像臂等场景）。
- 可选 `gripper_range: [闭合值, 全开值]`：设置后额外输出归一化夹爪开度（供 VR 式 EE 模式或独立夹爪通道使用）。
- lerobot 为延迟导入的可选依赖：不装 lerobot 不影响 SDK 其余能力。

### 5.3 VR：`vr_udp`

VR 设备生态碎片化（Vision Pro / Quest 各有桥接方案），设计选择**文档化 UDP JSON 协议 + 桥接适配点**，而不是绑定某一 VR SDK：

```json
{
  "ts": 1700000000000,
  "left":  {"pos": [x,y,z], "quat": [x,y,z,w], "grip": 0.8, "pinch": 0.4},
  "right": {"pos": [x,y,z], "quat": [x,y,z,w], "grip": 0.8, "pinch": 0.4},
  "head":  {"pos": [x,y,z], "quat": [x,y,z,w]}
}
```

约定：`pos` 米、右手系；`quat` 顺序 `quat_order`（xyzw 默认/wxyz）；`grip` 0~1（1=张开）；`pinch` 0~1（1=捏合，取 1−x 归一），同时存在优先 `grip`。

坐标变换流水线（把 VR 工作区坐标转到机器人基座系，全部可配）：

```text
pos → ×scale（工作区大小差异）→ +workspace_offset（VR 原点在基座系下的位置）
    → axis_remap（机器人轴 ← 源轴±符号，位置向量重排）
quat → 按 axis_remap 同一旋转矩阵 R 变换（q_robot = q_R ⊗ q_source）→ 归一化
```

- 超时 `stale_after_s`（默认 0.5s）无数据判 stale → 操作端停发（§6.2）。
- 夹爪来源可配（哪只手、grip/pinch）。

### 5.4 联调源：`mock_joints`

无硬件正弦波关节目标，用于不接主臂/VR 时验证"操作端 → 机器人 → 观测回显"全链路，也是单测的标准输入。

## 6. 动作映射与安全设计

### 6.1 映射配置

- **joint 模式**：`joint_map`（源关节键 → 机器人 R2C 关节名）必填；可选 `joint_scale`。
- **end_effector_pose 模式**：`ee_pose_map`（源键/手 → `Actions.end_effector_poses` 键）必填；夹爪按 `gripper.emit` 输出。

### 6.2 安全看门狗（默认全开，逐 tick 判定）

| 条件 | 处理 | 理由 |
|------|------|------|
| 尚未收到任何观测（`require_observation`，默认 true） | 停发 | 机器人不在场时不盲发 |
| 观测中断 > `observation_timeout_s`（默认 2.0，0 关闭） | 停发，恢复自动继续 | 遥操端与机器人失去联络不得继续驱动 |
| 输入源 stale（VR 流中断等） | 停发，恢复自动继续 | 旧姿态不代表操作者当前意图 |
| joint 模式源缺项 | **整 tick 跳过** | position 向量与 `r2c_to_device` 的 `source_index` 一一对齐，缺项发布会造成下标错位（错误关节收到错误值） |
| ee 模式单手缺位姿 | 其余手照常发布 | map 键路由天然支持部分发布 |

安全边界（明确不做）：本特性不做硬件急停（机器人侧安全由适配器/控制器承担）；操作端的安全职责是"失联即静默"。机器人侧遥操接管/归还复用既有 `runtime.mode_control`（`teleop_enter`/`teleop_exit` 命令，动作忽略 + 可选回位），由云端或操作方触发。

### 6.3 循环节拍

`action_hz`（默认 30，上限 500）定频循环；每 tick：读源 → 看门狗 → 映射 → 发布 → 录制。发布/跳过均限频记日志（2s 窗口），10s 一次状态汇总（published/skipped/观测年龄）。

## 7. 数据采集设计

### 7.1 帧模型

与 LeRobot 数据集帧结构对齐的最小单元，由操作端在**动作发布时**成对捕获：

```python
{
  "timestamp_ms": int,       # 动作发布时刻（墙钟毫秒）
  "observation": {...},      # 当时收到的最新机器人观测（图像为编码 bytes）
  "action": {...},           # 本 tick 发布的 Actions（dict 快照）
  "source": {...},           # 输入源原始目标（对齐/调试用）
  "source_name": "lerobot_leader"
}
```

在操作端（而非机器人端）采集的理由：动作在操作端生成， pairing 语义最直接；两种拓扑下观测流都经过操作端；机器人侧零改动。

### 7.2 录制格式：R2CR

复用 `common/utils/safe_serialization` 的 R2CR 格式（`[R2CR][version][count][index][frames]`，JSON 结构 + 原始 bytes/ndarray 块）：

- 相比 pickle：零代码执行风险（录制文件可能跨机流转）；
- 相比逐帧 protobuf：保留观测原始形态（编码图像 bytes、ndarray），回放/转换端自行解码；
- 观测图像保持**编码态**（jpeg/webp/h264 bytes）落盘，避免操作端解码-再编码损失。

### 7.3 与数据集转换的关系

R2CR 是采集暂存格式；训练前的 LeRobot 数据集转换（parquet + video 编码、episode 切分）作为独立下游工具（§2.2 非目标），`load_frames` 为其唯一读取入口。

机器人端另有观测-only 录制（`cloudrobo r2c client --record`，观测发布路径），用于回放调试；**遥操数据集以操作端成对录制为准**。

## 8. 配置设计

操作端配置独立于机器人配置（`--teleop-config`），顶层 `teleop` 节：

```yaml
teleop:
  source:      # 输入源：type + config（源自解释，§5）
  action:      # mode / action_hz / joint_map / ee_pose_map / gripper / gripper_joint
  safety:      # require_observation / observation_timeout_s
  recording:   # enabled / path
```

会话参数与 `cloudrobo r2c client` 完全一致（同一套 `build_session`）：`--bundle`/`--client-config`/`--endpoints`/`--mode`/`--endpoint-role`/`--project-id`/`--device-id`。`--record`/`--no-record` 覆盖录制配置。

## 9. 实现状态与设计一致性

实现已随 [PR #2](https://github.com/vincentguiling/cloudrobo-sdk-client/pull/2) 提交（分支 `agent/teleop-data-collection-72e38487b3f3`），与本文档的对应：

| 设计 | 实现 |
|------|------|
| §5.1 源抽象/注册表 | `teleop/base.py` |
| §5.2/5.3/5.4 三源 | `teleop/sources/{lerobot_leader,vr_udp,mock}.py` |
| §6 映射/看门狗/循环 | `teleop/operator.py`、`teleop/mapping.py` |
| §7 录制 | `teleop/recording.py`（复用 `common/utils/safe_serialization`） |
| §8 配置/CLI | `teleop/cli.py`、`config/teleop_so101_leader.yaml`、`config/teleop_vr_udp.yaml`、CLI 注册 `cloudrobo r2c teleop` |
| §4.3 远程相机画面 | 底座 `camera/`（dev1103 同步，PR #2 第一层） |

验证摘要：41 项单测（源映射/坐标变换/看门狗/录制回读/随包配置校验）；真实 zenoh E2E 冒烟通过（机器人端收 40 动作 @20Hz、关节映射正确、40 帧录制观测配对）；CLI 与 VR mock 发送端冒烟通过。

## 10. 测试与验证策略

- **单测**（无硬件）：源注册表、VR 协议解析/坐标变换/stale、leader 映射与夹爪归一化（测试替身注入）、看门狗逐条、chunk 语义、录制 round-trip、随包配置可解析。
- **E2E 冒烟**（真实 zenoh，LAN 拓扑）：mock 源 → 操作端 → 机器人端动作接收 + 观测回灌 + 录制配对断言。
- **真机验收清单**（部署时执行）：SO101 主/从臂标定对齐、VR 桥接端到端时延、互联网拓扑下相机推拉流与看门狗行为、急停值守演练。

## 11. 开放问题

| # | 事项 | 说明 |
|---|------|------|
| Q1 | VR 设备协议标准化 | 当前为本文档协议（桥接端适配成本低）；若后续指定具体 VR 硬件/官方桥（如 Vision Pro Tracking Streamer），评估是否收敛为原生源 |
| Q2 | 观测/动作时间戳对齐 | 录制帧用墙钟毫秒配对；若数据集要求严格软同步（观测采集时刻 vs 动作执行时刻），需引入观测侧硬件时间戳透传 |
| Q3 | 多操作端控制权仲裁 | 同一机器人被两个操作端同时驱动时无互斥；短期靠流程约束，中期可复用 ModeControl + 云端命令扩展（另立设计） |
| Q4 | LeRobot 数据集转换器 | R2CR → LeRobot parquet/video 的独立工具，训练侧提出后再立项 |
| Q5 | 设备树/多设备集成 | 多本体/多末端遥操（如双臂双主臂）依赖设备树特性建模，待其设计定稿后对齐本特性的源/映射配置形态 |
| Q6 | CI 覆盖 | r2c 测试目录纳入 CI pytest 路径（部署凭据缺 workflow scope，已在 PR #2 描述留一行修改建议） |
