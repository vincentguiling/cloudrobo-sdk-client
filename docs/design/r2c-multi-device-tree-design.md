# R2C 多设备管理（设备树）特性设计

> 本文件负责"基于 r2c 的多设备管理（多本体 / 多末端 / 多传感器）"特性的功能设计；不属于本文件的内容：业界调研与代码完全调研见 `docs/research/`（`r2c-multi-device-industry-survey.md`、`cloudrobo-r2c-codebase-survey.md`），R2C 现有单设备实现见 `packages/cloudrobo-r2c/`。
>
> 状态：**v0.4，待评审**。本版修订：①补充场景 2（ROS 机械臂+夹爪、双 OpenCV 相机）完整配置示例；②支持设备级独立配置文件 + 相对路径引用；③移除派生节点概念——历史/滤波等能力改为 entry-point 注册的普通设备类型。按特性设计模板组织（特性概述 / 需求分析〔需求背景、场景分析、友商分析、名词解释〕/ 功能方案〔功能约束与依赖、整体方案设计、详细功能设计〕/ 可靠性 / 安全 / 性能规格 / 兼容性 / 测试 / 遗留问题）。

## 1. 特性概述

**背景**：本版本要实现基于 r2c 的多设备管理（多本体、多末端、多传感器等）。现状的多设备形态是"整机扁平化"——设备没有独立身份、独立生命周期与独立故障隔离，映射表为全局单表（详见 2.1）。

**特性价值**：
1. 多本体 / 多末端 / 多传感器在配置与消息层获得统一身份与设备树结构；
2. 设备级生命周期、健康与故障隔离；
3. 历史、滤波、坐标变换等能力作为**普通设备类型**（entry-point 注册）提供，**每个设备只适配一次**，不随能力数量膨胀出驱动变体；
4. 采样频率与云端模型调用频率解耦，高频原始流不出端侧。

**目标**：
- `robot_config` 支持设备树定义：adapters（传输上下文）+ devices（设备，各自可独立配置文件）；设备按 `type` 经 entry-point 解析实现；
- 设备级生命周期状态机与逐设备心跳健康；
- 首批设备类型：本体/机械臂、末端/夹爪、OpenCV 相机、历史（history，预留 filter/tf）；
- 历史等设备输出经 `Observations.extensions` 随唯一观测入口上行，云端不感知端侧实现细节。

**非目标**（本版本不做）：云端多本体实时协同；`filter` / `tf` 设备类型的实现（仅预留注册位）；入湖数据的云端窗口查询；高频原始流上云；图形化运维界面。

## 2. 需求分析

### 2.1 需求背景

版本需求来自本版本规划："实现基于 r2c 的多设备管理（如多本体、多末端、多传感器等）"。对照代码完全调研（`docs/research/cloudrobo-r2c-codebase-survey.md`，结论均带 file:line），现状与差距：

| 维度 | 现状 | 差距 |
|------|------|------|
| 设备身份 | 一个 client/session = 一个 `device_id`（整机，session.py:93-341） | 多设备无独立身份，跨设备仅有订阅覆盖参数 |
| 适配器 | 一个 adapter = 一台整机；多设备靠键前缀约定（`leftarm_/rightarm_`、`camera_*`）或内部子对象 | 无对外子设备声明 API；新 adapter 无法声明"我有哪些设备" |
| 映射 | `device_to_r2c/r2c_to_device` 全局单表，无 device 字段（config_mapper.py:136-154） | 无法按设备分组映射与校验 |
| 消息 | Observations/Actions 扁平；仅 `images.color.<cam>` 与 `extensions` 天然分组 | 关节面无分组；设备级输出无标准承载 |
| 生命周期 | 整机 Heartbeat（默认 5s，status/mode/error_code/battery） | 无逐设备状态、健康与故障隔离 |
| 频率 | sync_client 单节拍循环：控制=采样=发布（`publish_hz` 默认 30，sync_client.py:326,922-945） | 采样率与云端模型调用率（事件驱动，无频率参数）耦合 |
| 配置 | 单一 robot_config + 内联映射表（`common/robot_config_schema.py` pydantic，`extra="allow"`） | 无设备级配置拆分与相对引用；未知字段静默 |

### 2.2 场景分析

**场景 1：多传感器设备 + 历史能力。** 多类传感器（六维力、IMU、多相机）挂在一台本体上。示例：六维力传感器 30Hz 采样，云上模型 0.5Hz 调用，每次需要"当前值 + 2 秒内历史（60 样本）"，且高频原始流不上云——历史必须在端侧组装，且不能让力传感器出现"带历史 / 不带历史"两种适配。→ 由"历史设备"（`type: history`）覆盖。

**场景 2：共享传输 + 异构传输 + 同型多实例。** 机械臂与夹爪走 ROS 通信（共享同一 ROS 上下文），两个相机走 OpenCV：

```text
ros:
  - 机械臂 (arm)
  - 夹爪 (gripper)
opencv 相机 1
opencv 相机 2
```

要点：第一层是**传输上下文**（"ros:" 是一个 ROS 上下文，臂与夹爪共享它），相机自带采集、与 ROS 无关，且两个相机是同一设备类型的两个实例。→ 由 adapters（传输）+ devices（设备类型 + 独立配置文件）覆盖，完整示例见 3.3.2。

**场景 3：多本体。** 多台整机各自一个 `device_id` 会话，云端按设备编排与汇聚（本版本不承诺云端实时协同）。

**能力矩阵**：

| 需求点 | 现状 | 本特性 |
|--------|------|--------|
| 设备身份（独立命名/类型） | ✗（键前缀约定） | ✔ devices 声明 + entry-point 类型 |
| 共享传输（多设备一个上下文） | 内部隐式 | ✔ transport 引用同一 adapter |
| 同型多实例（两台相机） | 内部隐式 | ✔ 两个 device 实例 |
| 异构传输并存（ROS + OpenCV） | 分散在各 adapter | ✔ 设备各自绑定/自带传输 |
| 独立故障隔离 | 整机级 | ✔ 单设备 DEGRADED |
| 能力复用（历史/滤波/变换） | adapter 内私有（如 flexiv wrench_filtered） | ✔ 独立设备类型，一次实现处处可用 |
| 配置可维护（设备级拆分） | ✗（单文件内联） | ✔ 设备独立配置文件 + 相对引用 |
| 频率解耦（30Hz 采样 / 0.5Hz 上行） | ✗（单节拍） | ✔ 双节拍 |
| 设备输出上云（云端不感知端侧实现） | ✗ | ✔ extensions 约定键，唯一观测入口 |

### 2.3 友商分析

| 友商 / 业界实现 | 关键机制 | 对本特性的启示 |
|----------------|----------|----------------|
| Viam | 资源模型（API+model 三元组+全局唯一 name）、sub-part/remote part 并入统一资源图、断线"标 unreachable 不消失"、单表 readings 六级元数据 | 设备命名与重名裁决；断线不消失语义；观测按 (org,location,machine,part,component) 组织 |
| ros2_control | system/sensor/actuator 组件 + `<设备>/<信号>` 接口命名；组件生命周期状态机 + on_error；命令接口独占仲裁 | 设备生命周期状态机与 HardwareStatus 健康消息；接口独占仲裁模型 |
| LeRobot | 配置即组合（注册表 + 嵌套 dataclass）；双臂/多相机键前缀分区 | 配置驱动组合形态；entry-point 注册表思想；前缀分区规范 |
| ROS 2 + Zenoh（rmw_zenoh / bridge） | topic → key expression 映射；按接口 regex 的频率/优先级/QoS 策略面；liveliness 上下线事件 | key 命名规范（高方差段靠右）；上下线事件替代心跳轮询（后续演进） |
| 灵渠 Link-U OS | AimRT 中间件（ROS 2 兼容 Topic/RPC）；`/aima/hal/<设备类型>/<安装位>/<信号>` 一路一 topic；aimrt_health_monitor（节点心跳/急停） | 传感器命名层级 `<类型>/<安装位>/<信号>`；健康监控清单 |
| Genie Studio（智元） | "Multi-Ontology and Multi-End Effector"采集任务模型；吞吐按 device 计量；Monitor 多机运维 | 设备作为配额/状态/质量的基本计量单元；多机运维独立模块 |
| Unitree SDK2 / ABB MultiMove | 按控制层级分 topic、扁平状态消息；工业多本体协同在控制器层 | 云端不做实时协同，保持每 device_id 独立执行体 |

完整对比与来源见 `docs/research/r2c-multi-device-industry-survey.md`。

### 2.4 名词解释

| 术语 | 定义 |
|------|------|
| 设备树 | `robot_config` 中 adapters（传输上下文）+ devices（设备）+ 引用关系（transport / input）构成的关系图；浅层级 + 引用，不做深内嵌 |
| 传输上下文（adapter） | 一个共享的通信/驱动上下文（一个 ROS 上下文、一个串口总线、一路相机采集），走现有 `r2c_sdk.adapters` 注册表；多个设备可引用同一上下文 |
| 设备（device） | 设备树的功能单元（本体、末端、相机、历史…）；`type` 经 `r2c_sdk.devices` entry-point 解析为具体实现；可绑定传输上下文（`transport`）或自带采集 |
| 设备类型（device type） | entry-point 注册的设备实现名（如 `arm`、`gripper`、`opencv_camera`、`history`）；**历史/滤波/变换等都是设备类型**，不是特殊节点 |
| 配置引用（config_file） | 设备/传输配置可写到独立文件，通过相对路径引用引入；路径相对引用方文件所在目录解析 |
| 环形缓冲（ring buffer） | 设备通用组件：按 `(设备, 通道)` 维度缓存最近 `ceil(window_s × sample_hz)` 个样本，溢出覆盖最旧 |
| 样本（sample） | 一个时刻的设备读数：`timestamp` + 数据载荷（复用 `ExtensionValue` 自描述结构） |
| 窗口批（window batch） | 历史设备注入 `Observations.extensions` 的扩展字段（键 `<设备名>.window`）：当前值快照 + 窗口内全部样本 + 起止时间戳 + 健康位 |

## 3. 功能方案

### 3.1 功能约束与依赖

**约束**：
- C1 高频原始采样流不出端侧：按设备声明的采样率写入端侧环形缓冲，仅设备输出上行；
- C2 设备适配只写一次：历史/滤波等能力以设备类型实现，不进入任何驱动型的设备实现；
- C3 采样（及历史窗口）频率与云端模型调用频率解耦，各自可配置；
- C4 兼容：v2.1 存量配置与 Protobuf 消息只增不改；
- C5 观测唯一入口 `{project_id}/{device_id}/inference/observations`，不新起 topic（2026-11-04 定案）；设备输出经 `extensions` 扩展字段承载，云端不感知端侧实现细节。

**依赖**：
- Zenoh 传输与 QoS 体系（沿用 `client/session.py` + `transport/zenoh.py`，不新增 topic 类型）；
- `r2c_sdk.adapters` entry-point 注册表（传输实现复用现有 16 个 adapter）；
- 新增 `r2c_sdk.devices` entry-point 注册表（设备类型）；
- 现有 `_config_dir` 配置目录传递机制（robot_factory.py:235-237）作为配置引用的基础；
- `common/robot_config_schema.py` pydantic 校验链（需扩展，见 3.3.2）；
- `Observations.extensions`（`map<string, ExtensionValue>`）与 proto 兼容规则。

### 3.2 整体方案设计

**技术选型**：

| 选型点 | 决策 | 依据 |
|--------|------|------|
| 组合层级 | adapters（传输上下文）+ devices（设备，可引用 transport）+ 设备间引用（`input`） | 与"一个 adapter = 一个上下文"的代码现状同构（ros2 adapter 持有 ROS 上下文）；同一 ROS 上下文下臂与夹爪是两个设备；相机自带采集不与 ROS 绑定 |
| 设备身份 | 设备 `name` 全局唯一 + `type` 经 entry-point 解析 | 对齐 Viam 资源 name / ros2_control 块名惯例；树负责"是谁、挂在哪"，引用负责"用谁" |
| 设备发现 | `describe_devices()` 元数据 API 自动推导 + 配置覆盖 | ros2 `get_transport_targets`、a1z `camera_meta`、命令注册表均为现有雏形 |
| 能力复用 | 新 entry-point 组 `r2c_sdk.devices`：`history`/`filter`/`tf` 与 `arm`/`gripper`/`opencv_camera` 同为一等设备类型 | ros2_control controller chaining / YARP remapper 同构：能力是设备，不是设备变体 |
| 配置组织 | 支持设备/传输级独立配置文件，通过 `config_file` 相对路径引用；展开后统一校验 | 设备和传输各自演进、复用（同一相机配置可用于多台整机）；现有 `_config_dir` 机制可承载 |
| 设备输出承载 | `Observations.extensions` 约定键 `<设备名>.window`，载荷 BYTES（proto 序列化） | 现有 `map<string, ExtensionValue>` 机制零侵入；观测唯一入口（C5） |
| 上行频率 | 观测上行 = 云端模型调用频率（0.5Hz）；采样按设备 `sample_hz` 独立节拍写环形缓冲 | 现状 sync_client 单节拍（控制=采样=发布）需拆为双节拍 |

**整体架构**：

```text
robot_config (v3.0: adapters 传输上下文 + devices 设备 + 引用)
  │
  ▼
DeviceManager（每 R2CClient 一个）
  ├── transport: ros (ros2)                        # 一个 ROS 上下文，被多个设备共享
  │     ├── device: arm      (type: arm)     ──30Hz──► RingBuffer(arm)
  │     └── device: gripper  (type: gripper) ──30Hz──► RingBuffer(gripper)
  ├── device: camera1 (type: opencv_camera, 自带采集) ──30Hz──► RingBuffer(camera1)
  ├── device: camera2 (type: opencv_camera, 自带采集) ──30Hz──► RingBuffer(camera2)
  ├── device: left_ft (type: force_torque)          ──30Hz──► RingBuffer(left_ft)
  └── device: left_ft_hist (type: history, input=left_ft)
        └─ 每上行周期: 截窗 [now-window_s, now) → 注入 extensions["left_ft_hist.window"]
  ▼
Zenoh: {project_id}/{device_id}/inference/observations（观测唯一入口）
  │  常规字段 + extensions（含设备输出）一并上行
  ▼
云模型（按调用频率订阅消费；只按约定键读 extensions，不感知端侧实现）
```

**关键时序**：

```mermaid
sequenceDiagram
    participant S as 源设备 left_ft (30Hz)
    participant R as RingBuffer
    participant H as 历史设备 left_ft_hist
    participant P as 上行组装 (0.5Hz)
    participant Z as Zenoh
    participant C as 云模型
    loop 采样节拍 (按设备 sample_hz)
        S->>R: push(sample{ts, 数据})
    end
    loop 上行节拍 (= 模型调用频率)
        P->>S: 各设备 get_observation 组装常规字段
        H->>R: snapshot [now-window_s, now)
        R-->>H: 样本 + health
        H->>P: extensions["left_ft_hist.window"] 注入
        P->>Z: publish Observations
        Z-->>C: 订阅消费
    end
```

### 3.3 详细功能设计

#### 3.3.1 功能原理

- **装载**：`DeviceManager` 读取 `adapters`（实例化传输上下文，复用现有 AdapterRegistry）与 `devices`（按 `type` 从 `r2c_sdk.devices` 解析实现并构造）；设备通过 `transport` 绑定传输上下文，或自带采集（如相机）。
- **双节拍**：采样节拍按设备 `sample_hz` 拉取数据写环形缓冲；上行节拍按模型调用频率组装 Observations（常规字段 + 各设备输出注入的 extensions），经现有会话发布。现状 sync_client 的单节拍循环拆分为两节拍。
- **生命周期**：借鉴 ros2_control 组件状态机——设备 `UNCONFIGURED → INACTIVE → ACTIVE`；`input` 引用设备（如历史设备依赖力传感器）在其输入设备 ACTIVE 后激活；单设备 ERROR 仅影响依赖它的设备（转 DEGRADED），其他设备不受影响。
- **故障隔离**：单设备离线不拖垮整机；断线资源"标记不可用而非消失"（对齐 Viam 语义），恢复后自动回归。

#### 3.3.2 接口设计

**配置接口**（robot_config schema v3.0）：两个顶层段——`adapters`（传输上下文）与 `devices`（设备）；二者都可整项或部分来自独立文件。

```yaml
schema_version: "v3.0"

adapters:                                   # 传输上下文
  - config_file: ./adapters/ros.yaml        # 整项来自文件（相对本文件目录）
  - name: serial_ft
    type: ft_serial
    params: {port: /dev/ttyUSB0, baudrate: 921600}

devices:                                    # 设备
  - config_file: ./devices/arm.yaml         # 整项来自文件
  - config_file: ./devices/gripper.yaml
  - config_file: ./devices/camera1.yaml     # 相机：独立文件 + 独立 entry-point
  - config_file: ./devices/camera2.yaml
  - config_file: ./devices/left_ft.yaml
  - config_file: ./devices/left_ft_hist.yaml  # 历史：独立文件 + 独立 entry-point
```

**场景 2 完整示例**（ROS 机械臂 + 夹爪、双 OpenCV 相机）：

`robot_config.yaml`（整机入口）：

```yaml
schema_version: "v3.0"

adapters:
  - config_file: ./adapters/ros.yaml

devices:
  - config_file: ./devices/arm.yaml
  - config_file: ./devices/gripper.yaml
  - config_file: ./devices/camera1.yaml
  - config_file: ./devices/camera2.yaml
```

`adapters/ros.yaml`（共享 ROS 上下文）：

```yaml
name: ros
type: ros2
config:
  subscriptions:
    - {topic: /arm/joint_states,     msg_type: sensor_msgs/msg/JointState, store_as: arm_joint_states}
    - {topic: /gripper/gripper_state, msg_type: sensor_msgs/msg/GripperState, store_as: gripper_state}
  publishers:
    - {topic: /arm/joint_command,    msg_type: sensor_msgs/msg/JointState}
    - {topic: /gripper/gripper_command, msg_type: sensor_msgs/msg/GripperCommand}
```

`devices/arm.yaml`：

```yaml
name: arm
type: arm                    # entry-point: r2c_sdk.devices → 机械臂设备实现
transport: ros               # 引用 adapters 中的传输上下文
config:
  joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]
  state_key: arm_joint_states
  command_key: arm_joint_command
  sample_hz: 30
```

`devices/gripper.yaml`：

```yaml
name: gripper
type: gripper
transport: ros               # 与 arm 共享同一个 ROS 上下文
config:
  state_key: gripper_state
  command_key: gripper_command
  sample_hz: 30
```

`devices/camera1.yaml`（相机 1，自带 OpenCV 采集，不依赖 ROS）：

```yaml
name: camera1
type: opencv_camera          # entry-point: r2c_sdk.devices → OpenCV 相机实现
config:
  index_or_path: "/dev/video0"
  width: 640
  height: 480
  fps: 30
  sample_hz: 30
```

`devices/camera2.yaml`（相机 2，同类型第二个实例）：

```yaml
name: camera2
type: opencv_camera
config:
  index_or_path: "/dev/video2"
  width: 640
  height: 480
  fps: 30
  sample_hz: 30
```

**历史设备示例**（场景 1）：

`devices/left_ft.yaml`：

```yaml
name: left_ft
type: force_torque
transport: serial_ft
config:
  channel: wrench          # 六维力 [fx, fy, fz, tx, ty, tz]
  sample_hz: 30
```

`devices/left_ft_hist.yaml`（独立文件 + 独立 entry-point）：

```yaml
name: left_ft_hist
type: history              # entry-point: r2c_sdk.devices → 历史设备实现
input: left_ft             # 引用另一个设备（可跨 transport）
config:
  window_s: 2.0            # 窗口随观测上行，无独立发布频率
```

**配置引用规则**：

| 规则 | 说明 |
|------|------|
| `config_file` | 整项配置来自指定文件；列表项可只写 `{config_file: ...}`，也可写在其他字段旁（文件内容与内联字段深合并，内联字段优先） |
| 路径解析 | 相对**引用方文件所在目录**解析；支持 `./`、`../`、绝对路径；被引用文件内的相对路径继续按其自身目录解析（可嵌套） |
| 循环检测 | 引用链出现环 → 加载失败并报出完整链路 |
| 展开后校验 | 全部文件展开合并后统一做 schema 校验（必填、唯一性、引用完整性、容量上限），错误信息带文件路径 |
| 环境 | 与现有 `_config_dir` 机制一致，配置根目录传给 adapter 工厂（robot_factory.py:235-237） |

**校验规则**：

| 规则 | 说明 |
|------|------|
| `adapters.<name>.type` | 走现有 `r2c_sdk.adapters` entry-point 注册表；一个 adapter = 一个传输上下文 |
| `devices.<name>.type` | 走新增 `r2c_sdk.devices` entry-point 注册表；设备名全局唯一 |
| `devices.<name>.transport` | 可选；必须指向已定义 adapter；相机等自带采集的设备不填 |
| `devices.<name>.input` | 可选；引用其他设备名（可跨 transport），用于历史/滤波/变换类设备；加载时做存在性与无环检测 |
| 容量上限 | `window_s × sample_hz ≤ 4096`，超出拒绝加载——需在 `robot_config_schema` 显式新增校验（现状 pydantic `extra="allow"` 对未知字段静默，代码调研偏差 #6） |
| 兼容 | 配置无 `devices`/`adapters` 块时，完全沿用 v2.1 单 `hardware` 块行为，不加载 DeviceManager |

**元数据 API**：`IRobotHardwareAdapter` 增加可选覆盖 `describe_devices() -> list[DeviceDeclaration]`（`name/type/channels`），默认返回空（由配置声明兜底）；ros2 适配器从 `SubscriptionConfig.store_as` 前缀推导。

**消息接口**：观测唯一入口不变；extensions 约定键见 3.3.3；proto 只增不改。

**界面设计**：本特性为 SDK/配置层特性，无新增图形界面；多机运维可视面（对标 AimStudio 式工具）列入后续规划。

#### 3.3.3 数据结构设计

**设备输出的 extensions 约定**（以历史设备为例）：

```text
extensions["<设备名>.window"] = ExtensionValue{
  dtype: BYTES,
  data:  ObservationWindow 序列化字节
}
```

键按设备名（如 `left_ft_hist.window`）——每个设备把自己的输出发布在自己的键下；云端按集成约定读取设备键，不感知端侧实现。

新增 `observation_window.proto`（包 `r2c_sdk`，与现有 proto 同目录、同编译链；只增不改）：

| 消息/字段 | 说明 |
|-----------|------|
| `WindowSample{timestamp_us, data}` | 单样本：UNIX 微秒时间戳 + `ExtensionValue` 自描述载荷（六维力 = FLOAT64[6]） |
| `WindowHeader{source_device, seq, window_start_us, window_end_us, sample_hz, expected_samples, health}` | 窗口头：`seq` 供云侧丢批检测；`expected_samples` 为满窗基准；`health ∈ {HEALTHY, WARMUP, DEGRADED}` |
| `ObservationWindow{header, current, samples}` | 载荷整体：当前值快照 + 窗口内样本（时间升序） |

**Heartbeat 扩展**（只增）：`repeated DeviceHealth devices`（name、type、state、error_code）——逐设备健康，整机字段保留。

**分组演进**：`images.color.<cam>` 与 `end_effector_poses` map 已按名分组；`joint_states` 的多设备分组列为后续特性（见遗留问题 3）。

## 4. 可靠性与异常设计

| 异常 | 行为 |
|------|------|
| 设备离线 | 该设备及依赖它的设备转 DEGRADED，其他设备不受影响；恢复后自动回归 |
| 环形缓冲未满窗（冷启动/断流恢复） | health = WARMUP，samples 为实际条数，`expected_samples` 恒定提供满窗基准 |
| 上行失败 / Zenoh 断连 | 沿用现有可靠传输重试；设备输出不做历史补发（快照语义），`seq` 供云侧检测丢批 |
| 配置文件缺失/引用成环 | 启动失败，报出引用链与文件路径 |
| 时间倒退 | 丢弃早于窗口起点的新样本并计数告警（心跳 error_code） |
| 客户端重启 | 冷启动重新走 WARMUP → HEALTHY |
| 断流检测 | 距上次样本超过 `2 / sample_hz` 判定断流 → health = DEGRADED |

## 5. 安全设计

沿用现有 Zenoh TLS / 凭证 bundle 体系，无新增认证面；extensions 扩展字段不新增敏感数据；配置文件引用仅允许读取配置目录内路径（防目录穿越），越界路径拒绝加载；不引入可预测的越权路径（与现有 topic 规则一致）。

## 6. 性能规格

- 上行带宽：以历史设备为例，每设备每批 ≈ 60 × (8B 时间戳 + 48B 六维力) + 头部 ≈ 4KB，0.5Hz ≈ 2KB/s/设备；N 台设备线性叠加；远低于原始流式上行（消息数 1/60）。
- 端侧内存：每环形缓冲 60 × 56B ≈ 3.4KB/设备（double 型信号；ExtensionValue 按实际 dtype）。
- CPU：缓冲写入 O(1)，每上行周期一次 O(窗口) 拷贝与 protobuf 编码，可忽略。
- 配置加载：多文件展开 + 校验一次性完成，启动期开销，与文件数线性。

## 7. 兼容性与升级

- 配置：无 `adapters`/`devices` 块 = v2.1 原行为，存量配置零改动；设备类型不配置 = 不加载。v2.1 单 `hardware` 块保留。
- Protobuf：只增不改（proto3 兼容规则）；旧云侧忽略 extensions 新键即可，无破坏。
- 回退：two-way door——回退客户端版本即回到单设备行为；无数据迁移。

## 8. 测试与验收

| 层级 | 内容 |
|------|------|
| 单测 | RingBuffer 边界（空/未满/满/覆盖）、截窗时间语义（频率抖动）、设备引用无环检测、配置引用（相对路径/嵌套/成环/越界）、配置校验报错（含未知字段显式校验） |
| 集成（场景 1） | 力设备 + 历史设备 e2e：本地 Zenoh 起 router，校验上行频率 ±10%、满窗样本数、seq 递增、health 状态迁移（WARMUP→HEALTHY→DEGRADED→HEALTHY） |
| 集成（场景 2） | ros2（arm+gripper 共享上下文）+ 两个 opencv_camera 实例（各自配置文件引用）：设备独立加载、异构传输并存、单设备故障不影响其他设备 |
| 验收标准 | 云侧按模型调用频率收到 Observations，extensions 含约定设备键；v2.1 存量配置回归通过；单设备故障不影响其他设备；设备配置可独立成文件并被相对引用 |

## 9. 遗留问题与后续规划

1. `filter` / `tf` 设备类型实现（本版本仅注册位）；
2. 入湖数据的云端窗口查询（训练 / 回放场景）；
3. 常规 Observations 内多设备 `joint_states` 分组消息（`end_effector_poses` map 模式推广到关节面）；
4. 运行中热更新设备树与配置文件重载（对标 Viam 热加载）；
5. 多机运维可视面（多机连接、批量部署、告警）；
6. 存量 16 个 adapter 分批补 `describe_devices()`（ros2/lerobot 优先，其余先回退配置声明 channels）；新增 `opencv_camera`、`history` 设备类型实现；
7. rmw_zenoh liveliness 上下线事件替代心跳轮询（长期演进）。

## 10. 决策点汇总

| # | 决策点 | 状态 / 建议 |
|---|--------|-------------|
| D1 | 组合层级 | **已定案**：adapters（传输上下文）+ devices（设备类型 + transport/input 引用）；不采用显式 parent 深层级 |
| D2 | Heartbeat 增加逐设备健康列表 | 采纳（proto 只增） |
| D3 | 观测上行频率 | **修订**：对齐云端模型调用频率（0.5Hz）；sync_client 拆采样/上行双节拍 |
| D4 | extensions 载荷格式 | BYTES 承载 `ObservationWindow` proto 序列化；备选：拆 data/ts 两个张量字段（待评审） |
| D5 | 设备输出 health 与 Heartbeat 的分工 | 设备输出只描述本设备，整机以 Heartbeat 为准 |
| D6 | 是否上报端云时钟差 | V1 不做，依赖 NTP 运维前提 |
| D7 | 设备输出承载方式 | **已定案**：不新起 topic，观测唯一入口，extensions 承载，云端不感知端侧实现 |
| D8 | 配置引用机制 | 采纳 `config_file` 相对引用（相对引用方目录、可嵌套、防环、展开后统一校验）；待评审确认路径沙箱范围（5 节：限制在配置目录内） |
