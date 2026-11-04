# R2C 多设备管理（设备树）特性设计 — 方案 B

> 本文件负责"基于 r2c 的多设备管理（多本体 / 多末端 / 多传感器）"方案 B（设备树 + 派生节点）的特性设计；不属于本文件的内容：业界调研依据见 [docs/research/r2c-multi-device-industry-survey.md](../research/r2c-multi-device-industry-survey.md)，R2C 现有单设备实现见 `packages/cloudrobo-r2c/`。
>
> 状态：**初稿，待评审**。章节按华为特性设计模板（特性概述 / 术语 / 需求分析 / 总体方案 / 详细设计 / 可靠性 / 安全 / 性能规格 / 兼容性 / 测试 / 遗留问题）组织；如团队有官方模板表单，本文各节内容可按编号 1:1 迁入。

## 1. 特性概述

**背景**：本版本要实现基于 r2c 的多设备管理。业界调研（PR #1 文档）结论：多设备组合应采用"设备树（浅层级 + 引用）+ 树上派生节点"形态；驱动场景为"云上模型需要当前六维力 + 2 秒内六维力历史"，采样 30Hz、模型调用 0.5Hz，且高频原始流不上云。

**定案范围**（2026-11-04 确认）：仅实现方案 B —— 设备树 + `history` 派生节点；方案 A（Zenoh 缓存按需查询）不做；方案 C（云端入湖窗口查询）暂不实现。

**特性价值**：
1. 多本体 / 多末端 / 多传感器在配置与消息层获得统一身份，不再是"命名约定扁平化"；
2. 历史等派生能力是树上的通用节点：**每个传感器只适配一次**，不出现"带历史 / 不带历史"两种驱动变体；
3. 采样频率（30Hz）与云端调用频率（0.5Hz）在树上解耦，高频数据不出端侧。

**目标**：
- `robot_config` 支持设备树定义（源设备 + 派生节点，引用式组合）；
- SDK 内置通用环形缓冲与 `history` 派生节点：按 `window_s × sample_hz` 维护滚动窗口，按 `publish_hz` 发布窗口批；
- 窗口批消息携带"当前值快照 + 窗口样本 + 起止时间戳 + 健康位"；
- 设备级生命周期与心跳扩展。

**非目标**（本版本不做）：云端多本体实时协同；`filter` / `tf` 派生节点实现（仅预留注册位）；入湖数据的云端窗口查询；Zenoh 高频原始流上云。

## 2. 术语

| 术语 | 定义 |
|------|------|
| 设备树 | `robot_config` 中 `devices` 列表构成的设备关系图：源设备 + 派生节点，靠引用（input / depends_on）组合，不做深层级内嵌 |
| 源设备（source） | 直接对接硬件/厂商 SDK 的设备节点（本体、末端、传感器），由现有 adapter 体系实现 |
| 派生节点（derived） | 以一个或多个源设备的输出为输入、产出派生信号的节点；本版本实现 `history`，预留 `filter` / `tf` |
| 环形缓冲（ring buffer） | SDK 通用组件：按 `(设备, 通道)` 维度缓存最近 `ceil(window_s × sample_hz)` 个样本，溢出覆盖最旧 |
| 窗口批（window batch） | `history` 节点按 `publish_hz` 发布的一条消息：当前值快照 + 窗口内全部样本 + 起止时间戳 + 健康位 |
| 样本（sample） | 一个时刻的设备读数：`timestamp` + 数据载荷（复用 `ExtensionValue` 自描述结构） |

## 3. 需求分析

**场景 1（驱动场景）**：六维力传感器 30Hz 采样；云上模型 0.5Hz 调用，每次需要"当前六维力 + 2 秒内历史（60 样本）"。约束：60 样本不在云端组窗（高频原始流不上云），必须端侧组装。

**场景 2**：多末端（双臂 + 各自夹爪）、多传感器（多相机 / IMU / 力）在同一台本体上共存，设备可独立启停、独立故障，互不拖垮。

**硬约束**：
- C1 六维力等高频原始采样流不出端侧（缓存于环形缓冲）；窗口批 0.5Hz 上行；常规 Observations 上行维持现状（见 5.3 / 决策点 D3）；
- C2 传感器适配只写一次，历史能力不进入任何传感器 adapter；
- C3 采样 / 历史频率（30Hz×2s）与模型调用频率（0.5Hz）解耦，各自可配置；
- C4 现有 v2.1 单设备配置与 Protobuf 消息保持兼容（只增不改）。

## 4. 总体方案

```text
robot_config (v3.0, devices 树)
  │
  ▼
DeviceManager（每 R2CClient 一个）
  ├── 源设备: left_ft (FT adapter)      ──30Hz──► RingBuffer(left_ft, cap=60)
  ├── 源设备: right_ft (FT adapter)     ──30Hz──► RingBuffer(right_ft, cap=60)
  ├── 派生: ft_hist (derived/history, input=left_ft, 0.5Hz)
  │        └─ 每 2s: 截窗 [now-2s, now) → ObservationWindow → Zenoh
  └── 派生: ...(filter/tf 预留)
  ▼
Zenoh: {project_id}/{device_id}/observations/window/{node_name}
  ▼
云模型（0.5Hz 订阅消费，纯订阅、无查询往返）
```

设计原则（对应调研结论）：
1. **本体在 key、设备在消息、信号在字段**：topic 首段仍为 `{project_id}/{device_id}`；设备身份用名称 + 类型进入配置与消息；窗口批 topic 增加派生节点名一段。
2. **树管"是谁、挂在哪"，引用管"用谁"**（devicetree phandle / Viam depends_on 同构）：浅层级 + 引用，不做深内嵌。
3. **派生能力是节点不是变体**（YARP remapper / ros2_control chaining 同构）：`history` 节点通用实现一次，所有源设备复用。

关键时序：

```mermaid
sequenceDiagram
    participant S as 源设备 left_ft (30Hz)
    participant R as RingBuffer(60)
    participant H as ft_hist (history, 0.5Hz)
    participant Z as Zenoh
    participant C as 云模型
    loop 每 33ms (30Hz)
        S->>R: push(sample{ts, 6维力})
    end
    loop 每 2s (0.5Hz)
        H->>R: snapshot [now-2s, now)
        R-->>H: ≤60 样本 + health
        H->>Z: publish ObservationWindow(current + samples + header)
        Z-->>C: 订阅消费（每次模型调用一份快照）
    end
```

## 5. 详细设计

### 5.1 配置模型（robot_config schema v3.0）

```yaml
schema_version: "v3.0"

runtime:            # 沿用 v2.1 字段，新增 window 批相关默认值
  publish_hz: 30.0
  ...

devices:
  # ---- 源设备 ----
  - name: left_ft
    type: force_torque            # 设备类型（本体/末端/传感器大类下的具体类型）
    adapter: ft300                # 实现，走现有 r2c_sdk.adapters entry-point 注册表
    params:
      port: /dev/ttyUSB0
      sample_hz: 30.0             # 源设备采样率，历史窗口容量按此折算
    translator:                   # 沿用现有 device_to_r2c / r2c_to_device 映射，归属到设备
      ...
  - name: right_arm
    type: arm
    adapter: bi_so_follower_right
    ...
  # ---- 派生节点 ----
  - name: left_ft_hist
    type: derived/history
    input: left_ft                # 引用（phandle 式）：单输入；多输入能力预留 input: [a, b]
    params:
      window_s: 2.0
      publish_hz: 0.5             # 与云模型调用频率一致
```

字段与校验规则：

| 规则 | 说明 |
|------|------|
| `name` 树内唯一 | 对齐 Viam 资源 name / ros2_control 块名；消息与 topic 按 name 引用 |
| `derived` 节点必须有 `input` | 引用必须指向已定义设备；允许引用派生节点（链式），配置加载时做无环检测 |
| 容量上限 | `window_s × sample_hz ≤ 4096`（防误配），超出拒绝加载并报错 |
| `derived/history` 参数默认值 | `window_s: 2.0`、`publish_hz: 0.5`（可覆盖） |
| 兼容 | 配置无 `devices` 块时，完全沿用 v2.1 单 `hardware` 块行为，不加载 DeviceManager |

**决策点 D1**：`devices` 列表是否需要显式层级字段（如 `parent`）？当前设计为"扁平列表 + 引用"，树形仅是引用关系图（业界证据：devicetree 单亲树 + phandle；Viam depends_on）。建议保持扁平，评审确认。

### 5.2 设备与派生节点运行时

- 新增 `DeviceManager`：加载 `devices` 列表，实例化源设备（复用现有 AdapterRegistry）与派生节点（新增 `r2c_sdk.derived_nodes` entry-point 组，镜像现有 adapter 注册表机制）。
- 生命周期（借鉴 ros2_control 组件状态机，调研已核验）：`UNCONFIGURED → INACTIVE → ACTIVE`；派生节点在其全部 `input` 均 ACTIVE 后才进入 ACTIVE。
- 错误隔离：单源设备 ERROR 只停用该设备与其下游派生节点（派生节点转 `DEGRADED`，见 5.4 健康位），不拖垮设备树其他分支——对齐 Viam "远端 unreachable 不消失" 语义。
- 心跳扩展（决策点 D2）：`Heartbeat` 新增 `repeated DeviceHealth devices`（name、state、error_code），整机字段保留；proto 只增不改。

### 5.3 数据面与 topic

| 数据 | 频率 | 去向 |
|------|------|------|
| 源设备原始采样（如 30Hz 六维力） | 30Hz | **仅端侧** RingBuffer，不出端（约束 C1） |
| 窗口批（history 派生节点输出） | 0.5Hz | Zenoh：`{project_id}/{device_id}/observations/window/{node_name}` |
| 常规 Observations（joint_states 等） | 维持现有 `runtime.publish_hz` | 沿用现有 topic（**决策点 D3**：是否为配合 0.5Hz 模型调用引入"快照模式"，本版本不强制） |
| Actions 下行 | 模型调用驱动 | 沿用现有 topic 与结构（多末端按设备键路由沿用 `end_effector_poses` map 模式） |

QoS / 可靠性：窗口批使用可靠传输（丢一批代价是模型拿到 4 秒旧窗口，宁可阻塞重传）；现有 Zenoh client 模式与 TLS 配置沿用，不新增传输配置面。

### 5.4 消息设计（Protobuf，只增不改）

新增 `observation_window.proto`（包 `r2c_sdk`，与现有 proto 同目录、同编译链）：

```protobuf
message WindowSample {
  int64 timestamp_us = 1;        // 样本时间戳（端侧单调时钟换算的 UNIX 微秒）
  ExtensionValue data = 2;       // 复用现有自描述载荷（六维力 = FLOAT64[6]）
}

message WindowHeader {
  string node_name = 1;          // 派生节点名（ft_hist）
  string source_device = 2;      // 输入设备名（left_ft）
  int64  seq = 3;                // 窗口批序号，递增，供丢批检测
  int64  window_start_us = 4;    // 窗口起止 [start, end)
  int64  window_end_us = 5;
  double sample_hz = 6;          // 源设备标称采样率
  int32  expected_samples = 7;   // 满窗样本数（60）；实际 samples 数可小于它
  HealthState health = 8;        // HEALTHY / WARMUP（未满窗）/ DEGRADED（输入故障/断流）
}

message ObservationWindow {
  WindowHeader header = 1;
  WindowSample current = 2;      // 最新样本（当前值快照）
  repeated WindowSample samples = 3;  // 窗口内样本，时间升序
}
```

**决策点 D4**：`data` 直接用 `ExtensionValue`（自描述、免扩 proto）还是为六维力定义强类型字段？建议 V1 用 `ExtensionValue`（零侵入、与 `extensions` 机制一致），评审确认。

**决策点 D5**：`Heartbeat.devices`（5.2）与窗口批 `health` 的关系：窗口批健康位只描述本派生链路；整机/全设备健康以 Heartbeat 为准。

### 5.5 环形缓冲与截窗实现

- `RingBuffer`：单写单读无锁环形队列；`push` O(1)、`snapshot` 一次批量拷贝；容量 `ceil(window_s × sample_hz)`，满后覆盖最旧。
- 截窗规则：以**样本时间戳**为准取 `[now − window_s, now)`（不按条数取——频率抖动时保证时间窗正确）；`expected_samples` 告知消费方满窗应为多少条。
- 时间基准：端侧写入双时间戳（单调时钟差值 + UNIX 时间），窗口起止用 UNIX 时间；端云时钟差由部署运维（NTP）保证在容差内，协议不做时钟补偿（**决策点 D6**：是否需要端云时钟差上报字段，V1 不做）。
- 断流检测：距上次 push 超过 `2 / sample_hz` 判定断流 → 窗口批 `health = DEGRADED`，并在头部保留 `expected_samples` 与实际样本数差异供定位。

### 5.6 配置、加载与热更新

- 配置校验在现有 `cloudrobo-r2c` CLI 加载链完成（引用不存在、成环、容量超限 → 拒绝启动并报具体节点名）。
- 本版本不支持运行中热改树（改配置需重启客户端）；与 Viam 热加载的差距记入遗留问题。

## 6. 可靠性与异常

| 异常 | 行为 |
|------|------|
| 源设备离线 | 该设备及其下游派生节点 DEGRADED，窗口批继续以低频发布（health 位标故障，样本为最后已知窗口）；设备树其他分支不受影响 |
| 缓冲未满窗（冷启动 / 断流恢复中） | `health = WARMUP`，samples 为实际条数，`expected_samples` 恒定提供满窗基准 |
| 发布失败 / Zenoh 断连 | 窗口批按可靠传输重传；**不做历史补发**（快照语义：恢复后从当前窗口继续，seq 供云侧检测丢批） |
| 时间倒退 | 丢弃早于窗口起点的新样本并计数告警（心跳 error_code） |
| 客户端重启 | 冷启动重新走 WARMUP → HEALTHY |

## 7. 安全

沿用现有 Zenoh TLS / 凭证 bundle 体系，无新增认证面；窗口批不携带新增敏感信息（仅设备名与传感数据）；topic 命名不引入可预测的越权路径（与现有同规则）。

## 8. 性能规格

- 上行带宽：每派生节点每批 ≈ 60 × (8B 时间戳 + 48B 六维力) + 头部 ≈ **4KB**，0.5Hz ≈ **2KB/s**；N 台设备线性叠加，远低于 30Hz 原始流式上行（消息数 1/60）。
- 端侧内存：每缓冲 60 × 56B ≈ **3.4KB**/设备（double 型信号；ExtensionValue 按实际 dtype）。
- CPU：push O(1)，每 2s 一次 O(窗口) 拷贝与 protobuf 编码，可忽略。

## 9. 兼容性与升级

- 配置：无 `devices` 块 = v2.1 原行为，存量配置零改动；`history_window` 相关字段不配置 = 不加载派生节点。
- Protobuf：只增不改（proto3 兼容规则）；旧云侧忽略新 topic 即可，无破坏。
- 回退：two-way door——回退客户端版本即回到单设备行为；无数据迁移。

## 10. 测试与验收

| 层级 | 内容 |
|------|------|
| 单测 | RingBuffer 边界（空/未满/满/覆盖）、截窗时间语义（频率抖动）、无环检测、配置校验报错 |
| 集成 | dummy 源设备（现有 `dummy` adapter + `robot_dummy_config.yaml` 扩展）+ `history` 节点 e2e：本地 Zenoh 起 router，校验 0.5Hz±10%、满窗 60 样本、seq 递增、health 状态迁移（WARMUP→HEALTHY→DEGRADED→HEALTHY） |
| 验收标准 | 云侧 0.5Hz 收到窗口批；每次模型调用样本时间跨度覆盖 2s；单设备故障不影响其他设备窗口批；v2.1 存量配置回归通过 |

## 11. 遗留问题与后续规划

1. `filter` / `tf` 派生节点实现（本版本仅注册位）；
2. 方案 C（入湖后云端窗口查询）作为训练 / 回放场景的后续特性；
3. 常规 Observations 上行频率与 0.5Hz 模型调用的对齐策略（决策点 D3）；
4. 运行中热更新设备树（对标 Viam 热加载）；
5. 多设备 `joint_states` 分组消息（Actions `end_effector_poses` map 模式推广到关节面）。

## 12. 决策点汇总（待评审）

| # | 决策点 | 本文建议 |
|---|--------|----------|
| D1 | 设备树用扁平列表+引用，还是显式 parent 层级 | 扁平列表 + 引用 |
| D2 | Heartbeat 增加逐设备健康列表 | 采纳（proto 只增） |
| D3 | 常规 Observations 上行是否对齐 0.5Hz"快照模式" | 本版本不强制，维持 publish_hz |
| D4 | 窗口批样本载荷用 ExtensionValue 还是强类型 | ExtensionValue |
| D5 | 窗口批 health 与 Heartbeat 的分工 | 窗口批只描述本链路，整机以 Heartbeat 为准 |
| D6 | 是否上报端云时钟差 | V1 不做，依赖 NTP 运维前提 |
