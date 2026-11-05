# R2C 遥操与数据采集特性设计

> 本文件负责"基于 r2c 的遥操与数据采集"特性的功能设计；不属于本文件的内容：设备树特性设计见 [PR #1 的 r2c-multi-device-tree-design.md](https://github.com/vincentguiling/cloudrobo-sdk-client/blob/agent/research-r2c-multidevice-d6cf83e2c104/docs/design/r2c-multi-device-tree-design.md)（该文档合入后改为仓库内相对路径），业界调研见 `docs/research/r2c-multi-device-industry-survey.md`，代码现状见 `docs/research/cloudrobo-r2c-codebase-survey.md`。
>
> 状态：**v0.5，待评审**。本版按反馈：①`teleop/observations` **只在本地流转、不上云**（由本地操作台端录制保存），数据格式与内容与 `inference/observations` 一致；**图像通过推拉流上云**；②遥操输入两类：**同构臂遥操**——操作前先把同构臂位姿对齐到当前位姿（读 `inference/observations`）；**VR 遥操**——相对位姿遥操，首帧位姿与 `inference/observations` 的 `end_effector_pose` 对齐，后续按 VR 相对位姿增量；③`teleop/actions` 下发频率 **20Hz**。前版：teleop 动作/观测独立话题、采集录 teleop 两话题、客户端不区分本地/远程。只出设计，不含实现。

## 1. 特性概述

**背景**：本特性主力场景是**遥操接管**——机器人按云端策略自主执行，遇异常时操作员接管，处理完交还自主；接管时段是价值最高的训练数据（回流配置中 `collect_method` 已有"遥操接管"）。现状：r2c 只有整机原始观测录制（`ObservationRecorder` + `--record`，cloudroboclient.py:188-193）与回放；遥操在 r2c 之外（LeRobot teleoperate、`--teleop.joint_sign`）。

**核心思路**：
1. **遥操动作独立话题**：`{p}/{d}/teleop/actions`，**20Hz** 下发，与策略动作 `{p}/{d}/inference/actions` 分开，互不覆盖；客户端两条来源都订阅，按"遥操新鲜优先、否则策略"执行。
2. **遥操观测独立话题**：`{p}/{d}/teleop/observations`，按 `publish_hz` 发布，**只在本地流转、不上云**（由本地操作台端录制保存）；**数据格式与内容与 `inference/observations` 一致**（同一 `Observations` 消息）。**图像通过推拉流上云**（不经 teleop/observations 承载，见 3.3.1）。
3. **本地/远程不区分**：本地操作台与远程操作员都发布到同一个 `teleop/actions`，客户端无本地/远程分支。
4. **采集=本地端录制两个话题**：`teleop/actions` 与 `teleop/observations`（本地录制器）。
5. **遥操输入两类**：同构臂——先对齐位姿再跟随；VR——相对位姿模式（首帧对齐 `end_effector_pose`）。

**目标**：
- 新增 `teleop/actions`（20Hz，客户端订阅、执行）与 `teleop/observations`（客户端按 `publish_hz` 发布，仅本地流转）；
- 两条动作来源的仲裁规则（遥操优先、超时回退策略）；
- 同构臂遥操的位姿预对齐流程；VR 遥操的相对位姿模式；
- 本地端录制 `teleop/actions` + `teleop/observations`，可轮转、可回放；
- 图像经推拉流上云（teleop/observations 不承载图像上云）；
- 配置最小化（客户端新增 ≤ 5 项，均有默认值）。

**非目标**（本版本不做）：客户端内建接管权限体系（由 Zenoh ACL/发布端承担）；推拉流服务端实现（复用现有视频通道/三方流媒体，本期只约定对接方式）；云端 episode 浏览/标注；多机器人协同遥操。

## 2. 需求分析

### 2.1 需求背景

| 维度 | 现状 | 差距 |
|------|------|------|
| 遥操动作通路 | 无；遥操在 r2c 外 | 需要与策略动作分开的遥操动作话题（20Hz） |
| 遥操观测 | 只有 `inference/observations`（给模型） | 需要本地高频的 `teleop/observations`（不上云），图像另走推拉流 |
| 遥操输入模式 | 只有外部主从臂 | 需要同构臂位姿预对齐与 VR 相对位姿两种模式 |
| 本地 vs 远程 | 概念上视为两种模式 | 客户端不应区分：同一 `teleop/actions` 话题 |
| 采集 | `--record` 录整机原始观测（cloudroboclient.py:415-425） | 需要本地端录制遥操两话题 |
| 回放 | playback 回放整机帧 | 录制产物需可回放 |

### 2.2 场景分析

**场景 1：遥操接管（主力）。** 策略在 `inference/actions` 上执行；操作员接管，开始向 `teleop/actions`（20Hz）发布。客户端检测到新鲜遥操动作即以遥操为准；操作台停止发布后回退到策略动作。接管期间 `teleop/observations` 按 `publish_hz` 本地发布（操作台反馈 + 本地录制），图像另行推拉流上云供远程观看。

**场景 2：同构臂遥操（同型号臂作主臂）。** 开始遥操前，客户端/操作台先把同构臂（主臂）的位姿**对齐到当前从臂位姿**（读 `inference/observations` 的当前 `end_effector_pose`/关节位姿），对齐完成前不下发动作；对齐后主臂增量即从臂增量。

**场景 3：VR 遥操（相对位姿）。** VR 控制器为输入：首帧 VR 位姿与 `inference/observations` 的 `end_effector_pose` 对齐（建立参考系），之后按 VR 相对位姿增量驱动从臂末端；无需物理对齐主臂。

**场景 4：远程遥操。** 操作员跨网络发布到同一 `teleop/actions`、图像经推拉流观看；`teleop/observations` 仍只本地流转（本地端录制）。

**场景 5：只采集。** 本地录制器记录遥操两话题。

**能力矩阵**：

| 需求点 | 现状 | 本特性 |
|--------|------|--------|
| 遥操动作与策略动作分离 | ✗ | ✔ `teleop/actions`（20Hz）vs `inference/actions` |
| 遥操观测本地高频、不上云 | ✗ | ✔ `teleop/observations` @ `publish_hz`，仅本地流转，格式与 inference 一致 |
| 图像上云 | 现有推理观测含图像 | ✔ 图像走推拉流（teleop 观测不承载上云） |
| 同构臂位姿预对齐 | ✗ | ✔ 操作前对齐到当前位姿（读 inference/observations） |
| VR 相对位姿遥操 | ✗ | ✔ 首帧对齐 `end_effector_pose`，后续增量 |
| 本地/远程一致 | 概念上分叉 | ✔ 同一 `teleop/actions`，客户端不分 |
| 动作来源仲裁 | 无 | ✔ 遥操新鲜优先、超时回退策略 |
| 本地端录制两话题 | ✗ | ✔ `teleop/actions` + `teleop/observations` |
| 回放 | 整机帧回放 | ✔ 按 topic 回放 |

### 2.3 友商分析

| 友商 / 业界实现 | 关键机制 | 启示 |
|----------------|----------|------|
| LeRobot | 主从臂 teleoperate、record/replay | 遥操与录制可以很薄：主臂→从臂、录制存帧 |
| Viam | 失联即停；数据 capture 按资源×方法 | 客户端只需超时即停；采集按来源分 |
| ROS 2 + Zenoh | topic → key expression；regex 策略控频率/QoS | 话题即接口；遥操/策略各走各的话题 |
| Genie Studio / 灵渠 OS | 采集任务与校验在云侧/工具链，端侧薄 | 端侧只发动作/录话题，编排上移 |

### 2.4 名词解释

| 术语 | 定义 |
|------|------|
| teleop/actions | 遥操动作话题；20Hz 下发；本地与远程操作台均发布到这里，客户端统一订阅 |
| teleop/observations | 遥操观测话题；客户端按 `publish_hz` 发布，**仅本地流转、不上云**（本地操作台端录制），格式与内容与 `inference/observations` 一致 |
| 推拉流 | 图像上云方式：客户端把相机画面推到流媒体服务，云端/远程操作台拉流观看；teleop/observations 不承载图像上云 |
| 同构臂位姿预对齐 | 同构臂遥操开始前，把主臂位姿调整到与从臂当前位姿一致（读 `inference/observations`），对齐完成前不下发动作 |
| VR 相对位姿遥操 | VR 首帧位姿与从臂当前 `end_effector_pose` 对齐建立参考系，之后按 VR 相对位姿增量驱动 |
| 动作来源仲裁 | 客户端在遥操动作与策略动作之间的执行选择规则：遥操新鲜优先，超时回退策略 |
| 发布权交接 | 接管/交还的实现方式：操作台开始/停止发布 `teleop/actions`，客户端不感知"接管"概念 |
| 录制器 | 订阅指定 topic、按到达时序落盘的订阅者（R2CR 格式） |

## 3. 功能方案

### 3.1 功能约束与依赖

**约束**：
- C1 遥操动作与策略动作**分话题**，互不覆盖：`teleop/actions`（20Hz）与 `inference/actions`；
- C2 客户端**不区分**本地/远程遥操，同一 `teleop/actions` 与同一执行路径；
- C3 `teleop/observations` 按 `publish_hz` 发布、**仅本地流转不上云**，格式与内容与 `inference/observations` 一致；
- C4 图像通过**推拉流**上云，`teleop/observations` 不承载图像上云；
- C5 遥操输入两类：同构臂（位姿预对齐）与 VR（相对位姿，首帧对齐 `end_effector_pose`）；
- C6 采集 = 本地端录制 `teleop/actions` + `teleop/observations`；
- C7 配置最小化（≤5 项，均有默认值）；兼容现有 `--record`、R2CR v1 与 playback。

**依赖**：
- `transport/zenoh.py`（pub/sub、QoS、TLS）与 `client/session.py`（topic 命名、发布/订阅方法）；
- `Actions`/`Observations` 消息与 `sync_client` 动作执行路径；
- `common/utils/recorder.py` + `safe_serialization.py`（R2CR）；
- Zenoh ACL（谁可发布遥操话题）。

### 3.2 整体方案设计

**技术选型**：

| 选型点 | 决策 | 依据 |
|--------|------|------|
| 遥操动作话题 | **新增** `{p}/{d}/teleop/actions`，**20Hz**，与 `inference/actions` 分离 | 策略与遥操互不覆盖、可独立录制与鉴权 |
| 遥操观测话题 | **新增** `{p}/{d}/teleop/observations`，按 `publish_hz` 发布，**仅本地流转**，格式与 `inference/observations` 一致 | 操作台反馈与本地录制；不上云降低公网带宽 |
| 图像上云 | **推拉流**：客户端推相机流到流媒体服务，云端/远程拉流 | 高频图像不走 Zenoh 观测话题；teleop/observations 保持轻量 |
| 同构臂预对齐 | 遥操开始前读 `inference/observations` 当前位姿，主臂对齐后才进入跟随 | 消除初始位姿差，避免起跳 |
| VR 相对位姿 | 首帧 VR 位姿与 `end_effector_pose` 对齐，后续按增量映射 | VR 无物理对齐手段，用参考系平移 |
| 仲裁规则 | 收到 `teleop/actions` 且在 `timeout_ms` 内 → 执行遥操；否则回退 `inference/actions`；两者都超时 → 停止 | 简单、确定、客户端无需接管状态机 |
| 本地/远程 | 不区分，同一话题 | 去掉客户端模式分支 |
| 采集 | 本地端录制 `teleop/actions` + `teleop/observations` | 与约定一致；R2CR 落盘；录制器在本地操作台侧 |

**整体架构**：

```text
发布端                                          客户端
  ├── 自主策略 ──► inference/actions ─────────► 仲裁（遥操新鲜优先）
  └── 操作台(本地/远程) ──► teleop/actions(20Hz) ─┘        │
                                                       ▼
                                                限幅/安全检查 → 设备

  客户端 ──► teleop/observations（publish_hz，仅本地流转）──► 本地操作台（反馈）
                                                        └──► 本地录制器 ─► recordings/*.r2cr
  客户端 ──► 相机画面 ──推流──► 流媒体服务 ──拉流──► 云端/远程观看（图像上云）
```

**关键时序（同构臂 / VR 遥操）**：

```mermaid
sequenceDiagram
    participant P as 自主策略
    participant O as 操作台（本地/远程）
    participant ZA as teleop/actions (20Hz)
    participant C as 客户端
    participant F as 设备
    participant ZO as teleop/observations（本地）
    participant R as 本地录制器

    P->>C: inference/actions（自主）
    Note over O,C: 开始遥操
    O->>C: 读取当前位姿请求（读 inference/observations）
    C-->>O: 当前 end_effector_pose / 关节位姿
    alt 同构臂
        O->>O: 主臂对齐到当前位姿（对齐完成前不下发）
    else VR
        O->>O: 首帧 VR 位姿与 end_effector_pose 对齐（建参考系）
    end
    loop 遥操（20Hz）
        O->>ZA: 遥操动作
        ZA->>C: 收到遥操动作
        C->>F: 执行（遥操优先）
        ZA->>R: 录制
    end
    loop publish_hz
        C->>ZO: teleop/observations（本地）
        ZO->>O: 操作台反馈
        ZO->>R: 录制
    end
    C->>C: 图像推流 → 流媒体（上云，不经 ZO）
    Note over O,ZA: 操作台停止发布 → 超时 → 回退策略
```

### 3.3 详细功能设计

#### 3.3.1 功能原理

- **动作订阅与仲裁**：客户端同时订阅 `inference/actions` 与 `teleop/actions`（20Hz 操作节拍）。执行规则：若最近 `timeout_ms`（默认 300ms）内收到遥操动作 → 执行遥操；否则若策略动作新鲜 → 执行策略；两者都超时 → 停止执行并保持。仲裁只在来源之间选择，不改变既有的 stale 判定、chunk 对齐、n-skip、fusion。
- **同构臂位姿预对齐**：遥操开始前，操作台读取当前位姿（`inference/observations` 的 `end_effector_pose`/关节位姿），驱动主臂运动到该位姿；对齐完成（误差 < 阈值）前操作台不下发 `teleop/actions`。对齐由操作台侧流程完成，客户端无需新状态机。
- **VR 相对位姿遥操**：VR 首帧位姿作为参考原点，与 `inference/observations` 的 `end_effector_pose` 对齐；之后按 VR 相对位姿增量（平移+旋转）叠加到参考位姿生成目标末端位姿。首帧对齐与增量映射在操作台/VR 侧实现。
- **遥操观测发布**：客户端在控制循环的每个 `publish_hz` 节拍组装 Observations，发布到 `teleop/observations`。**格式与内容与 `inference/observations` 一致（同一 `Observations` 消息）**，区别仅在话题、频率（`publish_hz` vs 0.5Hz）与流转范围（仅本地、不上云）。图像不在此话题承载上云。
- **图像推拉流**：相机画面由客户端推到流媒体服务（复用现有视频通道/三方流媒体，本版只约定对接方式），云端/远程操作台拉流观看；`teleop/observations` 保持轻量（非图像字段），仅本地流转。
- **本地/远程统一**：操作台无论本地进程还是远端节点，都向 `teleop/actions` 发布；`teleop/observations` 的消费者与录制器在本地。客户端无模式分支。
- **接管**：发布权交接——操作台开始/停止发布 `teleop/actions`；客户端仅按时间新鲜度仲裁。权限由 Zenoh ACL 控制。
- **采集**：本地录制器订阅 `teleop/actions` + `teleop/observations`，原样落盘（R2CR v1，按 topic 分文件），按大小/时长轮转；可选 `meta.json` 记设备树快照与配置哈希。
- **回放**：按录制文件与 topic 回放；playback 设备类型注入设备树。

#### 3.3.2 接口设计

**话题清单**：

| 话题 | 方向 | 频率 | 说明 |
|------|------|------|------|
| `{p}/{d}/inference/actions` | 策略 → 客户端 | 策略驱动 | 现有；策略动作 |
| `{p}/{d}/teleop/actions` | 操作台 → 客户端 | **20Hz** | **新增**；本地/远程遥操动作 |
| `{p}/{d}/inference/observations` | 客户端 → 云模型 | 0.5Hz（D3） | 现有；推理观测（含当前位姿，供预对齐/VR 首帧读取） |
| `{p}/{d}/teleop/observations` | 客户端 → **本地**操作台 | `publish_hz` | **新增**；仅本地流转不上云，格式与 inference/observations 一致 |
| 推拉流通道 | 客户端 → 流媒体 → 云端/远程 | 视频帧率 | **图像上云**（teleop/observations 不承载图像上云） |
| `{p}/{d}/state/*` | 客户端 → 云 | 各自配置 | 现有（joint_states/heartbeats 等） |

**配置（极简，全部有默认值）**：

```yaml
teleop:
  actions_topic: "{project_id}/{device_id}/teleop/actions"              # 默认
  observations_topic: "{project_id}/{device_id}/teleop/observations"    # 默认；仅本地订阅/录制
  actions_rate_hz: 20                                                    # 遥操动作下发频率（建议值，由操作台遵守）
  timeout_ms: 300                                                       # 遥操动作超时 → 回退策略

collection:                                          # 本地操作台侧录制器配置
  record: ["teleop/actions", "teleop/observations"]
  output_dir: ./recordings
```

不配 `teleop`/`collection` 段即为原有行为（不发布 `teleop/observations`、不录制）。

**CLI（在 `cloudrobo r2c client` 基础上增加）**：

| 命令/参数 | 说明 |
|-----------|------|
| `--record-topics "teleop/actions,teleop/observations"` | 命令行录制（等价 collection.record） |
| `--record-dir <dir>` | 录制目录 |
| `cloudrobo r2c record status` | 录制状态：topic、文件数、大小 |
| `cloudrobo r2c replay <record_dir>` | 回放 |

本地/远程操作台都只做"发布 `teleop/actions`、订阅 `teleop/observations`"，客户端无对应命令。

**界面设计**：无图形界面；录制状态用 CLI 与日志。

#### 3.3.3 数据结构设计

**录制目录**：

```text
recordings/
├── teleop_actions/2026-11-05T10-00-00.r2cr
├── teleop_observations/2026-11-05T10-00-00.r2cr
└── meta.json            # 可选：设备树快照、配置哈希、录制起止、publish_hz
```

- 文件内容沿用 **R2CR v1**（magic/version/frame_count/索引），帧为消息字节 + 时间戳；不引入新格式。
- 轮转按大小（默认 512MB）或时长（默认 10min），文件名带起始时间；`meta.json` 为可选旁注，不参与回放正确性。

## 4. 可靠性与异常设计

| 异常 | 行为 |
|------|------|
| `teleop/actions` 超时（默认 300ms） | 回退 `inference/actions`；若策略也超时 → 停止并保持 |
| 同构臂对齐未完成即收到动作 | 操作台不下发（对齐门禁在操作台侧）；客户端仍按新鲜度仲裁 |
| 操作台与策略同时发布 | 以遥操新鲜度为准（遥操优先）；策略动作不丢弃但暂不执行 |
| `teleop/observations` 发布失败 | 仅影响本地反馈/录制，不影响控制执行；记录告警 |
| 推拉流中断 | 只影响远程观看，不影响遥操控制与本地录制 |
| 远程链路抖动 | Zenoh 可靠传输/重连沿用现有；超时保护兜底 |
| 录制磁盘满 | 停止录制并告警，保留已写文件 |
| 录制丢帧 | 记录丢帧计数到 meta；不阻断控制 |
| 进程崩溃 | 已写 R2CR 文件可读；未完成文件标记 `.partial` |

## 5. 安全设计

- 谁能发布 `teleop/actions` 由 **Zenoh ACL** 控制；建议策略与操作台使用不同凭证，操作台凭证才可写遥操话题；
- 客户端保留限幅/限速；遥操使能（deadman）由操作台实现，客户端以超时即停兜底；
- TLS/凭证沿用现有 bundle；
- 录制文件路径限制在配置目录内；录制内容不含凭证。

## 6. 性能规格

- `teleop/observations` 仅本地流转：带宽占用在本地网段；格式与 `inference/observations` 一致（不含图像上云负担），30Hz 下小数据 KB/s 级（图像走推拉流）；
- `teleop/actions` @20Hz：每帧数十 KB（chunk 大小决定），约百 KB/s 级；
- 图像推拉流带宽单独计（视分辨率/码率，不经 Zenoh 话题）；
- 录制：磁盘吞吐为瓶颈；两话题同时录制时按 topic 独立文件；
- 客户端新增开销：一次来源仲裁 + 一路本地发布节拍，可忽略。

## 7. 兼容性与升级

- 新增两话题，不影响既有 `inference/*` 与 `state/*`；
- 不配 `teleop`/`collection` = 原有行为；`--record` 保留；
- R2CR v1 与 playback 兼容。

## 8. 测试与验收

| 层级 | 内容 |
|------|------|
| 单测 | 来源仲裁（遥操优先/超时回退/双超时停）、录制轮转与命名、回放解析、meta 生成 |
| 集成（仲裁） | 策略与遥操交替发布：遥操新鲜时执行遥操、停止后回退策略、双停则停止 |
| 集成（同构臂对齐） | 模拟主臂位姿 ≠ 当前位姿：对齐完成前操作台不下发；对齐后跟随 |
| 集成（VR 相对位姿） | 首帧对齐 end_effector_pose 后，增量映射正确、参考系不漂移 |
| 集成（遥操观测） | 校验 `teleop/observations` 以 `publish_hz` 发布（±10%）、仅本地订阅可见、格式与 `inference/observations` 一致 |
| 集成（推拉流） | 图像经流媒体可拉流观看；teleop/observations 无图像上云负载 |
| 集成（录制/回放） | 本地录制两话题，文件完整、可回放；磁盘满停止录制 |
| 验收标准 | 遥操与策略动作分话题且互不覆盖；`teleop/actions` 20Hz、`teleop/observations` 符合 `publish_hz` 且不上云；图像经推拉流可看；同构臂/VR 两模式按 3.3.1 语义工作；客户端无本地/远程分支 |

## 9. 遗留问题与后续规划

1. 推拉流服务端选型与对接（复用现有视频通道或三方流媒体；本期只约定客户端推流接口）；
2. 同构臂对齐的阈值与操作台引导 UI（对齐进度/误差显示）；
3. VR 相对位姿映射的奇异位形/工作空间边界保护；
4. 录制文件与 `cloudrobo-ingestion` 的目录/元数据对接细则（本地录制 → 上传）；
5. 远程遥操时延测量与跨公网 QoS 调优（动作 20Hz + 拉流）；
6. 操作台侧工具（主臂/手柄/VR 发布端）与校准；
7. 录制期间的 episode 分段（按接管起止自动切文件）。

## 10. 决策点汇总

| # | 决策点 | 状态 / 建议 |
|---|--------|-------------|
| U1 | 遥操动作是否独立话题 | **已定案**：独立 `teleop/actions`，与 `inference/actions` 分开 |
| U2 | 遥操观测话题与频率 | **已定案**：`teleop/observations`，按 `publish_hz` 发布 |
| U3 | 采集话题范围 | **已定案**：`teleop/actions` + `teleop/observations` |
| U4 | 遥操动作下发频率 | **已定案**：20Hz |
| U5 | teleop/observations 流转范围与格式 | **已定案**：仅本地流转不上云，格式与内容与 inference/observations 一致；图像走推拉流 |
| U6 | 遥操输入模式 | **已定案**：同构臂（位姿预对齐）与 VR（相对位姿，首帧对齐 end_effector_pose）两类 |
| U7 | 动作来源仲裁规则 | 建议：遥操新鲜优先（`timeout_ms` 默认 300），超时回退策略，双超时停止；待评审 |
| U8 | 录制粒度与轮转 | 建议：本地端按 topic 原样录制（R2CR v1），大小/时长轮转；待评审 |
| U9 | 配置项范围 | 建议：`teleop.actions_topic/observations_topic/actions_rate_hz/timeout_ms` + `collection.record/output_dir`；待评审 |
