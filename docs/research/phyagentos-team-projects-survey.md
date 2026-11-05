# PhyAgentOS 团队项目调研

> 回应的问题：调研 GitHub 组织 [PhyAgentOS](https://github.com/PhyAgentOS) 这个团队做的项目。
>
> 数据来源：GitHub REST API 全量枚举该组织 26 个公开仓库（2026-11-05 采集）、各仓库 README/docs、[arXiv 2607.16636](https://arxiv.org/abs/2607.16636) 技术报告摘要、官方站点与发布记录。**仓库内的评测数字均来自其自述材料，本调研未复跑任何 benchmark**，具体口径见第 7 节。

---

## 0. 结论先行

1. **这是一个"具身智能运行时底座"团队，不是 VLA 模型团队。** 他们不做视觉-语言-动作模型本身，而是做"把 VLA/世界模型/Agent 规划器编排起来并稳定执行"的那一层：调度、验收、记忆、基准、安全。论文原话是 "a runtime foundation delivering scheduling, verification, memory, benchmarking, and safety as system-level services"。
2. **技术主张有明确记忆点**：以 **Session**（而非单次 action）为调度与验收的最小单位；把认知层与物理层的边界做成**文件系统**（State-as-File，Markdown + YAML）；用 **SessionVerifier** 区分"执行结束"与"任务真正完成"；把已验证经验沉淀为可复用 Skill/Lesson，实现**免训练的自进化**。
3. **工程上是"一核 + 一框架 + 一堆节点"的形态**：核心 `PhyAgentOS-core`（MIT，★2992 / fork 135）+ 执行框架 `forge`（Apache-2.0，Dora 数据流）+ 网关 + 8 个机器人适配器 + 3 个相机适配器 + 8 个算子/策略节点 + 1 个基准节点。
4. **团队背景扎实**：中山大学 HCP Lab、鹏城实验室、X-Era Lab 共建；arXiv 技术报告作者含 Liu Yang、Chen Weixing、Song Xinshuai、Pu Tao 等；GOSIM Shenzhen 2026 Spotlight（Open Source Robotics track）。
5. **成熟度是"活跃但仍在补工程债"**：v1.1.1（2026-10-23）、23 位贡献者、外部 PR 持续进入；但社区 issue #155 系统性地记录了从零部署时的安装协议、运行时构建、校验和锁死、GPU 准入等一长串坑，官方贡献范围目前也只开放 TrackB（Node/Skill/机器人集成）。

---

## 1. 团队与背景速览

| 维度 | 事实 |
|---|---|
| GitHub 组织 | `PhyAgentOS`，Organization，创建于 2026-08-14，公开仓库 26 个，followers 10 |
| 旗舰仓库 | [`PhyAgentOS-core`](https://github.com/PhyAgentOS/PhyAgentOS-core)，创建于 2026-03-12，★2992 / fork 135 / watcher 24 |
| 共建单位 | 中山大学 HCP Lab、鹏城实验室（Peng Cheng Laboratory）、X-Era Lab |
| 学术产出 | arXiv 2607.16636（2026-07-18）：*PhyAgentOS: A Self-Evolving Operating System for Embodied Agents with Decoupled Cognitive Planning and Physical Execution* |
| 核心作者 | Yang Liu, Weixing Chen, Xinshuai Song, Tao Pu, Siwen Mo, Yongjie Bai, Zihao Chen, Qianran Sun, Liruo Zhong, Ying Shen, et al. |
| 许可证 | core = MIT；forge = Apache-2.0 |
| 站点 | [phy-agent-os.x-era.com](https://phy-agent-os.x-era.com/)、[phy-agent-os.net](https://phy-agent-os.net/) |
| 社区渠道 | Discord、X、Bilibili、小红书、LinkedIn；PyPI 包 `PhyAgentOS-ai` |
| 公开成员 | 组织公开 member 仅 1 个（`putao537`），实际开发以 core 仓库 23 位贡献者计 |

贡献者分布（core 仓库，按 commits）：`sxshco` 108、`WissingChen` 63、`PhyAgentOS-dev` 62、`mrgh97` 54、`Chenzihao37` 12、`baiyu858` 10、`HCPLab-SYSU` 9、`yifanwin` 6 —— 属于"少数核心 + 长尾外部"的典型开源结构。

---

## 2. 技术主张（论文与 README 提炼）

| 主张 | 含义 | 解决的问题 |
|---|---|---|
| **Session-Centered Runtime** | 会话（session）而非动作（action）是最小调度单位，也是兼容性预检、受监督执行、证据收集与验收的单位 | 单步 action 成功不代表任务成功；跨层状态无处安放 |
| **State-as-a-File** | 认知层与物理层的边界是文件系统：跨层状态物化为 Markdown + YAML | 让状态可检视、可版本化，且两层之间无代码依赖，避免 Agent 与 Runtime 深度耦合 |
| **SessionVerifier** | 基于证据判定 success / failure / replan，区分"执行终止"与"语义完成" | VLA/策略的"跑完了"和"做对了"是两件事 |
| **Epistemic memory** | 已验证结果沉淀为可复用知识与纠错 Lesson，受控晋升 + 修订历史 | 免重训练的试错闭环（trial-and-error loop without retraining） |
| **Benchmark 复用部署链路** | 评测复用同一条 session + verification 路径 | 让评测结果可追溯到真实执行，而非另一个评测专用实现 |
| **分层安全** | preflight → action bridge → SafetyGuard → 心跳监控 → 目标侧约束 | 同时约束策略驱动与 Agent 驱动的执行 |
| **渐进式验证** | 游戏 → 仿真 → 真机，认知层保持不变 | 分层定位问题：先验认知，再验动力学，最后加硬件噪声 |

论文自述在 Optimus-67、StarDojo、DST-Dojo 上做了基准，在 19+ 仿真与真机本体上验证，并在 LIBERO、Calvin、RoboCasa365 上跨多个 VLA 模型取得增益。

---

## 3. 仓库全景（26 个）

分层关系：**认知层（core）→ 执行框架（forge）→ 网关（gateway）→ 适配器/算子节点**。

| 层 | 仓库 | 语言 | 说明 |
|---|---|---|---|
| 认知层 | **PhyAgentOS-core** | Python | RSI 框架本体：Agent、Skill、验证、进化、记忆，CLI 为 `paos` |
| 执行框架 | **forge** | Python/Rust/C++ | Dora 数据流框架核心：消息 schema、节点 runner、ToolEndpoint 契约 |
| 网关 | **adapter-forge-gateway** | Python | 统一运行时入口：HTTP 控制、就绪检查、Tool 路由、WebSocket |
| 机器人适配器 | adapter-robot-piper | Python | AgileX Piper（含 MuJoCo/URDF/GLB/USD 资产） |
| | adapter-franka-fr3 | Python | Franka FR3v2 + Franka Hand |
| | adapter-robot-so101 | Python | SO-101 |
| | adapter-robot-xlerobot | Python | XLeRobot 双臂轮式 |
| | adapter-robot-unitree-go2 | Python | Unitree Go2（SportClient，含看门狗 StopMove） |
| | adapter-robot-zerith_h1_pro | Python | Zerith H1 Pro |
| | adapter-robot-nexform-litearm | Python | NexForm LiteArm（今日仍在推送） |
| | adapter-robot-open-duck | Python | Open Duck Mini v2（RL 行走） |
| 相机适配器 | adapter-camera-usb | Rust | dora USB 相机节点 |
| | adapter-camera-orbbec | Python | Orbbec |
| | adapter-camera-realsense | Python | RealSense |
| 其他适配器 | adapter-viewer-image | Rust | 图像查看器 |
| 仿真算子 | operator-simulator-mujoco | Python | MuJoCo 仿真 |
| 控制算子 | operator-controls / operator-motion | Python | 机器人控制、运动 |
| 策略算子 | operator-kai0 | Python | kai0 policy 单可执行包 |
| | operator-lerobot-runner | Python | LeRobot ACT 转换与在线推理（`policy.runner`） |
| | operator-gripper-action-policy | Python | 夹爪动作策略 |
| | operator-relative-pose-policy | Python | 相对位姿策略 |
| | operator-sim-control-policy | Python | `sim.control`：机器人信息查询 + 场景重置 |
| 基准 | benchmark-libero | Python | LIBERO 封装，进程内持有 env 并评测 |
| 支撑 | PhyAgentOS-website | TypeScript | 官网（React + Vite） |
| | PhyAgentOS-legacy | Python | 旧实现，从 legacy 分支抽出并保留完整 git 历史 |

> 说明：除 `PhyAgentOS-core`（★2992）外，其余仓库 star 基本为 0–2，属**配套件而非独立项目**；fork 集中在 core（135）。`PhyAgentOS-legacy` 把旧实现连同完整 git 历史单独抽出，说明核心经历过一次重写（另有一个仍为 open 状态的 "rebuild Core runtime, CLI and Forge integration" PR）。

---

## 4. 核心仓库详解

### 4.1 PhyAgentOS-core —— 认知层与 RSI 框架

- **定位**：Recursive Self-Improvement infrastructure for physical agents。
- **版本线**：v0.2.2（2026-08-21）统一 Physical Execution 到 Query/Action Tool API；v0.2.3（2026-08-27）Skill 可独立安装/激活为不可变 AgentTask 绑定；v1.0.0（2026-08-30）首个稳定版；v1.1.0（2026-10-09）集中 provider 管理、OpenAI Responses 直连、新网关；**v1.1.1（2026-10-23）** Opper 网关、本地 Skill 开发、Action 拒绝处理、双语 TrackB 与 Piper X 指南。
- **代码构成**：Python ≈1.83 MB，另有 TypeScript 12 KB、Shell 11 KB、Dockerfile 2 KB；23 位贡献者。
- **核心抽象**：
  - **Skill**：把"某环境下的工具与运行时"打包成带版本的物理能力，含 profile、锁定的 Node 产物与 workflow 文档；通过 `paos skill search/install/inspect/start/status/logs/stop/reconcile` 管理。
  - **AgentTask 绑定**：Skill 激活后绑定为不可变 task，workflow 哈希随绑定与经验留存。
  - **验证与进化配置**（v1.1.1 默认值，值得注意）：
    - `verification`: `serviceEnabled=true`、`maxReplansPerEpisode=2`、`maxVerifierCallsPerRun=50`、`evidenceRetention=failed`
    - `evolution`: `enabled=true`、`scope=verified_forge_lineage`、`promotionMode=guarded_auto`、`minSuccessfulEpisodes=3`、`maxLessonsPerSkill=8`、`maxEvolutionCallsPerRun=20`
  - **Forge 执行约束**：`requestTimeoutS=10`、`executionTimeoutS=300`；证据要求 `requiredImageSources=["front"]`、`maxArtifactBytes=8 MB`。
- **CLI**：`paos onboard` / `paos provider configure|use|test` / `paos agent` / `paos status` / `paos skill ...`；会话内命令 `/provider`、`/model`、`/effort`、`/status`。
- **记忆会话**：`contextWindowTokens` 控制归档阈值；`/new` 先把完整原始会话存到 `<workspace>/sessions/archives/`，归档失败也会新建会话并报出备份路径——设计上"备份 ≠ 已完成的记忆摘要"。
- **对外约束**：PAOS 只连接"显式启动且健康"的 Skill Runtime 的 Gateway，**不会因为 Agent 启动就下载/拉起具体 Skill**。

### 4.2 forge —— 执行框架（Dora 数据流）

- **定位**：面向 Dora 数据流的轻量机器人框架核心，只做框架层，具体驱动下沉到下游仓库。
- **内容**：跨语言消息 schema（`interfaces/forge_msgs/*.v1.yaml`，含 common/control/motion/geometry/interaction/perception/sensor/tool）、Python/Rust/C++ 三套实现、标准 Robot Adapter / Policy Operator runner、ToolEndpoint 契约、可选 Pinocchio 运动学（FK/雅可比/DLS IK）。
- **消息语义**：`JointState`、`JointCommand`、`LocomotionCommand`、`Image`/`CompressedImage`、`PointCloud(Buffer)`、`Imu`、2D/3D 检测、`Pose`、`PolicyCommand`、`ToolMessage`（ToolEndpoint Wire 的单行 Arrow 载体）。
- **节点分类学**：Operator（Forge 原生计算，边界是 Forge 消息）vs Adapter（拥有外部设备/服务边界并翻译协议）。Gateway 属 Adapter，仿真器属 Operator。
- **ToolEndpoint**：Query / Action / Session 三类能力，正交于节点分类；v1alpha1 是首个带 tag 的公开 Tool 契约（Tool 0.1.0 + Msgs 1.2.0）。
- **版本策略**：六个包族（Common/Msgs/Robot/Policy/Kinematics/Tool）独立小版本；Common 2.1.0，其余 2.0.0 基线；`master` 走 Forge 2.x（配 Dora 1.x），`stable/1.x` 保留 Dora 0.4 兼容线。

### 4.3 adapter-forge-gateway —— 统一运行时入口

10 条 HTTP 路由模板：`GET /tools`、`GET /tools/{id}`、`GET /tools/{id}/context`、`POST /tools/{id}:invoke`、`POST /tools/{endpoint}/{op}:invoke`、`GET /invocations/{id}`、`/result`、`/cancel`、`/stop`、`/events`(SSE)。

安全声明很直白：**控制 API 默认无鉴权、无 TLS，仅监听 127.0.0.1**，远程访问需可信网络 + 认证加密反向代理，接真机前须自行验证急停、工作区隔离与设备限位。这条既体现诚实，也划定了它当前只是"本机/内网工具"而非生产网关。

---

## 5. 端到端链路

### 5.1 仿真（官方 Quickstart）

```text
paos skill start robot-control-sim --profile mujoco-piper
  → MuJoCo 仿真 + 图像查看器
  → Agent 读 sim.get_robot_info 与 Tool context
  → sim.set_joint_positions（关节角，rad）或 sim.move_eef（TCP 位姿，由 motion server 解 IK）
  → 读 action 结果、finalize、汇报
```

预置 profile：`mujoco-piper`、`mujoco-franka`、`mujoco-so101`。注意官方明示：该 demo 用 `verification.mode=off`，成功仅由 Tool 执行结果推导，不是语义验收。

### 5.2 真机（Piper）

Skill 安装 → CAN 配置 → 硬件启动 → 工具就绪检查；`adapter-robot-piper` 提供 MJCF/meshes/GLB 资产与 Forge robot driver，配置项含 `port`、`is_follower`、`motion_speed_percent`（首次测试建议 ≤30）、`positions.initial/reset`；`SAFETY.md` 明确"非功能安全认证系统，风险自负"。

### 5.3 基准（LIBERO）

`benchmark-libero` 在进程内持有 LIBERO env 并直接调用 `env.env._check_success()`，与 Policy Node 通过 Dora 端口解耦：

```text
proprio_state / image/main / image/second → Policy Node
action ← 7D JointCommand
policy_command → start / set_instruction / reset_scene / stop
→ evaluate → benchmark_status(stream) / benchmark_result(batch 结束)
```

Endpoint：`benchmark.controller` 的 `describe`(Query) 与 `run`(Session)。

---

## 6. 版本与仓库活跃度

- 旗舰仓库最后推送 2026-10-30；最新 release v1.1.1（2026-10-23）；公开 issue 10 个。
- `adapter-robot-nexform-litearm` 于 2026-11-05（调研当日）仍有推送，说明适配器矩阵在持续扩张。
- 外部 PR 活跃且有商业方参与：新增 Opper、Runcrate 等 OpenAI 兼容网关；Keenable 提交 web_search provider。这类"生态方主动适配"是项目有一定影响力的信号。
- 多语言策略明确：Python 为主，Rust 用于相机/USB 与高性能节点，C++ 覆盖 msgs/robot 以对接嵌入式与既有 C++ 栈。

---

## 7. 评测结果与口径（重要限制）

`docs/benchmarks.md` 自述：数值来自维护者提供的三张评测截图，**未注明评测日期、抽样任务 ID、各模型 episode 数、随机种子、checkpoint、计算预算、重置/重试策略与基线原始链接，也未附原始日志与可复现命令**；不同方法可能采用不同评测协议，不声明同条件排名或统计显著性。

LIBERO-Long（成功率 / 耗时 s/task / tokens/task）：

| 配置 | 成功率 | 耗时 | Tokens |
|---|---:|---:|---:|
| PAOS / GPT-6 + π0.5 | 100.00% | 434 | 1.95M |
| PAOS / GLM 5.3 Flash + π0.5 | 100.00% | 407 | 1.80M |
| PAOS / DeepSeek 4.1 Flash + π0.5 | 97.22% | 234 | 2.20M |
| π0.5（公开榜单参考） | 93.30% | 84 | 0 |
| π0（公开榜单参考） | 85.20% | — | — |
| PAOS / DeepSeek 4.1 Flash（无 π0.5） | 82.00% | 253 | 10.15M |
| OpenVLA（公开榜单参考） | 53.70% | — | — |

RoboDojo：PAOS/DeepSeek 4.1 Flash + π0.5 26.70%、DM0.5 24.97%、PAOS/GPT-6 + π0.5 23.30%、G0.5 20.23%。

`benchmark-libero` 另给出历史结果：VLA-JEPA 在 `libero_spatial` 上 100 episodes 达 94%（rendering 256）/96%（rendering 224），对比 OpenVLA 84.7%。

**判读建议**：PAOS 的增益主要来自"认知层 + 已有 VLA 策略"的组合（GPT-6/GLM/DeepSeek + π0.5），这与其"不做模型、做编排"的定位一致；但 token 成本高（LIBERO-Long 单任务 1.8–2.2M tokens，GUI 类 Agent 开销显著），且上述数字均未经第三方复现。文档本身把"可复现发布需要补充什么"列清楚了，这点是加分项。

---

## 8. 成熟度与风险

**正面**

- 发布节奏稳定（v0.2.x → v1.0.0 → v1.1.1，约两个月三个版本），有 CHANGELOG、双语文档、CONTRIBUTING、SECURITY。
- 抽象层次想得清楚：Skill 版本化 + 不可变绑定 + 证据留存 + 受控晋升，避免 Agent 随意改运行时。
- 主动披露限制：基线截图未复跑、网关无鉴权、真机非安全认证、历史二进制可能与当前源码能力不一致（"does not establish equivalent capability in a historical Release"）。
- 三语言 + 多本体覆盖（8 类机器人、3 类相机），生态扩展成本被压低。

**风险 / 值得警惕**

1. **开箱部署仍是主要痛点。** issue #155 系统记录了：Skill bundle 字段与 core 校验不匹配（`profiles.gpt6_pi05 has unknown field(s): required_tools`）、`cannot determine registry artifact size`、`Force node archive must contain exactly one executable file`、`chmod: follow_symlinks unavailable`；`ensure-runtime.sh` 硬编码清华镜像导致 `ResolutionImpossible`；修改 `runtime-inputs/` 脚本触发 `Runtime input checksum mismatch` 形成"不改跑不起来、改了被校验拒绝"的死锁；`reward_slot_runner.py` 要求 GPU 完全空闲、`fixed_runner.py` 默认 420s 超时不足。
2. **Dora 版本双线并存。** core/gateway 仍钉 `dora-rs==0.4.1`，而 forge `master` 已转向 Dora 1.x（`stable/1.x` 才保留 0.4 兼容）。跨仓库混用版本需要逐仓核对锁文件，官方也明说依赖声明不保证互操作。
3. **评测可复现性未闭环**（见第 7 节）。
4. **贡献范围受限**：官方只推荐从 TrackB（Node/Skill/机器人集成）入手；外部资源平台提交与已注册仓库变更"尚未开放"，需等审批服务上线。
5. **安全默认值偏弱**：网关无鉴权/无 TLS；真机示例鼓励低风险参数但没有硬件级急停保障，接产线需自行加固。
6. **认知层与执行层耦合在文件系统上**，这是设计取舍：换来可检视与可版本化，代价是高频/低延迟回路上不适合走这条状态通道（其数据面仍走 Dora/Arrow）。

---

## 9. 对我们的参考价值（CloudRobo SDK）

- **接口分层值得借鉴**：ToolEndpoint 的 **Query / Action / Session** 三分，与我们在 `cloudrobo-dispatch`（会话/任务）和 `cloudrobo-infer`（推理服务）上的职责切分可对照——尤其"Session 可 stop、Action 可 cancel"这种把长时任务与一次性动作分开建模的做法。
- **"执行结束 ≠ 任务完成"** 是具身场景的硬需求，`SessionVerifier` 的证据化判定（success/failure/replan + 证据留存策略 `evidenceRetention=failed`）可作为我们任务验收设计的参照。
- **Skill 打包模型**（profile + 锁定 Node 产物 + workflow 哈希 + 不可变绑定）与我们 `cloudrobo-asset` 的资产版本化有相似问题域，其"本地开发放宽、激活严格"的双档校验是不错的经验。
- **反面教材同样有用**：硬编码镜像源、校验和与二次开发互斥、GPU 准入过严、超时写死——这些都是我们在 SDK 默认配置里应当避免的形态。

---

## 10. 参考链接

- 组织：https://github.com/PhyAgentOS
- 核心仓库：https://github.com/PhyAgentOS/PhyAgentOS-core
- 执行框架：https://github.com/PhyAgentOS/forge
- 网关：https://github.com/PhyAgentOS/adapter-forge-gateway
- 技术报告：https://arxiv.org/abs/2607.16636
- 官方站点：https://phy-agent-os.x-era.com/ 、https://phy-agent-os.net/
- 框架介绍（中文）：https://phy-agent-os.x-era.com/docs/zh/01-framework-introduction
- GOSIM Shenzhen 2026 公告：https://github.com/PhyAgentOS/PhyAgentOS-core/blob/main/docs/news/gosim-shenzhen-2026_zh.md
- 部署问题汇总 issue：https://github.com/PhyAgentOS/PhyAgentOS-core/issues/155
- 评测口径说明：https://github.com/PhyAgentOS/PhyAgentOS-core/blob/main/docs/benchmarks.md
