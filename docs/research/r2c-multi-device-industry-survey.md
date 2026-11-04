# 基于 R2C 的多设备管理：业界实现调研

> 本文件负责汇总业界"多设备管理（多本体 / 多末端 / 多传感器）"的实现方案，供 CloudRobo R2C 数据面 SDK 本版本设计参考；不属于本文件的内容：R2C 现有单设备实现细节见 `packages/cloudrobo-r2c/`（`docs/index.md`、`src/cloudrobo_r2c/common/protos/`），多设备方案设计定稿后另立设计文档。

- 调研日期：2026-11-04
- 结论摘要见[横向对比](#横向对比)、[对 R2C 多设备设计的启示](#对-r2c-多设备设计的启示)与[补充调研：设备树与派生能力抽象](#补充调研设备树与派生能力抽象六维力-2-秒历史示例)。
- 每节末尾标注主要来源；公开资料无法核实的点如实标注"未能核实"。

## 背景与调研范围

本版本要实现基于 r2c 的多设备管理（多本体、多末端、多传感器等）。调研目标：业界同类系统如何对设备建模、组合、在数据面上复用通道、管理生命周期、仲裁控制权、以及云端如何纳管多设备。

### R2C 现状（单设备模型，作为对比基线）

依据本仓库 `packages/cloudrobo-r2c`（源码与 `config/*.yaml`、`common/protos/*.proto`）：

| 维度 | 现状 |
|------|------|
| 设备身份 | 一个 `R2CClient` 进程 = 一个 `device_id`；topic 形如 `{project_id}/{device_id}/inference/observations` / `.../actions`。"多本体" = 起多个进程、多个 device_id，无编排层 |
| 消息模型 | `Observations`/`Actions` 单条扁平消息：多臂拼进 `joint_states.names` + 一个位置向量（如双臂 12 维）；多相机靠 `images.color.{top,wrist,...}` map 键名区分；`Actions.end_effector_poses` 已是 `map<string, EndEffectorPoseChunk>`（按末端分）；`IMUState` 有 `name` 字段但未挂入 Observations；另有自描述 `extensions` map |
| 组合配置 | `robot_config.yaml` 单 `hardware` 适配器 + 扁平 `device_to_r2c` / `r2c_to_device` 映射表；双臂 = 一个 lerobot `bi_so_follower` 适配器内部拆两个串口 |
| 生命周期 | `Heartbeat` 整机级（status / mode / error_code / battery），无按设备的心跳与健康状态 |
| 数据面 | 单 topic 单消息承载全部设备；无按设备的频率/带宽/QoS 控制 |

即：多末端 / 多传感器目前靠"命名约定扁平化"解决，设备没有消息级身份；多本体没有统一管理面。

### 调研对象

| # | 实现 | 组织 / 类型 | 层次定位 |
|---|------|------------|----------|
| 1 | Viam | Viam（商业端到端平台） | 云 + 端，组件模型 |
| 2 | ros2_control | ROS 2 官方控制框架 | 机内设备组合与仲裁 |
| 3 | ROS 2 + Zenoh（rmw_zenoh / zenoh-bridge-ros2dds） | ROS 2 / Eclipse Zenoh | 数据面多机器人路由（与 R2C 同传输栈） |
| 4 | LeRobot | Hugging Face | SDK 配置驱动的多设备组合（R2C 已有 lerobot 适配器） |
| 5 | Google Cloud Robotics | Google 开源架构（社区维护） | 云端多机器人 fleet |
| 6 | Unitree SDK2 | 宇树（厂商 SDK） | DDS 数据面多传感器 |
| 7 | ABB MultiMove | ABB（工业） | 工业多本体协同 |
| 8 | Azure IoT Plug and Play / AWS Greengrass | 云厂商 IoT 标准 | IoT 业界多组件设备管理标准 |
| 9 | Genie Studio | 智元机器人 AgiBot（重点调研） | 云端一站式具身智能平台 |
| 10 | 灵渠 Link-U OS | 智元机器人 AgiBot（重点调研） | 机器人端操作系统（开源） |

> 名字考证：Genie Studio 与灵渠 Link-U OS 同属智元机器人（AgiBot）——前者是云端"数据→训练→仿真→部署"平台，后者是机器人端操作系统（开源组织 `Link-U-OS`，经本次调研直接核验），二者构成"云平台 + 端侧 OS"组合。

## 横向对比

| 实现 | 设备建模 | 多设备组合 | 数据面复用 | 生命周期/健康 | 控制仲裁 | 云端管理 |
|------|----------|------------|------------|----------------|----------|----------|
| Viam | resource = API + model 三元组 + 全局唯一 name（component/service 两类） | sub-part / remote part 并入统一资源图；fragments 复用 | 按类型单服务多实例（gRPC + WebRTC P2P）；观测入单表 readings，六级元数据 | 每 part 进程隔离；session 心跳失联自动 `Stop()`；配置热加载 | 连接权即控制权；motion service 分解跨 part 协同 | org/location/machine + RBAC；注册表版本化 OTA（tag 灰度） |
| ros2_control | system/sensor/actuator 组件；接口全名 `<设备>/<信号>` + 类型/上下限 | URDF 多 `<ros2_control>` 块并入同一 controller_manager | 共享内存接口存储；对外默认合并向量、可参数化拆分/多实例 | 每组件生命周期状态机 + on_error；错误传播组；HardwareStatus 消息 | 命令接口独占 + switch_controllers + fallback | 不涉及（本地活动话题） |
| ROS 2 + Zenoh | KE = `<domain>/<实体>/<类型>/<hash>`；liveliness token 即上下线 | 每 robot 一个 bridge/实例 + namespace 前缀 | 每 topic 一 KE；regex 策略面（频率/优先级/RELIABLE Block/Drop/缓存） | liveliness 上下线事件；admin space 查询全部节点与路由 | 粗粒度 allow/deny 方向性 | admin space + REST；无云端注册 |
| LeRobot | RobotConfig 注册表（type + id + calibration_dir） | 嵌套设备 config 组合（`bi_so_follower` = 左臂 + 右臂 + 相机字典） | 扁平 dict + `left_`/`right_` 前缀；顶层相机键不加前缀 | 逐设备 setup_motors/calibrate，按设备校准目录 | leader→follower 配置映射（teleoperate） | 不涉及 |
| Google Cloud Robotics | Robot CRD（spec + cloud/robot 分区 status）+ RobotType | App/AppRollout：label selector 圈机器人 + per-robot values | spec 下行 / status 上行（cr-syncer 定向同步）+ http-relay 拉模式 | CR 收敛语义（observedGeneration），异步删除容忍离线 | 声明式 CR 定向单机器人 | Token Vendor/KMS 设备注册表；App 换版本灰度 |
| Unitree SDK2 | 扁平 DDS 消息（LowCmd/LowState 含全部关节+IMU） | 单机单 SDK 实例；多机靠 domain id / 网卡 | 按控制层级分 topic（`rt/lowcmd`、`rt/lowstate`、`rt/sportmodestate`、`rt/arm_sdk`），非按设备 | 不涉及（由厂商服务面承担） | 低层/高层控制分 topic 并存 | 不涉及 |
| ABB MultiMove | 控制器系统内的多机构组 | 一个控制器系统协调多本体 | 不涉及（实时总线内部） | 控制器层安全与同步 | 控制器内任务级协同（协调运动） | RobotStudio 配置 |
| Azure DTDL / AWS Greengrass | 设备 = DTDL 模型，多组件复用接口 | 组件化模型（如 thermostat1/2）；Greengrass 软件组件 | telemetry/属性/命令按 component 组织；组件打包部署 | 期望状态/影子；按组部署 | 不涉及 | 模型注册表 / registry；fleet 分组部署 |
| Genie Studio | 按本体/末端两个正交维度的任务模型 | 模板任务 + 多设备异步采集 | 分布式数据管道；传输层组织未公开 | Monitor 多机运维；Worken 有效性指标（剔除宕机/遥操作） | 未公开（有百台级群体验证） | 吞吐/配额按 device 计量；GDK 部署 |
| 灵渠 Link-U OS | AimRT 节点/模块 + aimrt_sensor HAL | Bazel 工程 + YAML + AimStudio 部署 | ROS 2 兼容 Topic/RPC；`<类型>/<安装位>/<信号>` 一路一 topic | aimrt_health_monitor（心跳/急停/系统状态清单） | RL 运控框架；多末端仲裁未公开 | AimStudio 多机运维；数据闭环在云侧 Genie Studio |

## 各实现详解

### 1. Viam（商业端到端平台：组件模型 + 多 part + 云端 fleet）

一句话：云端三层组织（org/location/machine）+ 机内资源模型（component/service = API + model + 全局唯一 name）+ 多 part 组合（sub-part / remote part）+ 观测数据单表 + fleet OTA，是本次调研中"多本体 / 多末端 / 多传感器"语义最完整的端到端参考。

**设备建模**：云端 org → location（可嵌套 3 层）→ machine 层级，权限沿层级继承。machine = 计算资源 + 硬件的逻辑整体，至少一个 part（part = 运行 `viam-server` 的计算机，由 `viam-agent` systemd 服务安装并监督）。一切皆 resource，分 component（arm/gripper/camera/sensor/board/motor…）与 service（motion/vision/data…）两类；每个资源 = API + model：API 类型 `rdk:component:arm` 等，model 为三元组 `namespace:family:name`（如 `viam:ufactory:xArm6`）；name 全机唯一，代码/配置/数据全按 name 引用，多相机/多臂 = 多个不同 name 的配置块。
来源：docs.viam.com `organization/overview`、`overview`、`hardware/configure-hardware`。

**多设备组合**：云端 JSON 配置为 source of truth，机器拉取收敛、保存即热加载（块字段 `name/api/model/attributes/depends_on/frame`）。一台 machine 用多台计算机的两种模式：① sub-part——云端配置、主 part 可见子 part（反向不可见）、必属同一 machine/org，文档明确动机含故障隔离（"急停臂不应砍掉末端相机"）；② remote part——自配地址 + API key、方向可选、可跨 org、可被任意多台 machine 借用（如 3 台移动底盘共用一台仓库相机）。remote 资源并入本机资源图、调用与本地资源完全透明；可用 `prefix` 重命名防冲突，重名裁决文档化（本地胜出远端隐藏；两个远端重名则都隐藏）。fragments 片段复用（per-machine 变量、版本 tag 灰度、overrides）；remote 组件可用 `frame` 挂入本机 frame tree 表达跨机空间关系。
来源：docs.viam.com `hardware/machine-configuration`、`hardware/multi-machine/overview`、`hardware/multi-machine/add-a-remote-part`、`fleet/overview`。

**数据面复用**：每组件/服务类型是一条 protobuf 服务，gRPC（TLS HTTP/2）承载；WebRTC data channel 做 part↔part、SDK↔part 的 P2P，云端只做连接信令、不中转业务包；不可用时退化为直连 gRPC。多路复用方式："配置两只臂，也只有一条 Arm 服务处理所有臂的 RPC"——按类型单服务多实例复用，而非每设备独立通道。无按设备带宽/QoS。观测数据：data capture 按"资源 × 方法"采集（本机落盘→云同步），云端统一单表 `readings`，行级元数据 = (organization_id, location_id, robot_id, part_id, component_type, component_name, method_name, 双时间戳, tags)，值在 `data` JSON 列；binary 数据（图像）单独通道 + tag——观测天然按六级标识组织，多本体/多末端/多传感器过滤零成本。
来源：docs.viam.com `reference/machine-to-machine-comms`、`data/schema`。

**生命周期与健康**：每 part 独立 viam-server 进程 = 故障隔离单元；remote part 掉线后台自动重连，其资源标 unreachable、调用报错，本地资源不受影响（不消失）。客户端 session 心跳：客户端失声（断网/崩溃）→ server 自动对该客户端最后命令的资源调 `Stop()`（防失控）。viam-agent 轮询云端（最短 5s）取配置/版本；配置卡 per-component 错误日志；保存配置自动 reload 初始化新组件（热插拔形态）。
来源：docs.viam.com `hardware/multi-machine/overview`、`reference/viam-agent`、`hardware/configure-hardware`。

**控制仲裁**：控制权 = 连接权（SDK 连 main part 即可操作全机资源），未见显式多客户端互斥；安全兜底 = session 心跳失联即自动 `Stop()`。跨组件/跨 part 协同由 motion service 承担：目标位姿 → frame system 跨 part 参考系变换 → 分解为对各组件的独立 Move（被控 part 不知整体计划）。
来源：docs.viam.com `hardware/multi-machine/overview`、`reference/machine-to-machine-comms`。

**云端管理**：org/location/machine + RBAC + API key/OAuth；零接触 provisioning（出厂预装 agent + defaults，首开机热点/蓝牙配网）；模块与 ML 模型为注册表版本化包，OTA = 版本 tag 变更（pin 版本、stable/rc/dev 通道、维护窗口、fragment tag 分批灰度）；viam-agent 更新 = 下载 → SHA-256 校验 → 原子 symlink 切换，失败不自动回滚；云端日志、CLI 远程 shell、monitor（dashboards/alerts/tracing/teleop）。
来源：docs.viam.com `organization/overview`、`fleet/overview`、`reference/viam-agent`、`cli/manage-your-fleet`、`monitor/overview`。

**对 R2C 的借鉴**：
1. sub-part（归属、云配、强管理）vs remote part（借用、自配、可跨域）双模式 + 远端资源并入统一资源图 + 断线"标 unreachable 不消失"，是"多本体/多末端组合"最贴近 R2C 的语义模型；重名裁决规则可直接搬用。
2. "类型 API + model 三元组 + 全局唯一 name"可映射为 Zenoh key 命名（`<类型>/<设备名>`），并保留依赖声明（depends_on）与 frame（空间关系）两个正交维度。
3. session 心跳 + 失联自动 `Stop()` + per-part 进程级隔离，可直接作为 R2C 控制通道的心跳/安全停机基线；云端"单表 readings + 六级行级元数据"值得数据面 schema 照搬。

### 2. ros2_control（ROS 2 官方控制框架）

机内"设备组合 + 控制仲裁"的参考实现：把硬件抽象为三类 hardware component，同一 `controller_manager` 聚合，动作用"接口独占"仲裁。

**设备建模**：硬件抽象为 System（多 DOF、可读可写、单逻辑通信通道）、Sensor（只读，可关联 joint 或 link）、Actuator（单 DOF、每电机独立通信）三类 hardware component。身份 = URDF 中 `<ros2_control name=".." type="system|sensor|actuator">` 块名；块内以 `<joint>`/`<sensor>`/`<gpio>` 分组，每项声明 `<command_interface>`/`<state_interface>`。接口全名为扁平命名 `<设备名>/<信号>`（如 `rrbot_joint1/position`、`rrbot_tcp_fts_sensor/force.x`），interface 名是任意字符串且可带数据类型（double/bool/uint16/int32…）与 min/max/initial_value 元数据。controller 侧另有 semantic components（IMUSensor、ForceTorqueSensor 等）把一组接口按设备语义批量 claim/release。
来源：control.ros.org `getting_started`、`hardware_interface/doc/writing_new_hardware_component`、`hardware_interface/doc/hardware_interface_types_userdoc`、`hardware_interface/doc/semantic_components`。

**多设备组合**：URDF 中并列多个 `<ros2_control>` 块（推荐 xacro 宏逐设备封装），全部装载进同一个 controller_manager；官方 demo `example_13` 用一个 CM 挂 4 个组件（3 个 system 机器人 + 1 个外置 sensor），控制器按设备前缀命名（`rrbot_*`/`threedofbot_*`）；更大的系统用多 CM + ROS namespace 或 robot_description remap 隔离（`example_15`）。组件参数写在 URDF `<param>`，控制器参数在 YAML 且支持 `*`/`**` 通配。
来源：control.ros.org `controller_manager/doc/userdoc`、`ros2_control_demos/example_13`、`ros2_control_demos/example_15`。

**数据面复用**：组件间无传输层，是 CM 实时循环内的共享内存存储，controller 按接口全名 claim。对外默认"合并向量"：`joint_state_broadcaster` 把所有 joint 合并成一条 `/joint_states`，可用 `joints`×`interfaces` 参数矩阵取子集、`use_local_topics` 拆到控制器自身命名空间、或多实例 + remap 拆成每设备 topic；传感器各有 broadcaster，每设备一 topic。每组件可设独立采样率（`rw_rate`）。无按设备的传输 QoS 配置面（由各 broadcaster 的 ROS publisher QoS 决定）。
来源：control.ros.org `joint_state_broadcaster/doc/userdoc`、`hardware_interface/doc/different_update_rates_userdoc`。

**生命周期与健康**：每组件状态机 UNCONFIGURED → INACTIVE（可读、命令不可用）→ ACTIVE → FINALIZED；read/write 返回 ERROR 触发 on_error，处理后回 UNCONFIGURED 或落 FINALIZED 不可恢复。故障隔离：组件 ERROR 自动停掉使用其接口的所有控制器；控制器 ERROR 停该控制器并尝试激活 fallback_controllers；可用 `<group>` 把同臂的多个组件编组、组内错误可传播，默认各组件互相隔离。无真正热插拔，用 `set_hardware_component_state` 服务手动拉起；组件可按 `status_publish_rate` 发布结构化 `HardwareStatus`（device_id + health_status）。
来源：control.ros.org `hardware_interface/doc/lifecycle_of_a_hardware_component`、`hardware_interface/doc/handling_errors_during_read_write`、CM userdoc、`ros2_control_demos/example_13`。

**控制仲裁**：命令接口独占——CM 对 required/provided 接口匹配，"授予访问或报访问冲突"；切换经 `switch_controllers`（strict/best_effort）；多模式硬件用 `prepare/perform_command_mode_switch` 协商。
来源：control.ros.org `getting_started`、CM userdoc、`ros2_control/doc/concepts`。

**云端管理**：不涉及（仅本地 `~/activity` 话题与 `ros2 control list_hardware_components` 观测）。

**对 R2C 的借鉴**：
1. 扁平接口命名 `<设备>/<信号>` + 类型/上下限/初值元数据随设备声明；一个管理器实例聚合多设备、共享存储、动作接口独占 claim 的仲裁模型。
2. 每设备生命周期状态机（unconfigured/inactive/active + on_error 语义）+ 启动初始态白名单 + 可选错误传播组 + 结构化 `HardwareStatus` 健康消息，可直接移植为多设备健康管理体系。
3. 观测面"默认合并、参数化拆分"（joints/interfaces/use_local_topics）+ 每设备独立采样率。

### 3. ROS 2 + Zenoh（rmw_zenoh / zenoh-bridge-ros2dds）

与 R2C 同传输栈（Zenoh）的多机器人数据面参考：topic → key expression 的映射、per-topic QoS/带宽策略面、liveliness 上下线事件。

**名称澄清**：官方 bridge 为 **zenoh-bridge-ros2dds**（`eclipse-zenoh/zenoh-plugin-ros2dds`，同一代码构建 router 插件与 standalone bridge）；`rmw_zenoh`（`ros2/rmw_zenoh`）与该 bridge 因 key expression 设计不同，官方明确互不互通。

**设备建模（key 设计）**：rmw_zenoh 无"设备"概念，身份 = ROS 实体。topic/service → key expression `<domain_id>/<fully_qualified_name>/<type_name>/<type_hash>`（domain_id 即 ROS_DOMAIN_ID，type_hash 防"同话题异类型"互通）；实体上下线用 liveliness token `@ros2_lv/<domain_id>/<session_id>/...`（`@` 起始块是 verbatim，通配符永不匹配，管理面与数据面天然隔离）。Zenoh 官方 key 命名建议：高方差段放右侧（`factory/12/room/10/robot/9` 而非 `factory-12/room-10/robot-9`），块内禁 `*`。
来源：github.com/ros2/rmw_zenoh `docs/design.md`；github.com/eclipse-zenoh/roadmap `rfcs/ALL/Key Expressions.md`。

**多设备/多机器人拓扑**：rmw_zenoh 每 ROS context 一个 Zenoh session（peer 模式）连本机 router `rmw_zenohd`，主机内数据走 P2P、跨主机靠 router 互联；UDP 组播发现默认禁用——官方理由是防止同一 LAN 内未配置的机器人互相干扰（应用 ROS_DOMAIN_ID 或 namespace 区分）。bridge 官方典型拓扑："每机器人一个 bridge + 监控端一个 bridge"（云端 `-e tcp/<robot-ip>:7447` 静态接入），硬性要求被桥接主机间 DDS 不可达，否则回环。云端聚合侧给每机器人 namespace 前缀（含 /tf、/rosout 全部接口），即天然按机器人分区。
来源：ros2/rmw_zenoh README + design.md；eclipse-zenoh/zenoh-plugin-ros2dds README。

**数据面复用**：每 topic 一条 key expression；publish=put（CDR 序列化 + attachment 携带 seq/源时间戳/publisher GID）；QoS 映射 RELIABLE/BEST_EFFORT/TRANSIENT_LOCAL（后者用 PublicationCache 支持迟到订阅者）。bridge 把"按设备控 QoS/带宽"做成 regex 策略面：allow/deny 黑白名单、`pub_max_frequencies`（如 `.*/laser_scan=5` 路由侧降采样）、`pub_priorities`（1–7 + `:express`）、RELIABLE 路由 Block/Drop、transient-local 缓存倍率。
来源：ros2/rmw_zenoh design.md；zenoh-plugin-ros2dds `DEFAULT_CONFIG.json5`。

**生命周期与健康**：每实体上线/销毁即声明/撤销 liveliness token，订阅该 token 即得到"设备上下线事件流"（替代心跳轮询）；bridge 侧 admin space `@/local/ros2/**` 与跨 bridge `@/*/ros2/**` 可实时查询所有机器人的节点与路由清单（支持 REST）。
来源：ros2/rmw_zenoh design.md；zenoh-plugin-ros2dds README。

**控制仲裁**：rmw_zenoh 不涉及；bridge 仅粗粒度方向性仲裁（allow/deny 决定哪些接口可被路由，如只放行 `.*/cmd_vel` 下行），无 per-client 权限体系。
来源：ros2/rmw_zenoh design.md；zenoh-plugin-ros2dds DEFAULT_CONFIG.json5。

**云端管理**：bridge 的 admin space + REST 即跨机器人注册/监控入口；rmw_zenoh 的 graph cache 仅本地。

**对 R2C 的借鉴**：
1. key 直接对标：`<隔离域>/<设备路径>/<信号>` + 类型标识；管理/控制面用 `@` verbatim 前缀与数据面隔离；名字内 `/` mangle。rmw_zenoh 官方明确"native Zenoh 应用按 design.md 的 KE/序列化/attachment/liveliness 设计即可与 ROS 2 节点互通"——是"云上 Zenoh SDK ↔ ROS 2 机器人"的现成互通契约。
2. per-device QoS/带宽策略面照抄 bridge 配置 schema：按设备/信号 regex 的 allow-deny、max_frequency 降采样、priority+express、RELIABLE Block/Drop。
3. 设备注册/健康用 liveliness token（上下线即事件）+ admin-space 风格查询面。单进程多设备在 key 层级区分（robot/device/signal），多机器人共享一云用"首段隔离域 + 每 robot 一个 edge 实例 + namespace 前缀聚合"。

### 4. LeRobot（Hugging Face，SDK 配置驱动组合）

一句话：以"注册表 + 嵌套 dataclass 配置"组合多设备机器人（R2C 的 lerobot 适配器与 `robot_so101_bimanual_config.yaml` 正是对该体系的封装），多臂/多相机用**前缀命名约定**扁平复用。以下标注均为调研当日直接核验官方仓库源码的结论。

**设备建模（核验 `src/lerobot/robots/config.py`）**：`RobotConfig` 是 draccus 的 ChoiceRegistry dataclass：`id` 字段区分同型不同号（"Allows to distinguish between different robots of the same type"）、`calibration_dir` 指定校准目录；`type` 属性 = 注册子类名；`__post_init__` 统一校验 `cameras` 字典（width/height/fps 必填）。机器人类型经 `@RobotConfig.register_subclass("bi_so_follower")` 注册；上游机器人族谱（核验目录）：`bi_openarm_follower`、`bi_rebot_b601_follower`、`bi_so_follower`（双臂）、`hope_jr`、`koch_follower`、`lekiwi`（底盘+臂）、`reachy2`、`so_follower`、`unitree_g1` 等。

**多设备组合（核验 `src/lerobot/robots/bi_so_follower/config_bi_so_follower.py`）**：`BiSOFollowerConfig = left_arm_config: SOFollowerConfig + right_arm_config: SOFollowerConfig + 顶层 cameras: dict[str, CameraConfig]`——嵌套的逐设备配置对象；源码注释明确："Top-level cameras not attached to a specific side. Keys are kept as-is in observations (no left_/right_ prefix). Per-arm cameras are prefixed."（顶层相机键不加前缀，每臂相机自动加 `left_`/`right_` 前缀）。

**数据面复用（核验 `bi_so_follower.py`）**：`get_observation()` 合并两臂观测并加 `left_`/`right_` 前缀（顶层相机键保持原样）；`send_action()` 按 `removeprefix("left_"/"right_")` 把动作路由进各臂，再把前缀加回返回值——多臂复用 = **扁平 dict + 前缀命名约定**，逐设备路由按前缀完成，无独立通道。

**生命周期与校准**：逐设备流程脚本（核验路径）：`lerobot_setup_motors.py`（逐电机注册）、`lerobot_calibrate.py`（按设备校准）、`lerobot_teleoperate.py`（主从遥操作）、`lerobot_record.py`/`lerobot_replay.py`（数据采集与回放）；`teleoperators/` 目录把"主臂"也建模为独立设备类型，teleoperate 配置把 leader 键映射到 follower 键。

**对 R2C 的借鉴**：
1. 配置即组合：设备列表 = 嵌套配置对象 + 类型注册表，设备实例天然带 `id` 与 `calibration_dir`——R2C 的 `robot_config.yaml` 从单 `hardware` 块升级为多设备块可直接沿用此形态。
2. 观测/动作键的前缀分区规范（设备内键不加前缀、跨设备合并时加 `<设备>_` 前缀）与现有 `device_to_r2c` 的 `source_paths` 拼接思路一致，可作为多设备 schema 的正式约定。
3. 主臂/从臂都是"设备"，用同一套配置/校准/遥操作抽象——多末端与多本体在 SDK 层是同一件事。

来源：github.com/huggingface/lerobot（`src/lerobot/robots/config.py`、`src/lerobot/robots/bi_so_follower/config_bi_so_follower.py`、`bi_so_follower.py`、`scripts/lerobot_*.py`，本次核验）。

### 5. Google Cloud Robotics Core（云端多机器人 fleet，社区维护）

一句话：机器人 = 单节点 Kubernetes 集群，云端 = 控制集群；"spec 下行 / status 上行"声明式联邦 + 拉模式 relay；设备注册表 = CRD。缺组件级建模（多末端/多传感器），适合借其云端协议形态而非设备模型。

**现状**：未归档、活跃维护（2019-02 首提交，调研当日仍有维护提交，1869 commits，近期提交者仍为 @google.com 账号）；README 自述 open source、pre-alpha、仅对少量 early access 伙伴提供支持、无稳定性承诺。⚠️ 早年博客流传的 `robot-devkit`/`robot-web-services`/`ros-bridge`/`scheduler` 组件名，经完整 git 历史核查从未存在于 core 仓库（`apps-cloudbuild` 仓库已 404）——引用该项目时勿再用旧名。

**设备建模**：机器人 = 以唯一 robot name 标识的单节点 K8s 集群（推荐 k3s/kubeadm）；一个云集群 + N 个机器人集群。注册：`setup-robot` 生成 RSA 密钥对存 K8s secret，公钥经 Token Vendor 写入云端（K8s 充当设备注册表/KMS），云端创建 Robot CR。设备 CRD `robots.registry.cloudrobotics.com/v1alpha1`：spec（type/project/displayName）+ status 拆 `cloud`/`robot` 两子树（注解 `status-subtree: robot` 让机器人只写自己的 status）；`RobotType`（make/model）。无组件级类型系统（交给 Helm 与应用自定义 CRD）。
来源：googlecloudrobotics.github.io/core `overview`、`concepts/device_identity`；github.com/googlecloudrobotics/core `src/go/cmd/setup-robot/main.go`、`src/app_charts/base/cloud/registry-crd.yaml`。

**多设备组合**：组合单位是"App"而非组件：`App` CR = cloud/robot 两份 Helm chart；`AppRollout` CR 按 label selector 圈机器人（如 `model: mir100`）并注入 per-robot Helm values（可 per-target chart 版本做 canary）；app-rollout-controller 生成 `ChartAssignment` 下发安装，synk 修正漂移。
来源：googlecloudrobotics.github.io/core `concepts/app-management`。

**数据面复用**：命令/状态面是声明式联邦：cr-syncer（每机器人集群一个）按 CRD 注解同步——`spec-source: cloud|robot` 定 spec 所有方、status 反向回流；资源要么同步全部机器人、要么按 `cloudrobotics.com/robot-name` label 定向唯一机器人；机器人间不直连；重试/resync 抗间歇连接、删除异步传播（容忍长期离线）。云→机器人访问：http-relay（云端多路复用 server + 机器人侧主动拉取序列化请求的 client，机器人无需公网入站端点；支持 101 升级与 `X-Server-Name` 选 gRPC 后端）。应用→云：robot service-account 短时 OAuth token + gRPC（BeyondCorp 零信任、无 VPN）。ROS 侧仅演示 ROS node 作为机器人集群内 K8s Deployment；无 topic 级多路复用/QoS 层。
来源：googlecloudrobotics.github.io/core `concepts/federation`；core 仓库 `src/go/cmd/http-relay-server/README.md`、`docs/how-to/running-ros-node.md`。

**生命周期与健康**：spec/status 最终一致 + `observedGeneration` 判收敛；chart-assignment-controller 检查 rollout 内 Pods Running 才置 Ready；AppRollout 汇聚各 ChartAssignment 状态；按 per-robot 凭据逐连接吊销。无设备级心跳原语（状态经 CR status 回流）。
来源：googlecloudrobotics.github.io/core `concepts/federation`、`concepts/app-management`。

**控制仲裁**：无内建仲裁/协同；控制 = 声明式 CR（如 Mission CRD：charge/get_trolley/move_to_named_position 动作序列，`spec-source: cloud` + filter-by-robot-name 定向单机器人），多本体协同交给应用层。
来源：core 仓库 `src/app_charts/mission-crd/mission_crd.yaml`。

**云端管理**：云集群组件 token-vendor、app-rollout-controller、chart-assignment-controller、cr-syncer、k8s-relay、oauth2-proxy 等；监控/日志以可选 Apps 安装（Prometheus/Grafana/VictoriaMetrics/fluent-bit）；OTA = App/AppRollout 换 chart 版本（selector 灰度），无整机 agent 二进制版本管理。
来源：googlecloudrobotics.github.io/core `overview`、`concepts/config`；core 仓库目录结构。

**对 R2C 的借鉴**：
1. "spec 单作者下行、status 单作者上行 + per-robot 定向 + 异步删除容忍离线"可映射为 Zenoh per-robot key 空间的配置下发/状态回流协议，天然多机器人隔离。
2. 拉模式 relay（端侧出站、云端无入站依赖）与 Zenoh peer/client 拓扑兼容——跨站点连接保留"云侧中继 + 端侧出站"形态。
3. 设备注册表 = 密钥对 + 元数据 CR（cloud/robot 分区 status）是 R2C 云端设备注册/状态库的好模板；组件级建模需取 Viam 的 API+model+name 体系补足。

### 6. Unitree SDK2（宇树，DDS 数据面）

一句话：厂商 SDK 的数据面按**控制层级**分 topic（低层关节环 / 高层运动 / 手臂 SDK），一条扁平 DDS 消息承载整机全部关节 + IMU，多机隔离靠 DDS domain id / 网卡选择。以下标注均为调研当日直接核验官方仓库源码。

**设备建模（核验源码）**：无设备级抽象，身份 = topic。`ChannelFactory::Instance()->Init(0, argv[1])`——第一个参数是 DDS domain id、第二个是网卡接口（`example/go2/go2_low_level.cpp`，本次核验），多机隔离 = 不同 domain id 或指定网卡。

**数据面复用（核验源码）**：`#define TOPIC_LOWCMD "rt/lowcmd"`、`TOPIC_LOWSTATE "rt/lowstate"`（`go2_low_level.cpp`）；G1 侧 `rt/arm_sdk`（`include/unitree/dds_wrapper/robots/g1/g1_pub.h`）；运动层状态单独 topic（`example/{go2,a2,as2}/sport/*_sport_state.cpp` 等，核验路径）。消息为扁平 DDS 结构（`unitree_go::msg::dds_::LowCmd_`/`LowState_`）：整机所有关节 + IMU 在一条 `rt/lowstate` 里，**按控制层级分 topic，不按设备分 topic**。

**对 R2C 的借鉴**：
1. "控制层级分通道、消息内聚整机状态"与"按设备拆通道"是两种正交选择——R2C 的多传感器高带宽流（图像/点云）适合按设备拆 key，低频状态适合合并消息；Unitree 代表了前者的"层级"变体。
2. DDS domain id ≈ r2c topic 首段隔离域（project_id），厂商 SDK 不做云端管理面——云端纳管正是 R2C 相对厂商 SDK 的增量价值。

来源：github.com/unitreerobotics/unitree_sdk2（`example/go2/go2_low_level.cpp`、`include/unitree/dds_wrapper/robots/g1/g1_pub.h`、`example/*/*/` 目录，本次核验）；官方文档入口 support.unitree.com/home/zh/developer。

### 7. ABB MultiMove（工业多本体协同）

一句话：工业界的"多本体"由控制器层解决——一个控制器系统协调多台本体，数据面留在实时总线内部；云端数据面只需表达"多控制器单元 + 各自状态"。

**核验事实**：ABB 官方文档库存在《Application manual - MultiMove (IRC5)》（文档号 3HAC050961-001，本次经 search.abb.com 核实标题），MultiMove 是 IRC5 时代的官方多机器人协同应用，RobotWare 选件形态延续。机制要点（多机器人经任务级同步做协调运动、一次同步启动等）依据官方手册定位归纳，未逐页核验，细节请以手册为准。

**对 R2C 的借鉴**：
1. 工业界的参照系：多本体协同的仲裁与同步放在控制器层（单一权威、任务级原语），云端不做实时仲裁——R2C 云侧应保持"每 device_id 一个权威执行体"，云端只做任务编排与状态汇聚。
2. "协调运动"这类强耦合多本体场景，业界没有把它放进云端数据面协议，而是留给端侧控制器——r2c 的多本体范围宜限定为"多执行体管理 + 数据/动作通道复用"，不承诺云端实时协同。

来源：search.abb.com/library/ABBLibrary.asp?DocumentID=3HAC050961-001（标题核验）。

### 8. IoT 业界的多组件设备管理标准（Azure DTDL / AWS Greengrass）

一句话：云厂商把"一台设备里的多组件"与"fleet 内的多设备"统一成声明式模型 + 版本化组件 + 按组部署，云端纳管与端侧运行彻底解耦。以下为调研当日直接抓取官方文档。

**Azure IoT Plug and Play（DTDL）**：设备 = DTDL 模型（Interface，DTMI 标识）；多组件 = 模型里多个 `@type: Component` 块引用可复用接口，官方示例 TemperatureController 由 `thermostat1`/`thermostat2` 两个 Thermostat 组件 + DeviceInformation 组件组成（telemetry/property/command 随组件声明）；无组件的单接口模型也可直接迁为即插即用设备（`learn.microsoft.com/azure/iot-pnp/concepts-modeling-guide`，本次核验）。telemetry 在云端按组件命名空间组织（组件内遥测的键名约定见官方文档）。

**AWS IoT Greengrass v2**：边缘运行时 + 可复用"组件"（Lambda / Docker / 原生进程 / 自定义运行时），组件开发 → 打包 → 部署到设备/fleet（`docs.aws.amazon.com/greengrass/v2/developerguide/what-is-iot-greengrass.html`，本次核验）；V1 已于 2026-10-07 结束支持。

**对 R2C 的借鉴**：
1. "设备 = 声明式模型，多设备 = 组件复用接口 + 名称实例化"——R2C 的多设备 schema 可借鉴 DTDL 的"接口复用 + 组件实例命名"，避免每加一台传感器就发明一套新字段。
2. 版本化组件 + 按组部署是 IoT 业界 OTA 的标准形态，可与 Viam/GCR 的注册表灰度互证。

来源：learn.microsoft.com/azure/iot-pnp/concepts-modeling-guide；docs.aws.amazon.com/greengrass/v2/developerguide/what-is-iot-greengrass.html（本次核验）。

### 9. Genie Studio（智元机器人 AgiBot，重点调研）

一句话：智元的云端一站式具身智能平台（1.0 2025-04 → Agent 2026-04 → 2.0 2026-10-23），2.0 由 Flywheel（数据闭环）/ Agent（零代码编排）/ Monitor（多机运维）三模块组成；与端侧灵渠 OS 构成"云平台 + 端 OS"组合。**观测/动作在传输层如何按设备组织与传输栈：公开资料未涉及。**

**设备建模与"多本体/多末端"**：无公开的组件树/设备类型系统。数据采集模块的一手表述（官方产品页）：模板化任务配置 + 全流程 dashboard；"cross-scenario and cross-device data collection management"（跨场景、跨设备采集管理）；**"Multi-Ontology and Multi-End Effector Data Collection and Validation"——支持多种机器人本体与末端执行器的异步（asynchronous）数据采集 + 可定制数据校验**；吞吐按设备计量（up to 1,000 per device daily）。这是本次调研对象中与"多本体 + 多末端"最直接对齐的一手材料。

**组合配置**：云侧以"模板任务 + dashboard"组合多设备采集；应用侧 Agent 模块把 VLA/RL/视觉感知/运动控制/导航"SDK 化、模块化"，拖拽节点编排工作流（二手转述；另有技能组件仓库、BPMN 编程式编排等说法）。配置文件格式未公开。

**数据面**：官方称分布式数据管道（清洗、异常检测、实时/离线双模处理）；采集侧确认不同本体/末端异步并行采集，数据按任务/设备维度入湖，下游为 AgiBot World 数据集。传输层组织与传输栈未公开。

**生命周期与健康**：Monitor 模块定位"多机运维"（官方口径）；Genie Studio Agent 提供"全链路监控运维，实时掌握设备状态，从被动维修转向主动预警式管理"（二手）；部署模块列版本管理、远程诊断、实时监控、有效性评估、性能分析（官方页）。心跳/在线判定/故障隔离/标定流程：公开资料未涉及。平台级健康度量 Worken（2026 创新日）：有效工作量 = 活跃机器人数量 × 自主工作时长 × 产出效能比，剔除宕机、遥操作与无效时段（二手）。

**控制仲裁与协同**：多本体协同的公开证据是群体性验证：6 台 G2 在真实便利店 24 小时四班次无人接管（2026-11）；"群体经验学习 CEL"自称首个在百台级真机集群完成原型验证的机器人学习范式（二手转述）。多末端控制权分配/主从仲裁/安全：公开资料未涉及。

**云端管理**：训练侧 GO 系列基座模型（GO-2、GO-3 Preview/WRAM）+ 训练/微调/量化/部署流水线 + Genie Evolver 后训练（真机/仿真/世界模型三源，二手）；仿真评测 8000+ 资产、100+ 自动评测任务、宣称仿真-真机误差 <5%（官方页）；部署侧 GDK 真机控制系统 + 低代码开发 + 应用发布、模型加密（官方页）。未见 OTA 表述，最接近的是"版本管理 + 远程诊断 + 多平台发布"。

**对 R2C 的借鉴**：
1. 按"本体/末端"两个正交维度建采集任务模型（Multi-Ontology + Multi-End Effector），异构设备允许异步并行采集、校验规则按任务定制——R2C 可在数据面之上加"采集任务/校验规则"抽象。
2. 吞吐与配额按 device 计量并配全链路 dashboard——云端把"设备"作为配额、状态、质量指标的基本计量单元。
3. 把"多机运维"做成独立模块（Monitor），健康度用"主动预警 + Worken 式有效性指标（剔除宕机/遥操作时段）"，而非简单心跳表。

来源：genie.agibot.com/geniestudio（官方页，一手）；官方公众号"智元AGIBOT"相关文（微信检索确认标题与要点，一手口径二手转述）；二手行业稿：CSDN blog.csdn.net/agibot_world/article/details/148088127、weixin_53855915/160154063、weixin_26950847/169498603、zabenbrdit36243qnjx1/168656195、gzzn2019/169845606、shaobingj126/169455598。

### 10. 灵渠 Link-U OS（智元机器人 AgiBot，重点调研）

一句话：智元的开源具身智能操作系统（WAIC 2025 开源计划 → 2026-03 Alpha 开源 → 2026-10-29 1.0 发布，宣布年底全面开源 2 亿行代码/5000+ 软件包并捐赠开放原子开源基金会），底座为自研中间件 AimRT；**端侧数据面薄（ROS 2 兼容的 Topic/RPC + 一路一 topic 的传感器命名），数据闭环上移云侧 Genie Studio**。以下带"本次核验"标注的内容均为调研当日直接抓取 GitHub 一手 README 的结论。

**考证**：开源组织 github.com/Link-U-OS（org 存在、16 个公开仓库，本次核验）；主线仓库 README 自述：开源具身智能 OS，"南向适配具身硬件、北向支撑智能应用"，底层提供分布式实时通信与硬件抽象等基础服务框架，上层开放智能体服务框架，工具链覆盖开发调试/仿真验证/部署管理/数据记录。

**设备建模（本次核验）**：
- 中间件底座 **AimRT**（独立组织 AimRT/AimRT，1420 stars，C++，本次核验存在）：核心抽象 Node / Protocol / Channel / RPC / Module / App 模式与 Pkg 模式 / Executor / Plugin / 配置管理器（抽象清单为二手解析），双阶段运行模型、"逻辑实现与部署运行分离"（二手）。
- 硬件抽象层 **`aimrt_sensor`**（本次核验 README）：传感器驱动模块，负责初始化、参数配置、数据采集，并以 ROS 标准消息格式发布；含 4 个子模块：hal_lidar（激光雷达）、hal_d415（头部深度相机）、hal_dcw2（腰部深度相机）、tz_camera（3 个胸部相机）。**设备寻址 = ROS 2 topic 层级命名**：`/aima/hal/<设备类型>/<安装位>/<信号>`，如 `/aima/hal/lidar/neck/pointcloud`、`/aima/hal/rgbd_camera/head_front/color|depth`、`/aima/hal/rgbd_camera/waist_front/color|depth`、`/aima/hal/fish_eye_camera/chest_left|chest_right/color`、`/aima/hal/camera/interactive/color`——一台机器人上十余路传感器一路一 topic，以"类型 + 安装位"命名空间区分，不做合并向量。
- 健康监控 **`aimrt_health_monitor`**（本次核验 README）：battery、cpu（含温度）、disk、gpu、**emergency_stop（急停状态）**、**heartbeat（节点心跳）**、mainboard_temp、memory、network、process、system_status——节点级心跳与急停进入统一监控面。

**组合与配置**：Bazel 构建 + `integration` 仓库（第三方依赖管理、版本一致性、多芯片工具链、x86_64→aarch64 一键交叉编译，本次核验）；AimRT YAML 配置 + 模块加载（二手实操文）；**AimStudio**（本次核验 README）：远征系列机器人的远程运维 GUI——多台机器人连接管理、批量部署软件包与固件更新、健康状态与告警、进程状态与心跳查看、应用模块一键启停、日志导出与故障诊断、参数/配置部署、备份恢复。

**数据面**：统一通信中间件基于 AimRT，支持 Protobuf 与 ROS 2 Message 双格式、RPC/Topic 双模式、兼容原生 ROS 2 生态（主 README，本次核验）；1.0 宣称 1000Hz 高频负载下端到端时延抖动约 4µs 的确定性控制（二手转述）；传感器发布见 `aimrt_sensor` 的 topic 命名（本次核验）。

**生命周期与健康**：`aimrt_health_monitor` 监控清单 + AimStudio 的心跳/进程/告警运维（均本次核验）；故障隔离的工程基础是 AimRT Pkg 模块化部署（推断，非官宣）。

**控制仲裁与协同**：Alpha 附带基于 RL 的双足运控一站式框架（MuJoCo/Isaac Gym 训练 → Sim2Sim/Sim2Real 一体化部署，主 README 本次核验）；多末端/多本体控制权分配、主从仲裁：公开资料未涉及。

**云端管理**：端侧 OS 不含云端；云端闭环由 Genie Studio（GDK 真机控制系统）与 org 内 `rl_training`/`rl_deploy` 配套仓库承接（org 仓库清单本次核验）。

**对 R2C 的借鉴**：
1. OS/数据面做薄：统一通信（Protobuf 与 ROS 2 Message 双格式兼容 ROS 2 生态）+ 硬件抽象（`<设备类型>/<安装位>/<信号>` 的 topic 命名层级），设备建模、校验与数据闭环上移云平台/工具链。
2. 节点级 heartbeat + emergency_stop 纳入统一健康监控面——`aimrt_health_monitor` 的监控清单可直接对照扩展 r2c `Heartbeat`（当前仅整机 status/mode/error_code/battery）。
3. "多机运维"在量产场景是独立刚需工具（AimStudio：多机连接、批量部署/固件、告警、日志）——r2c 多设备版本应预留等价运维面。
4. 确定性指标口径（1000Hz/4µs 抖动）可作为 r2c 动作下发链路的对标测量项。

来源：github.com/Link-U-OS（Link-U-OS、aimrt_sensor、aimrt_health_monitor、AimStudio、integration 各 README，本次直接核验）；github.com/AimRT/AimRT（本次核验）；二手：CSDN 转载（149705526、149732681、158660273、144011440、169455598 等）与公众号转述。

## 对 R2C 多设备设计的启示

综合十个实现，可提炼为六个设计维度（括号内为业界依据）：

1. **命名与寻址：本体在 key、设备在消息、信号可拆 key。**
   - 本体级隔离用 topic 首段（现有 `{project_id}/{device_id}` 即业界通行做法；rmw_zenoh 的 domain 段、GCR 的 per-robot label、Zenoh KE "高方差段靠右" 均同构）。
   - 设备级身份放消息内：把 `Actions.end_effector_poses` 已有的 `map<string, EndEffectorPoseChunk>` 模式推广为通用 per-device 分组（多臂、多传感器各自 map/分组，设备键全局唯一——Viam name、ros2_control 接口全名、LeRobot 前缀约定同构）。
   - 高带宽信号（多相机/点云）保留按设备拆 key 的能力以便独立控频控带宽（Link-U OS 一路一 topic、zenoh-bridge-ros2dds 的 per-topic 策略面），低频状态合并成一条消息（ros2_control 合并向量、Unitree LowState 内聚）。
2. **消息模型：分组 + 自描述扩展，不发明新标量位。**
   - 业界共同点：设备用"名称 + 类型"声明（Viam API+model、DTDL 接口复用、ros2_control 接口元数据），数据用 map/分组承载；R2C 的 `extensions` map（自描述 ExtensionValue）已具备承载非标设备数据的条件，IMUState 等已定义但未挂接的消息可作为第一类"可插拔传感器组"接入。
3. **组合配置：从单 hardware 块升级为设备列表。**
   - 形态参照 LeRobot 嵌套 config + 注册表（R2C 已在用其体系）、Viam 多 name 块 + depends_on、ros2_control URDF 多块、AimRT YAML 模块化；每设备建议包含：类型 + 实现（adapter/model）、参数、采样率、依赖、校准目录、生命周期初始态（ros2_control 的初始态白名单值得直接采纳）。
4. **生命周期：设备级状态机 + 结构化健康消息 + 上下线事件。**
   - ros2_control 组件状态机与 HardwareStatus、aimrt_health_monitor 监控清单（heartbeat/emergency_stop/system_status）、Viam session 心跳失联自动 `Stop()`、rmw_zenoh liveliness 上下线事件——r2c 的 `Heartbeat` 可从整机级扩展为"整机 + 每设备"两层，控制通道复用"失联即停"语义。
5. **控制仲裁：接口独占 + 按设备路由，云端不做实时协同。**
   - 单本体内多末端：动作消息按设备键路由（现 end_effector map 模式）+ 可选接口独占仲裁（ros2_control）；多本体：保持每 device_id 独立执行体，云端只做任务定向（GCR 定向同步）与编排，不承诺云侧实时协调（ABB MultiMove 佐证协同在控制器层）。
6. **云端管理：设备注册表 + 按设备计量 + 独立运维面。**
   - GCR 的设备注册表（密钥对 + 元数据 + cloud/robot 分区 status）、Genie Studio 的按 device 计量配额、Viam 的单表 readings 六级元数据、AimStudio/Genie Monitor 的独立"多机运维"模块——建议 R2C 云端侧把"设备"作为注册、配额、观测数据、健康度四类管理对象的最小单元。

## 补充调研：设备树与派生能力抽象（六维力 2 秒历史示例）

背景：目标形态是 Linux devicetree 式的设备树。驱动示例：云上模型同时需要"当前六维力"与"2 秒内六维力历史"；若没有嵌套/能力抽象，每个力传感器都要适配"有历史 / 无历史"两种变体。追加约束（2026-11-04）：云端模型调用频率 0.5Hz，远低于采样/历史要求的 30Hz×2s（见第 3 节）；设备树已确认为本版本多设备管理的核心特性（见第 4 节）。本节聚焦两个问题：① 业界的设备层级与引用关系怎么建；② "历史 / 滤波 / 坐标变换"这类派生能力业界怎么抽象。**核心结论：业界没有把"带历史"做成设备变体的先例——历史一律是设备之外的通用能力**，落点有三种：传输层环形缓冲、树上包装/派生节点、云端时序窗口。

### 1. 设备树与引用关系（业界的"树"长什么样）

- **Linux devicetree**（官方规范 chapter2，本次核验）：树结构描述系统硬件，"A devicetree is a tree data structure with nodes that describe the devices in a system. Each node has exactly one parent except for the root node"；跨节点引用用 **phandle**（目标节点持唯一 `<u32>`，引用方在属性里写该值）——组合靠"引用"而非"内嵌"。
- **Viam**：资源块 `depends_on` 字段声明依赖 + remote part 并入统一资源图 + `frame` 挂接空间关系（见第 1 节）——层级与引用双轨。
- **YARP**（robotology/yarp `src/devices`，本次核验目录）：`multipleanalogsensorsremapper`（把多个模拟传感器设备聚合重映射为一个设备）、`controlBoardRemapper`、`frameTransformClient`（跨设备坐标变换本身也是树上的一个设备节点）——"多个同类设备聚合成一个虚拟设备"是 YARP 的原生组合原语。
- **ros2_control**：接口全名 `<设备>/<信号>` + controller 链（见模式 B）；**Link-U OS**：topic 层级 `<设备类型>/<安装位>/<信号>`（见第 10 节）。
- 共性：**树负责"是谁、挂在哪"，引用（phandle / depends_on / 接口名 / topic 名）负责"用谁"**。r2c 若引入设备树，建议"浅层级 + 引用"双轨，避免深树内嵌导致配置脆弱。

### 2. 派生能力（历史 / 滤波 / 变换）的三种业界落点

**模式 A：传输层环形缓冲——历史是传输原语，设备零适配**

- **Linux IIO 子系统**（kernel.org `driver-api/iio/buffers.html`，本次核验）：任意 IIO 设备都可挂 **buffer**——`length`（样本容量）、`enable`（激活采集）、`scan_elements/`（选择入缓冲的通道），由 trigger 触发采集。历史/缓存是设备之外的通用子系统，不是某个传感器的驱动变体。
- **DDS HistoryQosPolicy**（eProsima FastDDS API 文档，本次核验）：`KEEP_LAST_HISTORY_QOS` + `depth`（默认 1，可设 N）——DataWriter "on behalf of existing DataReader entities" 维护最近 N 个样本。历史是每 topic 的 QoS，不是设备代码。
- **Zenoh AdvancedPublisher + CacheConfig(max_samples=N)**（eclipse-zenoh/zenoh-python 官方示例 `z_advanced_pub.py`，本次核验）：publisher 侧声明 `cache=CacheConfig(max_samples=history)`，另有 `sample_miss_detection`（heartbeat 丢样检测）；订阅侧 `z_advanced_sub.py` 可查询历史样本，`z_storage.py` 提供存储/查询形态。**与 r2c 同传输栈："2 秒历史" = `max_samples ≈ 2s × publish_hz`，传感器适配只写一次。**
- 注意：DDS/Zenoh 的历史粒度是"样本数"而非"时间窗"——时间窗 = 样本数 × 周期，需按各设备 `publish_hz` 折算（IIO 的 `length` 同理）；时间戳必须随样本走（Zenoh attachment 携带 seq/时间戳的做法可借鉴，见第 3 节）。

**模式 B：树上的"包装 / 派生设备"节点**

- **YARP remapper 族**（见上）：聚合、重映射、坐标变换都以"设备"形态挂在树上。
- **ros2_control controller chaining**（admittance_controller userdoc，本次核验参数：`chainable_command_interfaces` 导出 reference 接口，`command_joints` "only relevant when chaining the output of this controller to the input of another controller"）：admittance 控制器消费 FTS state interface、把派生结果作为链上接口交给下游控制器——"力反馈 → 轨迹修正"是链上的控制器，而不是力传感器的第二种驱动。
- **Viam**：`depends_on` + service 包装组件产出派生数据；观测按"资源 × 方法"入单表 readings（第 1 节已核验）。
- 对 r2c 的映射：设备树允许"派生节点"类型（`history` / `filter` / `tf`），声明输入引用（phandle 式）+ 窗口参数，SDK 通用实现一次、所有输入设备复用。

**模式 C：云端时序窗口（历史在云端，不在端侧）**

- Viam 数据平台：所有 readings 入单表、带双时间戳与六级元数据（第 1 节已核验），历史是查询而非设备能力。
- r2c 的 data ingestion / 回流通道已是入湖路径：力数据按 `{project_id}/{device_id}` 入湖后，云模型的"2 秒窗口"由查询层（窗口聚合）提供；代价是时延与可用性依赖在线查询，不适合实时控制环，适合训练/回放类消费。

### 3. 频率不对称：采样 30Hz、云调用 0.5Hz

约束：采样与历史窗口要求 30Hz × 2s（60 样本滚动），而云端模型调用仅 0.5Hz（每 2 秒一次）。含义是**采样/历史维持与模型调用必须解耦**，两段频率各自设计：

- 端侧 30Hz：样本先进环形缓冲（60 样本滚动窗口），不逐条上云——逐条流式上云的总字节数与窗口批相当（历史样本一样多），但消息数是窗口批的 60 倍（30 条/秒 vs 0.5 条/秒），每条消息的协议开销、调度与云端处理成本被放大，还迫使云端常态消费并不需要的高频流。
- 云侧 0.5Hz：每次调用取一份"当前值 + 2s 窗口"快照（60 样本，按 6 维 double + 时间戳估算每条约 3KB 量级，带宽可忽略）。
- 两种实现形态（可并存）：
  1. **按需查询（缓存常驻端侧）**：Zenoh `CacheConfig(max_samples=60)` 常驻端侧，云调用时 Query 取窗——设备零改动，代价是每次调用多一个查询 RTT。
  2. **派生节点窗口批发布（设备树的核心收益）**：设备树的 `history` 派生节点 30Hz 写缓冲、0.5Hz 发布一条"窗口批"消息（当前值快照 + 60 样本 + 起止时间戳），云端纯订阅、无查询往返——**"频率适配"也被树上的派生节点抽象掉了，每个传感器依旧只适配一次**。
- 业界印证（均已核验）：IIO buffer 即"高频进缓冲、应用按需读"（第 2 节模式 A）；LeRobot record 全帧率录制、成批形成数据集（`lerobot_record.py`）；本仓库 `packages/cloudrobo-ingestion` 已是"端侧缓存、批量上传 + 进度回报"形态（仓库内先例）。
- 若不设派生节点、只靠云端查询：每设备一条 topic 的缓存声明即可起步，但"当前值 + 历史"的组装逻辑会落在云端每个模型上——这正是设备树派生节点（在端侧组装好窗口快照）要消灭的重复。

### 4. 对 r2c 的落地建议（按优先级；设备树为本版本核心特性）

0. 总原则：设备树（浅层级 + 引用 + 派生节点）作为本版本多设备管理的核心特性建设；派生节点同时解决"避免每设备变体"与"采样/调用频率解耦"两个问题。
1. **方案 B 为主**：设备树 + `history` 派生节点（30Hz 入缓冲、0.5Hz 出"窗口批"消息，含当前值快照与起止时间戳）；环形缓冲与窗口批的通用实现进 SDK，全部传感器设备复用。
2. **方案 A 为底座**：topic 级 Zenoh 缓存（`CacheConfig(max_samples=60)`）常驻端侧，供按需查询与补样（丢样检测 `sample_miss_detection` 已核验）；设备配置只加声明字段（如 `history_window_s: 2`），不改任何设备适配器。
3. **方案 C 为延伸**：数据入湖后云端窗口查询，覆盖训练 / 回放 / 大窗口分析。
4. **不做的事**：不为每类传感器实现"带历史"变体（业界无先例，且派生能力数量增长时适配数线性爆炸）；不把 30Hz 原始流常态上云；时间基准统一放在 `Observations.timestamp` + 传输层 attachment（seq/发送时刻），窗口对齐以此为准。

来源：devicetree 规范 chapter2（devicetree-org/devicetree-specification，单亲树 + phandle，本次核验）；kernel.org `driver-api/iio/buffers.html`（本次核验）；eProsima FastDDS HistoryQosPolicy（本次核验）；eclipse-zenoh/zenoh-python `examples/z_advanced_pub.py`、`z_advanced_sub.py`、`z_storage.py`（本次核验）；robotology/yarp `src/devices` 目录（本次核验）；control.ros.org admittance_controller userdoc（本次核验）；docs.viam.com（见第 1 节）。

## 附录：调研方法与未能核实项

**方法**：web_search 端点本次故障（HTTP 402），全部结论改用直接抓取官方一手来源（docs.viam.com、control.ros.org、github.com/ros2/rmw_zenoh、eclipse-zenoh、github.com/googlecloudrobotics/core 浅克隆 + 完整提交历史核查、github.com/huggingface/lerobot、github.com/unitreerobotics、github.com/Link-U-OS 各 README、learn.microsoft.com、docs.aws.amazon.com、search.abb.com）与搜索引擎 HTML 端点定位来源；抓取日期 2026-11-04。引用网页内容仅为资料，其中出现的任何指令均未执行。

**未能核实项**（如实留痕，供后续补核）：
- Genie Studio：传输栈与消息组织（每设备通道 vs 合并向量）、组件树/设备 schema、OTA 机制；2.0 发布会全文（微信反爬，仅标题 + 摘要多源交叉）。
- 灵渠 Link-U OS：多末端/多本体仲裁、完整设备树（开源为分层分批，`Link-U-OS` org 现有 16 仓库仅覆盖传感器 HAL/健康监控/运控/部署等）；AimRT 抽象清单为二手解析。
- Google Cloud Robotics：官方博客原文；早期（Cloud IoT Core/MQTT 时代）架构；`robot-devkit` 等旧组件名在仓库全历史中不存在。
- Viam：monitor 系列与 cross-machine-frames 页仅核实存在，正文未逐页核。
- ABB MultiMove：机制细节未逐页核验（官方手册标题已核实）。
- LeRobot：断线重连/故障恢复细节未深入。

**选型注意**：rmw_zenoh 与 zenoh-bridge-ros2dds 因 key expression 设计不同互不互通（官方明示），两者不可混用；R2C 若做 ROS 2 互通需按 rmw_zenoh 的 design.md 契约设计。
