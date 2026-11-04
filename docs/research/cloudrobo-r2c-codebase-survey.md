# cloudrobo-r2c 代码完全调研（为设备树领域建模校准）

> 本文件负责对 `packages/cloudrobo-r2c` 的现有实现做完全调研：模块地图、运行时数据流、配置体系、"设备"概念在各层的真实语义、扩展点，以及据此对《R2C 多设备管理（设备树）特性设计（方案 B）》的偏差清单与修正方向；不属于本文件的内容：业界调研见 [r2c-multi-device-industry-survey.md](r2c-multi-device-industry-survey.md)，方案 B 设计稿见 [../design/r2c-multi-device-tree-design.md](../design/r2c-multi-device-tree-design.md)（其领域建模以本文件第 9–10 节为准修订）。
>
> 调研日期：2026-11-04；方法：4 路并行精读（会话/传输、适配器、核心映射、云侧与配套）+ 交叉核验，结论均带 `文件:行号`。行数为约数。`core/interfaces.py:1` 自述"Abstract interfaces inspired by the r2c_v2 draft's boundary design"——该 r2c_v2 草案不在本仓库，需向团队索取（见第 8 节缺口清单）。

## 1. 模块地图

```text
src/cloudrobo_r2c/
├── client/            # R2CClient/R2CSession：会话、topic 拼接、H264 编解码、心跳循环
│   ├── client.py      #   connect() 唯一入口（~240 行）
│   ├── session.py     #   R2CSession：publish/subscribe 全套方法（~650 行）
│   └── _heartbeat_loop.py  # HeartbeatLoop 后台线程
├── cloudroboclient.py # 进程入口：装载配置→adapter→translator→SyncRobotClient→心跳（~440 行）
├── sync_client.py     # SyncRobotClient：控制循环/action 队列（~1100 行）
├── core/              # interfaces.py（五接口）、config_mapper.py（映射引擎 ~880 行）、
│                      # transformers.py（变换库 ~2400 行）、fusion.py（融合 ~510 行）、internal/
├── robots/            # robot_factory.py + 16 个 entry-point 适配器 + commands/ 命令框架
├── translators/       # ConfigurableDeviceTranslator / ConfigurableModelTranslator / 工厂
├── transport/zenoh.py # ZenohTransport：字节级 pub/sub（~1300 行）
├── common/            # config.py（ClientConfig/连接）、robot_config_schema.py（pydantic v2.1）、
│                      # schema.py（只读导出）、models/（Observations/Actions/Heartbeat…）、protos/
├── inference/         # 云侧：lerobot policy server、OpenPI ws/http 客户端、cloud config 规范化器
└── tools/r2c_config_migrator/  # 旧 r2c_config JSON → v3.4 routes YAML
config/                # client 1 / cloud 7 / robot 18 / dummy 3 / simulation 5 / test_dummy 6
tests/                 # 仅 test_adapter_registry.py（2.5KB）——包内自动化测试几乎为空
scripts/, examples/    # mock_policy_server、export_schema、40+ 云适配器/app 样例
```

## 2. 端到端数据流（现状）

**端侧主循环**（`sync_client.py`，单控制线程）：
1. `SyncRobotClient.from_config`：`runtime.publish_hz` → `target_fps`（**默认 30**，sync_client.py:326），`environment_dt = 1/fps`（:236）。
2. `_control_loop`（:922-945）每 tick：fusion 进行中跳过 → `_apply_one_action_step`（出队 action step → `translator.r2c_to_device` → 非 dry_run 才 `adapter.send_action`，:1131-1180）→ `_check_and_publish`（:947-965）→ 按 dt sleep。**控制/采样节拍与观测发布节拍是同一个**。
3. 观测发布 `_publish_latest_observation`（:967-1026）：`adapter.get_observation()`（:971，设备原生扁平 dict）→ `translator.device_to_r2c`（:978；`check_completeness` 不完整则跳过本 tick，device_translator.py:42-50）→ recorder → `session.publish_observations`（:1011）。触发条件：异步模式下队列深度 ≤ `publish_trigger_threshold` 或等待超时（3s 起 ×backoff，封顶 `action_response_timeout_s`）。
4. action 接收 `_enqueue_action_chunk`（:529-611）：stale 判定（`actions.id ≠ _latest_observation_id`，:688-714）→ chunk 对齐（前 6 关节欧氏距离，:613-674）→ `max_enqueue_actions_per_chunk` 截断 → n-skip → `FusionEngine.apply`（:818-852）。

**云侧 policy server**（`inference/r2c_lerobot_policy_server.py`）：**纯事件驱动**——`subscribe_observations` 每条消息回调一次、调一次模型、回一条完整 action chunk（:481-520）；`chunk_size` 来自云配置 `model.action_chunk_size`（cloud_config.yaml:35=50）或按映射后关节维度推断（model_translator.py:87）。**代码中不存在"模型调用频率"参数**：实际调用率由机器人侧 `publish_hz` 的观测流速决定；云侧唯一节流项是 `runtime.delay_return`（发 action 前 sleep，:453-465）。远程推理另有 OpenPI WebSocket/HTTP 客户端（probe 探活、eager/lazy 连接策略，ws_policy_client.py:43-213）。

## 3. Topic 与 QoS 全表

Topic 拼接存在**两处等价实现**：`client/session.py` 各方法内 f-string（session.py:93/145/203/264/287-341）与 `transport/zenoh.py` 的 `TOPIC_DEFINITIONS` 注册表（zenoh.py:59-100，含遗留段 `observation`/`action_chunk`/`robot_meta`）；底层 `_build_standard_topic = {project_id}/{device_id}/{relative_path}`（zenoh.py:1143-1152）。

| Topic | 消息 | 默认 QoS（cfg:24-48） |
|-------|------|----------------------|
| `{p}/{d}/inference/observations` | Observations / ObservationsH264 | RELIABLE / DROP / REAL_TIME |
| `{p}/{d}/inference/actions` | Actions | RELIABLE / DROP / REAL_TIME |
| `{p}/{d}/state/joint_states` 等 4 类 | 各 State proto | BEST_EFFORT / DATA |
| `{p}/{d}/state/heartbeats` | Heartbeat | BEST_EFFORT / BACKGROUND |

会话层特性：H264 编码在 session 层（共享 encoder，session.py:489-538），传输层只收 bytes；`ZenohTransport` 无主动重连（依赖 zenoh 后台重试 + connectivity listener 更新状态，zenoh.py:457-722）；publisher 懒声明缓存，逐条 `publisher.put`（无批量/SHM）。

## 4. 配置体系

- **robot_config（v2.1）**：pydantic 校验在 `common/robot_config_schema.py`，**`extra="allow"`**（未知字段不报错，:20-23），必填 5 节 runtime/hardware/translator/device_to_r2c/r2c_to_device（:202-213）；`common/schema.py` 只是只读 schema 导出器（复用 MapperRule 解析 mappings 为字段元数据，单条失败仅 warning，:376-382）。
- **映射表**：`device_to_r2c/r2c_to_device.mappings` 由 `ConfigurableDeviceTranslator` 构建为两个 `ConfigDrivenMapper`（device_translator.py:80-101）。MapperRule 完整语义（config_mapper.py:136-828）：`source`（单点路径）/`source_paths`（多路径成列表）/`source_index`/`source_name_ref`（按 names 推导取索引，优先于 index）/`target`/`target_index`（自动补 None）/`transforms`（管线，56 个内建变换 + `r2c_sdk.transformers` entry-point，builtin 优先）/`use_lookup_dotted`/`use_assign_dotted`/`required`/`default`（无 source 即常量）/slice/list_mode/extension 打包 ExtensionValue。
- **translator 工厂**：`configurable`（默认）或 `custom`（class_path 动态加载，经模块黑名单校验，translator_factory.py:32-60）。
- **cloud_config**：normalizer 把 v2.2–v3.2 单路由旧格式与 v3.3–v3.9 `routes[]` 统一为 `NormalizedConfig`（r2c_config_normalizer.py:33-210），endpoint 校验 `{{HOST_URL}}` 模板与 ws↔wss/http↔https 家族互通。
- **migrator**：旧 r2c_config JSON（model_feature_mapping）→ v3.4 routes YAML，显式丢弃旧 chunk_size/shape/stop_condition（converter.py:27-29）。

## 5. 适配器生态

- **契约**（core/interfaces.py）：`IRobotHardwareAdapter` = `connect/disconnect/get_observation() -> 设备原生扁平 dict/send_action(设备原生 dict)` + 命令注册（`register_command_class/register_command/execute_command`，:34-79）；`IDeviceTranslator`（device↔R2C）、`IModelTranslator`（R2C↔模型）、`IValueTransformer`（值级变换）、`IPolicyClient`（云推理边界）。
- **注册**：`r2c_sdk.adapters` entry-point 组 16 个；`AdapterRegistry` 惰性扫描、同名冲突偏好 `cloudrobo_r2c.`（robot_factory.py:100-103，防旧 r2c_sdk 包遮蔽）；`RobotFactory` 另有 legacy 硬编码表（lerobot/ros2/raw_sdk/ur5e_rtde/zenoh_ros1/flexiv/playback/dummy/custom，:442-452）。
- **一个 adapter = 一台整机/工作站**。多设备在 adapter 内部以键前缀或内部子对象存在：moz1 `leftarm_/rightarm_` 前缀、xifeng 前 7 后 7 关节、ur5e/jaka `_cameras`/`_gripper` 内部子对象（无对外枚举 API）、a1z 每相机采集线程 + `camera_meta`（含 timestamp/age_ms/stale/error，a1z_hardware_adapter.py:249-254）、q25 全扁平 72+ 键、flexiv `wrench_raw_*` 与 `wrench_filtered_*` 双变体。
- **commands 框架**：`AdapterCommand`（`requires_pause` 属性联动 keyboard 暂停，commands/base.py:64-90）+ YAML `commands.<instance>.{type,...}` 工厂实例化（robot_factory.py:489-547）；go_home 家族按机型分派。
- **ros2 适配器**（最声明式的现成范式）：`SubscriptionConfig(topic,msg_type,qos,store_as,include_fields,field_aliases,max_update_hz,transforms)`、`ObservationSyncConfig`（message_filters）、publisher/service/action 三类命令路由、`get_transport_targets(kind)` 能力查询（ros2_hardware_adapter.py:75-149,197-222）。zenoh_ros1 适配器：zenoh-bridge key `{hex(datatype)}/{md5}/{namespace}/{topic}` + wire 类型表 + `go_home_role` 角色标注。

## 6. 云侧与配套

CLI 仅 `r2c client` 一子命令（cli.py:8-25，宿主 `cloudrobo r2c`）；policy server / cloud adapter / mock server 是独立 argparse 入口。`mock_policy_server.py` 提供无 torch 的 E2E（正弦 action chunk）。

## 7. "设备"语义盘点（分层）

| 层 | "设备"语义 | 多设备现状 | 设备树挂接点 |
|----|-----------|-----------|--------------|
| 连接/会话 | `device_id` = 整机身份，一 session 一设备 | 仅 `subscribe_*(target_device_id=)` 可跨设备订阅；发布恒用自身 device_id（session.py:203） | `_build_standard_topic` 已参数化 device_id |
| 适配器 | adapter = 整机，扁平 dict 边界 | 键前缀约定 / 内部子对象，无对外声明 API | 命令注册表、`camera_meta`、`get_transport_targets`、`RobotFactory._register_adapter_commands` 注入缝 |
| 翻译 | 全局单表，无 device/group 字段 | 仅相机前缀 `images.color.<cam>` 与 `extensions` 天然分组 | MapperRule 加 `device` 字段或分表；`custom translator` 逃生口 |
| 消息 | Observations/Actions 扁平 + 相机 map + extensions | `end_effector_poses` 已是 `map<string,…>` | map 模式推广为通用 per-device 分组 |
| 数据面 | topic 首段 `{p}/{d}` = 整机 | 无设备段 | 窗口批等新 topic 需同步改 session.py 与 TOPIC_DEFINITIONS 两处 + QoS 表 |
| 心跳/生命周期 | 整机 HeartbeatLoop provider（默认 5s±jitter） | 无 per-device | provider 扩展 + proto `Heartbeat.devices`（只增） |
| 云侧 | 单设备订阅（--device-id） | 无 | target_device_id 已支持跨设备订阅 |

## 8. 质量现状与缺口

1. **测试近乎为空**：`tests/` 仅 1 个 registry 回归测试；session/config/translator/fusion 无单测；E2E 靠 `run_e2e_tests.sh`（引用外部遗留路径 `/home/robot/suhanwu/r2c_sdk_python`）与 mock 脚本。设备树这类结构改动缺乏回归安全网。
2. **topic 双处维护**：session.py f-string 与 zenoh.py TOPIC_DEFINITIONS 等价并存，新增 topic 类型需改两处，漂移风险。
3. **`image_encoding` 是死字段**：YAML 声明性字段无运行时消费者，`Observations.to_protobuf` 恒写 `"raw"`（observations.py:301）；H264 走独立 `ObservationsH264` 消息模型（session.py:167 按标记识别）。
4. **悬空引用**：ros2 adapter 引用 `commands.ros2.Ros2GoHomeCommand`（ros2_hardware_adapter.py:239，ADR-0005）但 commands/ 无该文件。
5. **r2c_v2 草案不在仓库**：接口层明确受其"边界设计"启发，但文档缺失——设备树领域模型应与它对齐，需团队提供。
6. pydantic `extra="allow"`：新配置节（如 `devices`）若不做显式校验，拼错字段不会报错。

## 9. 设备树设计 v0.1 的偏差清单

对照《方案 B 特性设计（v0.1）》与代码现状：

| # | v0.1 建模 | 代码现状 | 偏差定性 |
|---|-----------|----------|----------|
| 1 | `devices` 列表每项 = 独立 adapter 实例，DeviceManager 聚合 | 一个 adapter = 一台整机；**不存在多 adapter 聚合层**；多设备是 adapter 内部键前缀/子对象 | 领域锚点错位：v0.1 引入了架构中不存在的"聚合层"，未说明这是全新机制 |
| 2 | 派生节点 `input` 引用"设备" | 数据流的单元是**键空间**（映射表 source/target 均为键路径）；"设备端口"概念不存在；滤波类派生（flexiv wrench_filtered）当前发生在 adapter 内部 | 派生节点输入应定义在键空间/通道层，或先定义设备=键空间分组的声明 |
| 3 | 窗口批 topic 新增段 | topic 在 session.py 与 TOPIC_DEFINITIONS **双处维护**，且 QoS 按消息类型配置 | 设计遗漏双处改动与 QoS 表扩展 |
| 4 | "常规 Observations 维持 publish_hz" | 云侧调用率由观测流速驱动、无频率参数；用户约束"所有观测 0.5Hz 上传、不支持高频上传 60 samples"意味着**常规观测上行也应对齐 0.5Hz** | v0.1 的 D3 建议与用户约束冲突 |
| 5 | DeviceManager 按源设备配 RingBuffer | `sync_client` 是**单节拍循环**（控制、采样、发布同频）；30Hz 采样与 0.5Hz 上行解耦需要把"采样写缓冲"与"上行"拆成两个节拍（adapter 内线程或改控制循环） | 遗漏现状单节拍约束，解耦点未落地 |
| 6 | "配置校验拒绝加载"是现有能力 | pydantic `extra="allow"`，未知字段静默；`devices` 节需要显式新增校验 | 校验非自动获得 |
| 7 | 未提及 commands 框架 | 命令已是 per-adapter 实例注册机制（YAML `commands.<instance>.type`） | 设备树"节点/命令"应与该形态对齐 |
| 8 | 未覆盖 image/H264 现状 | `image_encoding` 死字段、H264 独立消息 | 涉图像的设计点需绕开死字段 |
| 9 | 未对齐 r2c_v2 边界草案 | 接口层受其启发但草案不在仓库 | 需索取草案后再定稿领域模型 |

## 10. 修正方向：设备树的三个候选组合层级（待评审定案）

**路线 1：键空间树（配置/映射层组合，侵入最小）**
- "设备" = adapter 键空间的声明式分组：`devices.<name>` 声明 `{type, channel 前缀/键清单, sample_hz}`；映射表按设备分组（MapperRule 增 `device` 字段或 `devices.<name>.mappings`）；Observations/Actions 增 per-device 分组（推广 `end_effector_poses` 的 map 模式）；`history` 派生节点在**翻译后的 R2C 载荷层**工作（端侧 translator 之后、session 之前的管线），以 `(device, channel)` 为输入。
- 优点：单 adapter/单 translator/单会话不变，ring buffer 挂在载荷管线，改动集中在 config/mapper/新增节点；与现状"多设备=键前缀"完全同构（moz1/q25 即此形态）。
- 缺点：设备身份靠约定（前缀清单正确性由 adapter 作者保证）；跨 adapter 组合不支持。

**路线 2：adapter 树（v0.1 方案，多 adapter 聚合层）**
- 每设备独立 adapter + translator + 生命周期，DeviceManager 聚合，设备可各自 topic/带宽。
- 优点：设备身份/生命周期最干净，独立故障隔离天然成立。
- 缺点：引入全新聚合层与多 translator 编排，存量 16 个 adapter 均为"整机"语义需重写拆分；改动量大，且与"高频数据不出端侧、统一 0.5Hz 上行"的约束无额外收益。

**路线 1.5：声明式子设备清单（中间态，推荐）**
- 保持单 adapter，但要求 adapter 声明子设备（推广 ros2 `get_transport_targets`/a1z `camera_meta`/命令注册表为通用 `describe_devices() -> [{name, type, channels}]` 元数据 API）；配置里 `devices` 树引用这些声明并叠加映射分组与派生节点。
- 优点：设备身份有正式声明（消除纯前缀约定），又不引入多 adapter 编排；与 ros2 适配器已有的声明式风格一致。
- 缺点：需要给 16 个 adapter 补 `describe_devices()`（可先支持声明式最优的 ros2/lerobot，其余回退到配置声明）。

**与用户约束的对应（三路线共同点）**：采样/历史 30Hz 在端侧环形缓冲（写入点在路线 1/1.5 = 载荷管线，路线 2 = 各 adapter）；上行统一 0.5Hz 窗口批 + 常规观测上行对齐 0.5Hz（修正 v0.1 偏差 #4）；`history` 派生节点通用实现一次，传感器 adapter 零改动。

## 11. 后续动作

1. 评审本文件第 10 节，定组合层级（路线 1 / 1.5 / 2）；
2. 取得 r2c_v2 边界草案，校准接口层命名；
3. 按定案修订《方案 B 特性设计》至 v0.2（含偏差 #3/#4/#5 的落地细节与新增校验）；
4. 补关键路径单测（mapper/translator/session）作为设备树改动的前置安全网。
