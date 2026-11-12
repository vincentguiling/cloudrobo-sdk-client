# AgiBot G01（智元精灵 G01）适配器 — 环境配置与启动指导

> 本文件负责 `agibot_g01` 适配器的环境配置、启动与现场验证；R2C 数据面与 CLI 的通用用法见
> `packages/cloudrobo-r2c/docs/index.md`，字段映射细节见同目录
> `config/robot_agibot_g01_config.yaml` 的注释。适配器接口依据
> 《智元精灵G01+GDK v1.5.0使用指南》第 7 章（RobotDds）与第 9 章（CosineCamera）。

## 1. 环境要求

| 项 | 要求 | 说明 |
|---|---|---|
| 运行机 | 与 G01 同网段的 **x86 Ubuntu 22.04** PC | GDK（`a2d_sdk`）仅提供 Linux 环境；R2C SDK 侧无平台限制，但适配器必须与 a2d_sdk 同进程运行 |
| Python | a2d_sdk 侧 **3.10**；R2C SDK 侧 ≥ 3.10 | 建议 conda/venv 隔离 |
| ROS2 | 不需要 | 本适配器走 GDK Python 接口，不依赖 ROS2 |
| 机器人 | 已开机并完成初始化（开机后等待约半分钟至一分钟再切换模式） | 见《指南》2.3 |

## 2. 安装 GDK（a2d_sdk）

`a2d_sdk` 不发布到 PyPI，必须从机器人内置的安装服务获取（《指南》2.2）：

```bash
# 1) PC 与 G01 网络连通后，在目标虚拟环境中执行
sudo apt install iproute2
pip install -r requirements_gui.txt   # GDK 官方依赖清单（numpy/protobuf/scipy/opencv 等）
curl -sSL http://10.42.0.101:8849/install.sh | bash   # 10.42.0.101 为机器人默认地址

# 2) 安装完成后必须执行（加载 DDS 等环境变量）
cd a2d_sdk
source env.sh
```

注意：

- 需先进入目标虚拟环境再执行安装脚本；GDK 升级后需重新更新 PC 侧环境信息。
- `source env.sh` 是 `RobotDds` / `CosineCamera` 正常通信的前提（每次新开终端都需要）。
- 未安装 `a2d_sdk` 时适配器 `connect()` 会抛出带本指引的 `RuntimeError`。

## 3. 机器人切换 Copilot 模式

外部调用 GDK 接口要求机器人处于 **Copilot 模式**（《指南》2.3）：

```bash
# 进入机器人容器/虚拟环境后执行
robot-service -s -c ./conf/copilot.pbtxt
```

- Copilot 模式默认开放 4 路相机：头部 D455/D457 彩色+深度、左右腕 D405 彩色，默认 30Hz。
- 分辨率：D455 彩色/深度 1280×720，D457 1280×800，D405 848×480。
- 修改 `copilot.pbtxt` 后需保存并重启完成模式切换。

## 4. 安装 R2C SDK

```bash
# 仓库根目录
pip install -r requirements-dev-editable.txt   # 开发安装（含 hw-cloudrobo-r2c）
# 或仅安装运行所需
pip install "hw-cloudrobo-r2c[client]"
```

适配器本身只额外依赖 `numpy`（已包含在 `client` extra 中）。

## 5. 机器人配置

使用 `src/cloudrobo_r2c/config/robot_agibot_g01_config.yaml`，关键项
（`hardware.config`，均有安全默认值）：

| 键 | 默认 | 说明 |
|---|---|---|
| `init_wait_s` | 0.5 | `RobotDds` 初始化等待秒数（《指南》7.4 建议等待资源就绪） |
| `camera_enabled` | true | 是否启动 `CosineCamera` 取流线程 |
| `camera_group` | head / hand_left / hand_right | 启用的 G01 相机通道（`head_depth`、`*_fisheye` 等可扩展） |
| `camera_key_map` | head→cam_high、hand_left→cam_left_wrist、hand_right→cam_right_wrist | G01 通道名 → R2C 图像键 |
| `camera_poll_interval_s` | 0.03 | 取流轮询间隔（≈30Hz） |
| `enable_motion_controller` | true | 启用 `RobotController`（《指南》8 章）：末端位姿/关节控制；创建失败自动降级为仅第 7 章关节控制 |
| `control_lifetime_s` | 1.0 | 第 8 章位姿/关节指令有效时间（秒），超时未收到下一帧则停止 |
| `pose_control_group` | ["left_arm", "right_arm"] | 位姿控制部位（`set_end_effector_pose_control` 的 control_group） |
| `dry_run` | false | 适配器级干运行：只打印动作不执行 |
| `commands.go_home` | 全零示例位姿 | GDK `reset()` 复位目标，现场按需修改 |

深度/鱼眼通道接入：在 `camera_group` 中加入通道名，并在 `device_to_r2c.mappings`
仿照现有条目增加 `images.color.*`（或深度用 `images.depth.*` + `ndarray_to_png`）映射。
`body_pose`（整体姿态）维度随整机配置变化，默认不映射，现场确认维度后可仿照
`extensions.head_joint` 增加（`dtype: FLOAT32` + 固定 `shape`）。

### 控制路由：按 joint_states.names 逐维区分第 7 / 8 章

云端动作的 `joint_states.names` 逐维决定适配器行为（YAML 已内置 `joint_names` +
全量 `position` 映射），names 与 position 一一对应：

| names 逐维内容 | 章节 | GDK 调用 | 说明 |
|---|---|---|---|
| `left_arm_joint_1..7` / `right_arm_joint_1..7`（须 14 维齐全） | 7 章 | `move_arm` | 关节角按名回填；**不全不硬执行** |
| `left_pose_x/y/z` + `left_pose_rx/ry/rz/rw`（四元数） | 8 章 | `set_end_effector_pose_control` | 各侧分量须完整，内部无补帧 |
| `left_pose_x/y/z` + `left_pose_roll/pitch/yaw`（欧拉角，ZYX） | 8 章 | 同上 | 内部转换为四元数 |
| `left_gripper` / `right_gripper`（两侧须同时出现） | 7 章 | `move_gripper` | 优先于 `gripper_cmd_pos` 通道 |
| 全部为部位组名（`left_arm` / `right_arm` 等，每组 7 关节） | 8 章 | `set_joint_position_control` | 组名路由 |

**互斥与完整性规则（违反即拒绝执行整条动作，不做补帧兜底）**：

- 关节与末端位姿互斥：同一动作不得同时包含关节分量与位姿分量（含经
  `end_effector_poses` 通道下发的位姿）
- 四元数与欧拉角互斥：同一动作内不得混用两种表示（左右侧之间也须一致）
- 不全不硬执行：关节须 14 维齐全、每组关节须 7 维、位姿各侧分量须完整、
  夹爪须两侧同时出现，否则拒绝执行
- 未知 names 视为协议不一致，拒绝执行

### 观测镜像：位姿与夹爪复制进 joint_states

`device_to_r2c` 会把末端位姿与夹爪复制一份进 `joint_states`（YAML 已内置）：

- 14 双臂关节角 + 末端位姿逐维镜像（四元数 `left_pose_x..rw` 共 14 维，或经
  `hardware.config.ee_pose_repr: "euler"` 切换为 `left_pose_x/y/z/roll/pitch/yaw`
  共 12 维，由四元数转换）+ 夹爪 `left_gripper` / `right_gripper` 2 维
- 即 `joint_states.position` 维度 = 14 + 14 + 2 = 30（quat 默认）或 14 + 12 + 2 = 28
  （euler），**云端模型输入维度需按此配置**
- 同时 `end_effector_poses.pose` 与 `end_effector_states.position` 仍照常各发送一份；
  位姿不可用（运控器禁用/未就绪）时仅省略位姿段

### 样例 cloud-adapter：3 组动作往复

`examples/agibot_g01_cloud_adapter.py` 演示云端在 3 组动作之间来回下发
（真机调试采集的关节目标值，均为第 7 章关节动作）：

- 关节插值：从当前关节角线性插值到目标，**chunk size = 2 × publish_hz**
  （默认 30Hz → 60 步，每组执行约 2 秒），避免单步跳变超 0.2618 rad（15°）限幅
- 夹爪任意角度：三组分别 `0.15/0.15`、`0.85/0.35`、`0.5/0.8`（连续值透传）
- 往复序列 `0 → 1 → 2 → 1 → 0 → ...`
- 连接约定（client config）：**zenohd 与网关监听 `tcp/0.0.0.0:7447`**（所有网卡；
  网关侧 client config 用 `endpoint_role: "listen"` + `mode: "peer"`）；云端适配器
  `endpoint_role: "connect"`，同机部署连 `tcp/0.0.0.0:7447`，跨机部署用
  `--zenoh-endpoints tcp/<网关IP>:7447` 指向实际地址；`project_id` / `device_id`
  按现场部署填写（示例默认 `r2c` / `local`），与机器人端取值一致
- 第 8 章示例组（`--include-ch8`）：组 3 位姿+夹爪经 joint_states 逐维 names、
  组 4 位姿经 `end_effector_poses` 通道、组 5 组名关节（详见模块 docstring）；
  默认循环不含这些组（真机安全）

```bash
# 云端侧（与机器人端 r2c client 同网段；建议机器人端先 dry_run: true）
python examples/agibot_g01_cloud_adapter.py --project-id r2c --device-id local \
    --steps 60 --duration 60
# 第 8 章示例组（位姿目标 = 观测当前末端位姿 + z 偏移）
python examples/agibot_g01_cloud_adapter.py --include-ch8 --pose-offset-z 0.05
```

## 6. 启动与验证

```bash
# 1) 干运行（安全默认）：观测正常发布，动作只打日志不执行
cloudrobo r2c client \
    --bundle <cert_config_*.zip> \
    --robot-config <path/to/robot_agibot_g01_config.yaml> \
    --log-level INFO
```

验证要点：

- 日志出现 `AgiBot G01 client connected` 与相机取流启动日志；无 `a2d_sdk` 时会得到
  明确报错并指向本文件第 2 节。
- 云端能收到观测（关节数值随姿态变化；相机首帧可能为空——`CosineCamera` 首帧返回
  `None`，取流线程会自动过滤，稍等即有图像）。
- 下发动作仅出现在 `[AgiBotG01][DRY_RUN]` 日志中。

```bash
# 2) 真实执行：确认无误后关闭两级 dry_run
#    runtime.dry_run: false（客户端级）与 hardware.config.dry_run: false（适配器级）
```

安全注意（《指南》5.1 / 7.5）：所有角度为弧度，腰部高度单位为厘米；手臂目标关节角与
实时值差值需小于 0.2618 rad（15°），由上层规划保证；首次上电先 `go_home` 复位验证。

## 7. 常见问题

| 现象 | 处理 |
|---|---|
| `a2d_sdk 未安装` RuntimeError | 未部署 GDK 或未 `source env.sh`，见第 2 节 |
| 相机长时间无图像 | 确认机器人处于 Copilot 模式且 `copilot.pbtxt` 开启对应相机通道；首帧 `None` 属正常 |
| 关节状态查询失败告警 | 单次 DDS 异常，适配器自动沿用上一帧值；持续出现检查网络与模式 |
| 动作维度报错跳过 | 云端动作维度需为 双臂 14 / 夹持器 2 / 灵巧手 12 / 头 2 / 腰 2 / 轮 2 |
| `body_pose` 未上报到云端 | 默认不映射（维度未确认），见第 5 节 |

## 8. 离线自检（无真机）

```bash
python -m pytest packages/cloudrobo-r2c/tests/test_agibot_g01_adapter.py -v
```

测试以假 `a2d_sdk` 模块覆盖连接生命周期、观测组装、动作分发、干运行与
YAML→翻译器端到端链路，无需 GDK 环境。
