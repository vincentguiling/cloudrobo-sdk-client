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
| `dry_run` | false | 适配器级干运行：只打印动作不执行 |
| `commands.go_home` | 全零示例位姿 | GDK `reset()` 复位目标，现场按需修改 |

深度/鱼眼通道接入：在 `camera_group` 中加入通道名，并在 `device_to_r2c.mappings`
仿照现有条目增加 `images.color.*`（或深度用 `images.depth.*` + `ndarray_to_png`）映射。
`body_pose`（整体姿态）维度随整机配置变化，默认不映射，现场确认维度后可仿照
`extensions.head_joint` 增加（`dtype: FLOAT32` + 固定 `shape`）。

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
