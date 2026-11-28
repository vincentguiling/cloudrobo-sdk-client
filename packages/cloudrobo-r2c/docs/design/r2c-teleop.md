# R2C 遥操操作端接入指南（r2c teleop）

本指南对应设计文档《R2C 遥操特性设计》v3.2（并入 r2c 仓库方案，§3.3.6）与实现
`packages/cloudrobo-r2c/src/cloudrobo_r2c/teleop/`。操作端（遥操操作台）随 r2c 包发布，
无需独立仓库。

## 快速开始

```bash
# 安装（操作端依赖单独一个 extra；机器人端仍用 [client]）
pip install hw-cloudrobo-r2c[teleop]

# 1) 启动 Zenoh 路由器（本地）
zenohd --listen tcp/0.0.0.0:7447

# 2) 机器人端照常运行（发布 state/current，teleop_to_device 已声明即接受遥操）
cloudrobo r2c client --robot-config config/robot_dummy_config.yaml

# 3a) VR 设备（默认）：网关伺服 WebXR HUD 页面 + /ws
cloudrobo r2c teleop gateway --port 8000
#     Quest：adb reverse tcp:8000 tcp:8000，浏览器打开 http://localhost:8000

# 3b) mock 设备（无头显）：按 state/current 末端位姿上下左右小幅偏移自动发动作
cloudrobo r2c teleop mock

# 4) 观察总线（帧率 / 延迟 / 帧大小 / 命令帧）
cloudrobo r2c teleop bench --seconds 20
```

默认两个遥操设备：

| 设备 | 命令 | 输入 | 映射 | 会话 |
|------|------|------|------|------|
| VR | `cloudrobo r2c teleop gateway` | WebXR HUD（Quest 浏览器，three.js） | `relative_pose`（相对位姿 + 离合重锚 + reach limit + EMA） | HUD 的 B/Y 控制（两段式 start；长按 finish）；`--bundle` 时由云端驱动 |
| mock | `cloudrobo r2c teleop mock` | 合成：读取 `state/current` 每臂末端位姿并叠加小幅摆动（默认半径 2cm、0.3Hz） | `passthrough`（样本位姿即目标位姿） | 自动 start（`--auto-session`） |

mock 参数：`--amplitude 0.02`、`--rate 20`（发布频率）、`--port`；配置文件
`teleop.input.params` 里可设 `side`（both/left/right）、`amplitude_m`、`rate_hz`、
`interval_s`。

## 接入新输入设备（同构臂、3D 鼠标等）

设备接入 = 实现一个 `InputSource` + 注册 entry point + 选择映射策略，**不改 r2c 代码**。

### 1. 实现 InputSource

```python
from cloudrobo_r2c.teleop.inputs import InputSample, INPUT_MODE_VR  # 或 INPUT_MODE_ISOMORPHIC

class MySpaceMouse:
    name = "space_mouse"
    input_mode = INPUT_MODE_VR
    ws_path = None            # 若设备自带 HUD 页面才需要 "/ws"
    wants_auto_session = True  # 无操作员按钮时自动开会话（可选）

    def __init__(self, params: dict, state_getter=None):
        self.params = params
        self.state_getter = state_getter   # 需要 state/current 基准时使用

    def capabilities(self) -> dict:
        return {"bimanual": False, "trigger": True, "deadman": "grip",
                "buttons": [], "rate_hz": 125}

    async def start(self, sink):
        # 设备回调里把读数归一化为 InputSample 列表交给 sink（网关负责节流/映射/发布）
        # await sink([InputSample(side="right", pose=[...7 值...], grip=True, trigger=0.5)])
        ...

    async def stop(self):
        ...
```

约定：

- `pose` 为 7D `[tx,ty,tz,qx,qy,qz,qw]`；同构臂类设备用 `joints`（关节绝对位置，
  映射策略选 `isomorphic`，后续扩展）。
- **deadman**：必须提供按住才跟随的语义（grip/trigger 或外接按键）；声明
  `deadman: "none"` 时网关以动作流超时兜底并告警。
- 预对齐（同构臂）在源内完成：对齐完成前不要产出动作样本。
- 速率高于 `teleop.actions_rate_hz` 时由网关节流，只保留最新样本。

### 2. 注册

```toml
# 你的包 pyproject.toml
[project.entry-points."r2c_sdk.teleop_inputs"]
space_mouse = "my_pkg.inputs:MySpaceMouse"
```

或配置里直接写 `模块:类` 路径（无需安装 entry point）。

### 3. 选择映射策略

```yaml
teleop:
  input:
    source: space_mouse       # 内置 webxr | mock；或 entry point 名 / module:Class
    params: {side: right, scale: 0.5}
  mapping: relative_pose      # relative_pose（位姿类）| passthrough（样本即目标）| 第三方
```

自定义映射同样注册 entry point `r2c_sdk.teleop_mappings`，构造签名
`(reach_limit_m, smoothing_alpha)`（或省略参数）。

### 4. 验证

```bash
cloudrobo r2c teleop bench --seconds 20          # 帧率/延迟/帧大小/命令帧
cloudrobo r2c teleop state-inspect               # state/current 采样率与位姿字段
cloudrobo r2c teleop wsdriver --port 8000        # 模拟 HUD 驱动网关（无头显）
```

验收清单：start/finish 全程动作连续；deadman 松开目标保持；无 state/current 基准时
拒发该手动作；预对齐（如适用）完成前无动作帧；心跳 mode 在会话期间为 CONTROLLING、
空闲为 AUTO。

## 配置参考（teleop-operator.yaml）

```yaml
zenoh:
  mode: client                 # client | peer
  endpoints: ["tcp/127.0.0.1:7447"]
  tls: {enabled: false}        # --bundle 时自动覆盖（r2c client 同款凭据包）
teleop:
  project_id: r2c
  device_id: local             # --bundle 时取凭据包 robot_id
  operator_id: python-gateway
  actions_rate_hz: 20
  reach_limit_m: 0.5
  smoothing_alpha: 1.0
  input:
    source: webxr              # webxr | mock | <entry point> | module:Class
    params: {}
  mapping: ""                  # 空=自动（mock→passthrough，其余→relative_pose）
state_current:
  source: end_effector_poses   # end_effector_poses | joint_states（agibot 镜像）
  effector_names: [left, right]
  format: quat                 # quat（7 值）| euler（6 值弧度）
  euler_order: xyz
  joint_name_suffixes: [left, right]
  values_per_arm: 7
  joint_pose_layout: trailing  # trailing | inline
gateway:
  http_host: 0.0.0.0
  http_port: 8000
session:
  auto_start: false            # mock 默认 true
```

## 话题与协议（不变）

- `{p}/{d}/teleop/actions`：遥操动作（BEST_EFFORT/DataHigh，`actions_rate_hz`）+ 生命周期
  `command` 帧（RELIABLE/InteractiveHigh）；
- `{p}/{d}/state/current`：相对控制基准（CurrentState，约 10Hz）；
- `{p}/{d}/state/heartbeats`：网关心跳（5s±0.5s，BEST_EFFORT/Background，
  mode = AUTO / CONTROLLING）；
- 协议为 r2c 包内 `common/protos/teleop.proto`（TeleopFrame），Python/C# 生成物同源。

## 与旧独立仓库（cloudrobo-teleop）的关系

操作端已并入 r2c 仓库（2026-11-27 起，v3.0 设计）；cloudrobo-teleop 只读冻结为迁移
来源。WS JSON 协议、TeleopFrame 协议、会话状态机与映射算法 1:1 移植，WebXR HUD 页面
零改动复用（`packages/cloudrobo-r2c/src/cloudrobo_r2c/webxr/`）。