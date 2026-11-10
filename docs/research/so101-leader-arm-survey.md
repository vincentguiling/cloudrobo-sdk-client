# SO101 主动臂（Leader Arm）单独使用初步调研

> 问题：不连接从动臂（follower）的情况下，LeRobot 的 SO101 主动臂能否 ①读取关节角度、②做遥操启动前的对位控制（把主臂对到从臂当前位姿）。结论先行：**两个都可行**——读角度开箱即用；程序化对位硬件与库均支持（lerobot 官方 DAgger handover 即此模式），但需小步限速与安全约束；r2c 侧需要一个小的适配扩展（leader 是 `Teleoperator` 而非 `Robot`）。

## 1. 硬件与库形态

| 项 | 事实（来源：lerobot 仓库 main / v0.4.0、so101 官方文档） |
|----|------|
| 舵机 | 主/从臂同为 Feetech STS3215 串行总线舵机 ×6（shoulder_pan/shoulder_lift/elbow_flex/wrist_flex/wrist_roll/gripper），主臂减速比 1/147~1/345（为人力拖动设计，扭矩仍可伤手） |
| 连接 | 主臂独立经 USB 串口连接 PC（`lerobot-find-port` 找端口；Linux 需 `chmod 666 /dev/ttyACM*`）；与从动臂是否存在无关 |
| 类归属 | 主臂 = `Teleoperator`（`lerobot.teleoperators.so101_leader.SO101Leader`，v0.4.0；main 已重命名为 `lerobot.teleoperators.so_leader.SOLeader`，`SO101Leader = SOLeader` 别名）；从臂 = `Robot`（`lerobot.robots.so101_follower.SO101Follower`） |
| 校准 | `lerobot-calibrate --teleop.type=so101_leader --teleop.port=… --teleop.id=…`：homing 位姿（正交位，wrist_roll full-turn），校准文件按 id+port 存本地；`connect(calibrate=True)` 自动触发 |

## 2. 问题一：单独读取关节角度 —— ✅ 开箱即用

`SOLeader.get_action()` 实现即一行总线同步读（so_leader.py:127-133）：

```python
action = self.bus.sync_read("Present_Position", num_retry=…)   # 读 6 颗舵机当前角
action = {f"{motor}.pos": val for motor, val in action.items()}
```

- 无从动臂参与；返回 `{"shoulder_pan.pos": …, …, "gripper.pos": …}`（`use_degrees=True` 时为角度；缺省 [-100,100] 归一化）。
- 官方即有**单独使用主动臂**的先例：Isaac 示例 `examples/isaac_teleop_to_so101/isaac_teleop/teleop_so101_leader_arm.py`（关节空间直读 6 关节，无 retargeter、无从臂）；`lerobot-calibrate` 也是单臂流程。
- 单次 `sync_read` 毫秒级（源码 debug 日志打印耗时），轮询 30–60Hz 无压力。
- r2c 侧 `LeRobotHardwareAdapter.get_runtime()` 已支持 so101 风格 `.pos` 键（缺省 `runtime_joint_names` 即 6 关节 `.pos`），读数落地几乎零成本。

**注意**：`SOLeader` 是 `Teleoperator`，只有 `get_action()`/`send_feedback()`，没有 `Robot.get_observation()`/`send_action()`；r2c 现有适配器（`lerobot_hardware_adapter._default_robot_factory` → `lerobot.robots.make_robot_from_config`，随后调 `get_observation()`）只覆盖从臂路径——接入主臂需要一个小适配分支（见 §4）。

## 3. 问题二：对位控制（把主臂对到从臂当前位姿）—— ✅ 可行，有先例，需安全约束

**库能力**（so_leader.py）：

```python
def send_feedback(self, feedback):          # :136-139
    goals = {k.removesuffix(".pos"): v for k, v in feedback.items()}
    self.bus.sync_write("Goal_Position", goals)     # 硬件写目标位置（与从臂同一舵机型号）
def enable_torque(self): self.bus.enable_torque()   # :117
def disable_torque(self): self.bus.disable_torque() # :120（configure() 缺省关扭矩，保持人力可拖动）
```

**官方先例（正是"对位"场景）**：`bi_so_leader.py:113-141` 的 DAgger smooth-handover——`send_feedback` 把主臂 command 到从臂当前位姿（注释原文："both leader arms are commanded to the follower's current pose to avoid discontinuities"）。即"程序化把主臂对位到从臂当前位姿"是 lerobot 已在用的模式。

**推荐对位流程**（r2c 遥操 §3.3.1.3 同构臂对齐的落地）：

1. 操作台从 `inference/observations` 读从臂当前位姿（关节角）；
2. `enable_torque()` → **小步长插值**写 `Goal_Position`（STS3215 位置模式是阶跃运动，需低步速：先写 `Profile_Velocity/Profile_Acceleration` 限速或分 N 步+延时），逐步逼近从臂位姿；
3. 误差 < 阈值（如 2°）→ `disable_torque()` 交还人力 → 发 `command=start` 进入遥操。

**两条路线对比**：

| 路线 | 做法 | 风险 | 工作量 |
|------|------|------|--------|
| A. 程序化对位（上） | 扭矩使能 + 限速写目标 | 主臂减速比大、扭矩输出可伤手/卷线缆；必须低速+行程 clamp+人旁急停 | 1–2 天（含参数调试） |
| B. 引导式对位（保底） | UI 显示从臂目标位姿，人手把主臂摆到位，客户端比对 `get_action` 与目标、误差<阈值自动通过 | 零（全程人手、无扭矩） | 半天 |

建议：先 B 上线（满足"对位"语义、零风险），A 作为增强并行开发。

## 4. r2c 集成缺口（小改）

| 缺口 | 说明 | 改动 |
|------|------|------|
| 类路径 | `make_robot_from_config` 只建 `Robot`（从臂）；`SOLeader` 是 `Teleoperator`，不在其覆盖范围 | 适配器按配置 `robot.type=so101_leader` 走 `make_teleoperator_from_config` 分支 |
| 方法名 | 适配器调 `get_observation()`/`send_action()`；leader 是 `get_action()`/`send_feedback()` | 适配分支内做方法映射（`get_action()` 结果按 `.pos` 键进 `get_runtime()` 现有路径；对位走 `send_feedback`） |
| 扭矩控制 | 对位需要 `enable_torque/disable_torque` 透传 | adapter 增加两个透传方法 + 对位小步插值工具函数 |
| 版本 pin | r2c 未 pin lerobot 库版本；v0.4.x（`lerobot.teleoperators.so101_leader`）与 main（`lerobot.teleoperators.so_leader`）类路径不同 | 建议 pin（如 `lerobot>=0.4,<0.5`）或两条路径 import 兼容 |

## 5. 结论

1. **单独读取关节角度：✅** `SOLeader.get_action()` 一行总线读，官方有单臂先例，r2c 的 `.pos` 键通路已兼容，当天可验证；
2. **对位控制：✅ 可行**——库已暴露 `enable_torque` + `send_feedback`（Goal_Position），且 lerobot 官方 DAgger handover 就是"主臂程序化对到从臂位姿"；建议先上引导式对位（B），程序化对位（A）作为增强并配低速/限程/急停；
3. r2c 接入需一个小的 leader 适配分支（`Teleoperator` 方法映射 + 扭矩透传 + 版本 pin），预估 1 天内完成。

## 参考

- lerobot main：`src/lerobot/teleoperators/so_leader/so_leader.py`（SOLeader/SO101Leader，get_action/send_feedback/enable_torque）、`src/lerobot/teleoperators/bi_so_leader/bi_so_leader.py`（DAgger handover 对位注释）、`examples/isaac_teleop_to_so101/isaac_teleop/teleop_so101_leader_arm.py`（单臂关节直读）
- lerobot v0.4.0：`src/lerobot/teleoperators/so101_leader/so101_leader.py`（类路径，与 main 不同）、`src/lerobot/robots/so101_follower/so101_follower.py`（Robot 接口对比）
- 官方文档：`docs/source/so101.mdx`（校准流程、舵机与减速比表、`lerobot-find-port`）
- r2c 现状：`packages/cloudrobo-r2c/src/cloudrobo_r2c/robots/lerobot_hardware_adapter.py`（`get_runtime` so101 `.pos` 键缺省、`runtime.robot_calibrate` 开关、`make_robot_from_config` 工厂）
