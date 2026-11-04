"""TeleopOperator：遥操操作端主循环（读源 → 映射 → 发布 Actions → 采集）。

链路::

    TeleopSource.read() ──► 目标映射（joint_map / ee_pose_map）──► Actions
                                                                  │
    机器人 Observations（订阅）──► 最新观测快照 ──► TeleopSessionRecorder

安全策略：

- ``require_observation``（默认开）：未收到任何机器人观测前不发动作。
- ``observation_timeout_s``（默认 2.0）：观测中断超时后停发动作（遥操端
  与机器人失去联络时不再盲发），恢复收到观测后自动继续。
- 源 ``stale``（如 VR 数据流中断）时停发动作，恢复后自动继续。

动作以 ``chunk_size=1`` 单步发布，``id`` 单调递增；频率 ``action_hz``。
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Mapping, Optional, Sequence, Tuple

from cloudrobo_r2c.common.models import Actions

from .base import TeleopSource, TeleopTarget, create_source
from .mapping import build_ee_action, build_joint_action, observation_to_dict
from .recording import TeleopSessionRecorder

logger = logging.getLogger(__name__)


@dataclass
class GripperActionConfig:
    """夹爪输出配置（末端位姿模式或 VR+关节模式）。"""

    emit: str = "joint"  # joint | end_effector_state | none
    joint_name: str = "gripper"
    range: Tuple[float, float] = (0.0, 1.0)
    source_hand: str = ""

    @classmethod
    def from_cfg(cls, cfg: Optional[Mapping[str, Any]]) -> "GripperActionConfig":
        cfg = dict(cfg or {})
        emit = str(cfg.get("emit", "joint")).lower()
        if emit not in ("joint", "end_effector_state", "none"):
            raise ValueError(f"action.gripper.emit must be joint|end_effector_state|none, got {emit!r}")
        rng = cfg.get("range") or [0.0, 1.0]
        values = [float(v) for v in rng]
        if len(values) != 2:
            raise ValueError("action.gripper.range must have two values [closed, open]")
        return cls(
            emit=emit,
            joint_name=str(cfg.get("joint_name", "gripper")),
            range=(values[0], values[1]),
            source_hand=str(cfg.get("source_hand", "")),
        )


@dataclass
class OperatorConfig:
    """operator 配置（``teleop_config.yaml`` 的 ``teleop`` 节）。"""

    source_cfg: Mapping[str, Any] = field(default_factory=dict)
    mode: str = "joint"  # joint | end_effector_pose
    action_hz: float = 30.0
    joint_map: Dict[str, str] = field(default_factory=dict)  # 源关节键 → R2C 关节名
    joint_scale: Dict[str, float] = field(default_factory=dict)  # 源关节键 → 缩放
    joint_default_scale: float = 1.0
    ee_pose_map: Dict[str, str] = field(default_factory=dict)  # 源键(手) → R2C 末端名
    gripper: GripperActionConfig = field(default_factory=GripperActionConfig)
    gripper_joint: str = ""  # joint 模式：target.gripper 输出为该 R2C 关节（空=关闭）
    require_observation: bool = True
    observation_timeout_s: float = 2.0

    @classmethod
    def from_cfg(cls, cfg: Mapping[str, Any]) -> "OperatorConfig":
        cfg = dict(cfg or {})
        action_cfg = dict(cfg.get("action") or {})
        mode = str(action_cfg.get("mode", "joint")).lower()
        if mode not in ("joint", "end_effector_pose"):
            raise ValueError(f"teleop.action.mode must be joint|end_effector_pose, got {mode!r}")
        action_hz = float(action_cfg.get("action_hz", 30.0))
        if not (0.0 < action_hz <= 500.0):
            raise ValueError(f"teleop.action.action_hz out of range (0, 500]: {action_hz}")

        def _name_map(key: str) -> Dict[str, str]:
            raw = action_cfg.get(key) or {}
            if not isinstance(raw, Mapping):
                raise ValueError(f"teleop.action.{key} must be a mapping")
            return {str(k): str(v) for k, v in raw.items()}

        joint_scale_raw = action_cfg.get("joint_scale") or {}
        if not isinstance(joint_scale_raw, Mapping):
            raise ValueError("teleop.action.joint_scale must be a mapping")
        joint_scale = {str(k): float(v) for k, v in joint_scale_raw.items()}

        safety_cfg = dict(cfg.get("safety") or {})
        observation_timeout = float(safety_cfg.get("observation_timeout_s", 2.0))
        require_observation = bool(safety_cfg.get("require_observation", True))

        config = cls(
            source_cfg=dict(cfg.get("source") or {}),
            mode=mode,
            action_hz=action_hz,
            joint_map=_name_map("joint_map"),
            joint_scale=joint_scale,
            ee_pose_map=_name_map("ee_pose_map"),
            gripper=GripperActionConfig.from_cfg(action_cfg.get("gripper")),
            gripper_joint=str(action_cfg.get("gripper_joint", "")),
            require_observation=require_observation,
            observation_timeout_s=observation_timeout,
        )
        config.validate()
        return config

    def validate(self) -> None:
        if self.mode == "joint" and not self.joint_map:
            raise ValueError(
                "teleop.action.joint_map is required in joint mode "
                "(source joint key → robot R2C joint name)"
            )
        if self.mode == "end_effector_pose" and not self.ee_pose_map:
            raise ValueError(
                "teleop.action.ee_pose_map is required in end_effector_pose mode "
                "(source key (hand) → Actions.end_effector_poses key)"
            )


class TeleopOperator:
    """遥操操作端：持有源、订阅观测、按频率发布动作并驱动录制。"""

    def __init__(
        self,
        session: Any,
        config: OperatorConfig,
        *,
        source: Optional[TeleopSource] = None,
        recorder: Optional[TeleopSessionRecorder] = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        config.validate()
        self._session = session
        self._config = config
        self._source = source if source is not None else create_source(config.source_cfg)
        self._recorder = recorder
        self._clock = clock

        self._obs_lock = threading.Lock()
        self._latest_observation: Optional[Dict[str, Any]] = None
        self._last_observation_clock: Optional[float] = None

        self._seq = 0
        self._published = 0
        self._skipped = 0
        self._last_status_clock: Optional[float] = None
        self._last_skip_log_clock: Optional[float] = None
        self._running = False
        self._subscription_started = False

    # ── 属性 ─────────────────────────────────────────────────────────

    @property
    def published_count(self) -> int:
        return self._published

    @property
    def skipped_count(self) -> int:
        return self._skipped

    @property
    def source(self) -> TeleopSource:
        return self._source

    # ── 生命周期 ─────────────────────────────────────────────────────

    def start(self) -> None:
        """订阅机器人观测并启动输入源。"""
        if self._subscription_started:
            return
        self._session.subscribe_observations(self._on_observation, decode_images=False)
        self._subscription_started = True
        self._source.start()
        logger.info(
            "Teleop operator started: source=%s mode=%s action_hz=%.1f",
            self._source.describe(),
            self._config.mode,
            self._config.action_hz,
        )

    def close(self) -> None:
        self._running = False
        try:
            self._source.stop()
        except Exception:
            logger.warning("teleop source stop failed", exc_info=True)
        if self._recorder is not None:
            self._recorder.close()
            self._recorder = None

    def run(self, duration_s: float = 0.0) -> None:
        """按 ``action_hz`` 循环发布；``duration_s<=0`` 表示一直运行。

        KeyboardInterrupt 由调用方（CLI）处理为优雅退出。
        """
        self.start()
        self._running = True
        interval = 1.0 / self._config.action_hz
        start_clock = self._clock()
        while self._running:
            loop_start = self._clock()
            self.tick()
            if duration_s > 0.0 and (self._clock() - start_clock) >= duration_s:
                break
            elapsed = self._clock() - loop_start
            sleep_s = interval - elapsed
            if sleep_s > 0:
                time.sleep(sleep_s)

    # ── 观测订阅 ─────────────────────────────────────────────────────

    def _on_observation(self, observation: Any) -> None:
        flat = observation_to_dict(observation)
        with self._obs_lock:
            self._latest_observation = flat
            self._last_observation_clock = self._clock()

    def observation_age_s(self) -> Optional[float]:
        with self._obs_lock:
            last = self._last_observation_clock
        if last is None:
            return None
        return self._clock() - last

    def _snapshot_observation(self) -> Optional[Dict[str, Any]]:
        with self._obs_lock:
            return self._latest_observation

    # ── 单 tick ──────────────────────────────────────────────────────

    def tick(self) -> Optional[Actions]:
        """读取源一次并（在允许时）发布一个动作；返回发布的 Actions。"""
        target = self._safe_read_source()
        if target is None or target.stale:
            self._skip("input unavailable" if target is None else "input stale")
            return None

        age = self.observation_age_s()
        if age is None:
            if self._config.require_observation:
                self._skip("no observation received yet")
                return None
        elif (
            self._config.observation_timeout_s > 0.0
            and age > self._config.observation_timeout_s
        ):
            self._skip(f"observation stale ({age:.1f}s)")
            return None

        observation = self._snapshot_observation()
        action = self.build_action(target)
        if action is None:
            # 关节向量缺项：publish 会造成 position 下标与 r2c_to_device
            # 映射错位，必须整 tick 跳过，不能发残缺向量。
            self._skip("source missing required joint values")
            return None
        self._session.publish_actions(action)
        self._seq += 1
        self._published += 1
        if self._recorder is not None:
            try:
                self._recorder.record(observation, _action_snapshot(action), target)
            except Exception:
                logger.exception("teleop recorder failed; continuing without this frame")
        self._log_status(age)
        return action

    def _safe_read_source(self) -> Optional[TeleopTarget]:
        try:
            return self._source.read()
        except Exception as exc:
            now = self._clock()
            if self._last_skip_log_clock is None or now - self._last_skip_log_clock > 2.0:
                self._last_skip_log_clock = now
                logger.warning("teleop source read failed: %s", exc)
            return None

    # ── 映射 ─────────────────────────────────────────────────────────

    def build_action(self, target: TeleopTarget) -> Optional[Actions]:
        """TeleopTarget → Actions（chunk_size=1，id 单调递增）。

        返回 None 表示本 tick 无法构成完整动作（joint 模式下关节缺项），
        调用方应跳过发布。
        """
        timestamp_ms = int(time.time() * 1000)
        if self._config.mode == "joint":
            return self._build_joint(target, timestamp_ms)
        return self._build_ee(target, timestamp_ms)

    def _build_joint(self, target: TeleopTarget, timestamp_ms: int) -> Optional[Actions]:
        joint_map = self._config.joint_map
        names: List[str] = []
        positions: List[float] = []
        missing: List[str] = []
        for source_key, r2c_name in joint_map.items():
            value = target.joint_positions.get(source_key)
            if value is None:
                missing.append(source_key)
                continue
            scale = self._config.joint_scale.get(source_key, self._config.joint_default_scale)
            names.append(r2c_name)
            positions.append(float(value) * scale)
        if missing:
            # 位置向量必须与机器人侧 r2c_to_device 的 source_index 一一
            # 对齐，缺项时不允许发布残缺向量。
            now = self._clock()
            if self._last_skip_log_clock is None or now - self._last_skip_log_clock > 2.0:
                self._last_skip_log_clock = now
                logger.warning(
                    "teleop source is missing joint values for %s; skipping tick", missing
                )
            return None
        if (
            self._config.gripper_joint
            and target.gripper is not None
            and self._config.gripper.emit != "none"
        ):
            lo, hi = self._config.gripper.range
            names.append(self._config.gripper_joint)
            positions.append(lo + (hi - lo) * _clamp01(target.gripper))
        return build_joint_action(
            timestamp_ms=timestamp_ms, seq=self._seq, names=names, positions=positions
        )

    def _build_ee(self, target: TeleopTarget, timestamp_ms: int) -> Optional[Actions]:
        pose_map: Dict[str, Sequence[float]] = {}
        missing: List[str] = []
        for source_key, ee_name in self._config.ee_pose_map.items():
            pose = target.end_effector_poses.get(source_key)
            if pose is None:
                missing.append(source_key)
                continue
            pose_map[ee_name] = pose
        if not pose_map:
            now = self._clock()
            if self._last_skip_log_clock is None or now - self._last_skip_log_clock > 2.0:
                self._last_skip_log_clock = now
                logger.warning(
                    "teleop source has no end effector poses "
                    f"(source keys: {sorted(target.end_effector_poses)}); skipping tick"
                )
            return None
        if missing:
            # 末端位姿按 map 键路由，单手缺位姿只影响该手，其余照常发布。
            now = self._clock()
            if self._last_skip_log_clock is None or now - self._last_skip_log_clock > 2.0:
                self._last_skip_log_clock = now
                logger.warning(
                    "teleop source is missing end effector poses for %s; publishing remaining",
                    missing,
                )
        return build_ee_action(
            timestamp_ms=timestamp_ms,
            seq=self._seq,
            pose_map=pose_map,
            gripper=target.gripper,
            gripper_emit=self._config.gripper.emit,
            gripper_joint_name=self._config.gripper.joint_name,
            gripper_range=self._config.gripper.range,
        )

    # ── 日志 ─────────────────────────────────────────────────────────

    def _skip(self, reason: str) -> None:
        self._skipped += 1
        now = self._clock()
        if self._last_skip_log_clock is None or now - self._last_skip_log_clock > 2.0:
            self._last_skip_log_clock = now
            logger.warning("teleop tick skipped: %s (skipped=%d)", reason, self._skipped)

    def _log_status(self, observation_age: Optional[float]) -> None:
        now = self._clock()
        if self._last_status_clock is None or now - self._last_status_clock >= 10.0:
            self._last_status_clock = now
            age_text = "n/a" if observation_age is None else f"{observation_age:.2f}s"
            logger.info(
                "teleop status: published=%d skipped=%d observation_age=%s",
                self._published,
                self._skipped,
                age_text,
            )


def _clamp01(value: float) -> float:
    return 0.0 if value < 0.0 else (1.0 if value > 1.0 else float(value))


def _action_snapshot(action: Actions) -> Dict[str, Any]:
    """发布动作的录制快照（dict 形态，含关节/末端数据）。"""
    snapshot: Dict[str, Any] = {
        "timestamp": action.timestamp,
        "id": action.id,
        "chunk_size": action.chunk_size,
    }
    if action.joint_states.names or action.joint_states.position:
        snapshot["joint_states"] = {
            "names": list(action.joint_states.names),
            "position": [list(step) for step in action.joint_states.position],
        }
    if action.end_effector_poses:
        snapshot["end_effector_poses"] = {
            name: [list(pose.data) for pose in chunk.pose]
            for name, chunk in action.end_effector_poses.items()
        }
    if action.end_effector_states.names or action.end_effector_states.position:
        snapshot["end_effector_states"] = {
            "names": list(action.end_effector_states.names),
            "position": [list(step) for step in action.end_effector_states.position],
        }
    return snapshot
