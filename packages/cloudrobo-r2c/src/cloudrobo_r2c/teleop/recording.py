"""遥操会话录制器：把（最新观测, 已发布动作）成对落盘。

采集数据形态（遥操数据集的最小单元，与 LeRobot 帧结构对齐）::

    {
      "timestamp_ms": int,      # 动作发布时刻（墙钟毫秒）
      "observation": {...},     # 当时收到的最新机器人观测（图像为编码 bytes）
      "action": {...},          # 本 tick 发布到机器人的 Actions dict
      "source": {...}           # 可选：输入源原始目标（调试/对齐用）
    }

文件格式复用 ``common.utils.safe_serialization``（R2CR，无代码执行风险，
支持 ndarray 与 bytes 块）；回放用 ``load_frames`` 读取。
"""

from __future__ import annotations

import logging
import time
from typing import Any, Dict, List, Mapping, Optional

from cloudrobo_r2c.common.utils.safe_serialization import save_frames

logger = logging.getLogger(__name__)


class TeleopSessionRecorder:
    """录制 (observation, action) 数据对到 R2CR 文件。

    用法::

        recorder = TeleopSessionRecorder("teleop_session.r2cr")
        recorder.record(observation_dict, action_dict, source_target)
        ...
        recorder.close()
    """

    def __init__(self, file_path: str, *, source_name: str = "") -> None:
        self._file_path = file_path
        self._source_name = str(source_name)
        self._frames: List[Dict[str, Any]] = []
        self._closed = False

    @property
    def frame_count(self) -> int:
        return len(self._frames)

    @property
    def file_path(self) -> str:
        return self._file_path

    def record(
        self,
        observation: Optional[Mapping[str, Any]],
        action: Mapping[str, Any],
        source_target: Optional[Any] = None,
    ) -> None:
        """追加一帧。``observation`` 为 None 时也记录动作（观测缺失期间）。"""
        if self._closed:
            raise RuntimeError("recorder already closed")
        frame: Dict[str, Any] = {
            "timestamp_ms": int(time.time() * 1000),
            "observation": dict(observation) if observation is not None else {},
            "action": _action_to_plain(action),
            "source": _source_to_plain(source_target),
        }
        if self._source_name:
            frame["source_name"] = self._source_name
        self._frames.append(frame)

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        save_frames(self._file_path, self._frames)
        logger.info(
            "Teleop recording saved: %d frames to %s", len(self._frames), self._file_path
        )


def _action_to_plain(action: Any) -> Dict[str, Any]:
    """把 Actions（或 dict）转为可序列化 dict（protobuf 之外仅基础类型）。"""
    if isinstance(action, Mapping):
        return {str(k): _plain(v) for k, v in action.items()}
    data = getattr(action, "__dict__", None)
    if isinstance(data, dict):
        return {str(k): _plain(v) for k, v in data.items()}
    return {"repr": repr(action)}


def _source_to_plain(target: Any) -> Dict[str, Any]:
    if target is None:
        return {}
    if isinstance(target, Mapping):
        return {str(k): _plain(v) for k, v in target.items()}
    data = getattr(target, "__dict__", None)
    if isinstance(data, dict):
        return {str(k): _plain(v) for k, v in data.items()}
    return {"repr": repr(target)}


def _plain(value: Any) -> Any:
    """轻量展平：Actions dict 里出现的模型对象转为基础类型。"""
    import dataclasses

    if isinstance(value, Mapping):
        return {str(k): _plain(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_plain(v) for v in value]
    if isinstance(value, (str, bool, int, float, bytes, bytearray, memoryview)) or value is None:
        return value
    module = getattr(type(value), "__module__", "") or ""
    if module.startswith("numpy"):
        return value
    if dataclasses.is_dataclass(value) and not isinstance(value, type):
        return _plain(dataclasses.asdict(value))
    data = getattr(value, "data", None)
    if isinstance(data, (list, tuple)):
        return [_plain(v) for v in data]
    return str(value)
