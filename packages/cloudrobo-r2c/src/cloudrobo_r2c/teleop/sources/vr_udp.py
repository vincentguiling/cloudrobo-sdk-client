"""vr_udp：VR 姿态 UDP JSON 数据流输入源（Vision Pro / Quest 等桥接接入）。

协议（每个 JSON 数据报为一帧，字段均可选，缺哪路就不输出那路）::

    {
      "ts": 1700000000000,                                   // 桥接端毫秒时间戳
      "left":  {"pos": [x,y,z], "quat": [x,y,z,w], "grip": 0.8, "pinch": 0.4},
      "right": {"pos": [x,y,z], "quat": [x,y,z,w], "grip": 0.8, "pinch": 0.4},
      "head":  {"pos": [x,y,z], "quat": [x,y,z,w]}
    }

约定：``pos`` 单位米、右手坐标系；``quat`` 顺序由 ``quat_order`` 配置
（默认 ``xyzw``）；``grip`` 0~1（1=张开）；``pinch`` 0~1（1=捏合），
同时存在时优先 ``grip``。

坐标变换（把 VR 手柄坐标转到机器人基座坐标系）：

1. 缩放：``p = scale * pos``（VR 工作区与机器人工作区大小差异）。
2. 平移：``p = p + workspace_offset``。
3. 轴重映射 ``axis_remap``：机器人轴 ← 源轴 + 符号，如
   ``{"x": "+y", "y": "-x", "z": "+z"}``；四元数按同一旋转矩阵
   ``R`` 变换（``q_robot = q_R ⊗ q_source``）。

无新数据超过 ``stale_after_s`` 即判 stale，operator 停发动作。
"""

from __future__ import annotations

import json
import logging
import math
import socket
import threading
import time
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

from ..base import TeleopSource, TeleopTarget, register_source

logger = logging.getLogger(__name__)

_IDENTITY_QUAT = (0.0, 0.0, 0.0, 1.0)  # x, y, z, w


@register_source("vr_udp")
def create_vr_udp(config: Mapping[str, Any]) -> TeleopSource:
    return VrUdpSource(config)


def _matrix_from_axis_remap(axis_remap: Mapping[str, str]) -> Optional[List[List[float]]]:
    """把轴重映射配置转换为 3x3 旋转矩阵 R（源坐标 → 机器人坐标）。

    每个机器人轴声明其来源轴与符号（如 ``"x": "+y"``）→ ``R[i][j] = 符号``。
    无配置返回 None（恒等）。
    """
    if not axis_remap:
        return None
    matrix = [[0.0] * 3 for _ in range(3)]
    axis_index = {"x": 0, "y": 1, "z": 2}
    for robot_axis, source_expr in axis_remap.items():
        ri = axis_index.get(str(robot_axis).lower())
        if ri is None:
            raise ValueError(f"axis_remap: unknown robot axis {robot_axis!r}")
        expr = str(source_expr).strip().lower()
        sign = 1.0
        if expr.startswith(("+", "-")):
            sign = -1.0 if expr[0] == "-" else 1.0
            expr = expr[1:]
        si = axis_index.get(expr)
        if si is None:
            raise ValueError(f"axis_remap: invalid source axis {source_expr!r}")
        matrix[ri][si] = sign
    return matrix


def _apply_matrix(matrix: Sequence[Sequence[float]], v: Sequence[float]) -> List[float]:
    """3x3 矩阵作用于三维向量。"""
    return [
        matrix[0][0] * v[0] + matrix[0][1] * v[1] + matrix[0][2] * v[2],
        matrix[1][0] * v[0] + matrix[1][1] * v[1] + matrix[1][2] * v[2],
        matrix[2][0] * v[0] + matrix[2][1] * v[1] + matrix[2][2] * v[2],
    ]


def _quat_from_axis_remap(axis_remap: Mapping[str, str]) -> Optional[Sequence[float]]:
    """轴重映射配置对应的旋转四元数 q（源坐标系 → 机器人坐标系）。"""
    matrix = _matrix_from_axis_remap(axis_remap)
    if matrix is None:
        return None
    return _quat_from_matrix(matrix)


def _quat_from_matrix(m: Sequence[Sequence[float]]) -> Sequence[float]:
    """3x3 旋转矩阵 → 四元数 (x, y, z, w)（Shepperd 方法）。"""
    m00, m01, m02 = m[0]
    m10, m11, m12 = m[1]
    m20, m21, m22 = m[2]
    trace = m00 + m11 + m22
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m21 - m12) / s
        y = (m02 - m20) / s
        z = (m10 - m01) / s
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        w = (m21 - m12) / s
        x = 0.25 * s
        y = (m01 + m10) / s
        z = (m02 + m20) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        w = (m02 - m20) / s
        x = (m01 + m10) / s
        y = 0.25 * s
        z = (m12 + m21) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        w = (m10 - m01) / s
        x = (m02 + m20) / s
        y = (m12 + m21) / s
        z = 0.25 * s
    return _quat_normalize((x, y, z, w))


def _quat_normalize(q: Sequence[float]) -> Tuple[float, float, float, float]:
    x, y, z, w = (float(v) for v in q)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 0.0:
        return _IDENTITY_QUAT
    return (x / norm, y / norm, z / norm, w / norm)


def _quat_multiply(a: Sequence[float], b: Sequence[float]) -> Tuple[float, float, float, float]:
    """四元数乘法 (x,y,z,w)：a ⊗ b。"""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _rotate_vector(q: Sequence[float], v: Sequence[float]) -> List[float]:
    """用四元数旋转三维向量（v' = q v q*）。"""
    x, y, z, w = q
    vx, vy, vz = v
    # t = 2 * (q_vec × v)
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    # v' = v + w*t + (q_vec × t)
    return [
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    ]


class VrUdpSource(TeleopSource):
    """监听 UDP 端口接收 VR 姿态流；``read()`` 输出末端位姿 + 夹爪开度。"""

    def __init__(self, config: Mapping[str, Any]) -> None:
        cfg = dict(config or {})
        self._listen_host = str(cfg.get("listen_host", "0.0.0.0"))
        self._listen_port = int(cfg.get("listen_port", 9870))
        self._stale_after_s = float(cfg.get("stale_after_s", 0.5))
        self._scale = float(cfg.get("scale", 1.0))
        offset = cfg.get("workspace_offset") or [0.0, 0.0, 0.0]
        if len(offset) != 3:
            raise ValueError("vr_udp.workspace_offset must have 3 values [x, y, z]")
        self._offset = [float(v) for v in offset]
        self._quat_order = str(cfg.get("quat_order", "xyzw")).lower()
        if self._quat_order not in ("xyzw", "wxyz"):
            raise ValueError(
                f"vr_udp.quat_order must be 'xyzw' or 'wxyz', got {cfg.get('quat_order')!r}"
            )
        self._hands = [str(h) for h in (cfg.get("hands") or ["left", "right"])]
        ee_map = cfg.get("ee_name_map") or {}
        self._ee_name_map = {str(k): str(v) for k, v in ee_map.items()}
        gripper_cfg = cfg.get("gripper") or {}
        self._gripper_source = str(gripper_cfg.get("source", "grip")).lower()
        if self._gripper_source not in ("grip", "pinch", "none"):
            raise ValueError(
                f"vr_udp.gripper.source must be grip|pinch|none, got {gripper_cfg.get('source')!r}"
            )
        self._gripper_hand = str(gripper_cfg.get("hand", self._hands[0] if self._hands else "right"))
        self._gripper_hand = self._gripper_hand or "right"

        self._q_remap = _quat_from_axis_remap(cfg.get("axis_remap") or {})
        self._matrix = _matrix_from_axis_remap(cfg.get("axis_remap") or {})

        self._lock = threading.Lock()
        self._latest: Dict[str, Dict[str, Any]] = {}
        self._latest_monotonic: Dict[str, float] = {}
        self._stop_event = threading.Event()
        self._socket: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._last_frame_monotonic: Optional[float] = None
        self._bound_port: Optional[int] = None

    @property
    def name(self) -> str:
        return "vr_udp"

    @property
    def bound_port(self) -> Optional[int]:
        """实际绑定端口（``listen_port: 0`` 时为内核分配的临时端口）。"""
        return self._bound_port

    # ── 生命周期 ─────────────────────────────────────────────────────

    def start(self) -> None:
        if self._thread is not None:
            return
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self._listen_host, self._listen_port))
        sock.settimeout(0.5)
        self._socket = sock
        self._bound_port = int(sock.getsockname()[1])
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._receive_loop, name="teleop-vr-udp", daemon=True
        )
        self._thread.start()
        logger.info(
            "vr_udp source listening on udp/%s:%d (hands=%s, stale_after_s=%.2f)",
            self._listen_host,
            self._listen_port,
            self._hands,
            self._stale_after_s,
        )

    def stop(self) -> None:
        self._stop_event.set()
        thread = self._thread
        if thread is not None:
            thread.join(timeout=2.0)
            self._thread = None
        sock = self._socket
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass
            self._socket = None

    def describe(self) -> Dict[str, Any]:
        return {
            "type": self.name,
            "listen": f"udp/{self._listen_host}:{self._listen_port}",
            "hands": self._hands,
            "scale": self._scale,
            "workspace_offset": self._offset,
        }

    # ── 接收线程 ─────────────────────────────────────────────────────

    def _receive_loop(self) -> None:
        assert self._socket is not None
        while not self._stop_event.is_set():
            try:
                payload, _addr = self._socket.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                if self._stop_event.is_set():
                    return
                raise
            try:
                frame = json.loads(payload.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                logger.warning("vr_udp: dropped malformed datagram (%d bytes)", len(payload))
                continue
            if not isinstance(frame, dict):
                continue
            now = time.monotonic()
            with self._lock:
                self._last_frame_monotonic = now
                for hand in self._hands:
                    hand_data = frame.get(hand)
                    if isinstance(hand_data, dict):
                        self._latest[hand] = hand_data
                        self._latest_monotonic[hand] = now
            logger.debug("vr_udp: frame from %s", _addr)

    # ── 读取 ─────────────────────────────────────────────────────────

    def read(self) -> Optional[TeleopTarget]:
        now = time.monotonic()
        with self._lock:
            snapshot = {h: self._latest.get(h) for h in self._hands}
            stamps = {h: self._latest_monotonic.get(h) for h in self._hands}
            last_frame = self._last_frame_monotonic
        if last_frame is None:
            return None  # 尚未收到任何数据

        target = TeleopTarget(source_timestamp_ms=int(time.time() * 1000))
        fresh_any = False
        for hand in self._hands:
            data = snapshot.get(hand)
            stamp = stamps.get(hand)
            if not data or stamp is None or (now - stamp) > self._stale_after_s:
                continue
            fresh_any = True
            pose = self._hand_pose(data)
            if pose is not None:
                target.end_effector_poses[self._ee_name_map.get(hand, hand)] = pose
            if hand == self._gripper_hand:
                target.gripper = self._hand_gripper(data)

        if not fresh_any:
            target.stale = True
        return target

    def _hand_pose(self, data: Mapping[str, Any]) -> Optional[List[float]]:
        pos = data.get("pos")
        if not isinstance(pos, (list, tuple)) or len(pos) != 3:
            return None
        x, y, z = (float(pos[0]) * self._scale + self._offset[0],
                   float(pos[1]) * self._scale + self._offset[1],
                   float(pos[2]) * self._scale + self._offset[2])
        xyz = [x, y, z]
        if self._matrix is not None:
            xyz = _apply_matrix(self._matrix, xyz)
        quat_raw = data.get("quat")
        if isinstance(quat_raw, (list, tuple)) and len(quat_raw) == 4:
            values = [float(v) for v in quat_raw]
            quat = tuple(values) if self._quat_order == "xyzw" else (values[1], values[2], values[3], values[0])
        else:
            quat = _IDENTITY_QUAT
        if self._q_remap is not None:
            quat = _quat_multiply(self._q_remap, quat)
        quat = _quat_normalize(quat)
        return [xyz[0], xyz[1], xyz[2], quat[0], quat[1], quat[2], quat[3]]

    def _hand_gripper(self, data: Mapping[str, Any]) -> Optional[float]:
        if self._gripper_source == "none":
            return None
        value = data.get("grip") if self._gripper_source == "grip" else data.get("pinch")
        if not isinstance(value, (int, float)):
            return None
        value = float(value)
        if self._gripper_source == "pinch":
            value = 1.0 - value  # pinch 1=捏合 → 开度 0
        return 0.0 if value < 0.0 else (1.0 if value > 1.0 else value)
