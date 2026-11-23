"""遥操操作端 CLI 入口（``cloudrobo r2c teleop``）。

会话参数与 ``cloudroboclient`` 完全一致（同一套 ``build_session``），
因此现场遥操（局域网直连）与远程遥操（互联网经云端）只是 endpoint /
凭据不同，遥操配置本身不变：

- 现场遥操：机器人 ``--endpoint-role listen --endpoints tcp/0.0.0.0:7447``，
  操作端 ``--endpoints tcp/<机器人IP>:7447``（peer 直连）。
- 远程遥操：两端均连云端（操作端 ``--bundle`` 平台证书，或指向云端
  zenoh 接入点的 client-config）；相机画面走 LiveKit 推拉流
  （``camera_streaming``，云端下发 CAMERA_START 即推流）。
"""

from __future__ import annotations

import argparse
import logging
import os
from logging.handlers import RotatingFileHandler
from pathlib import Path
from typing import Any, Mapping, Optional, Sequence

import yaml

from cloudrobo_r2c.common.config_path import resolve_config_path
from cloudrobo_r2c.teleop.operator import OperatorConfig, TeleopOperator

logger = logging.getLogger(__name__)


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run the R2C teleoperation operator: reads a teleop source "
            "(leader arm / VR), publishes Actions to the robot."
        )
    )
    parser.add_argument(
        "--teleop-config",
        required=True,
        help="Path to the teleop operator config YAML (teleop: source/action/safety).",
    )

    # ── 会话参数（与 cloudroboclient 对齐，build_session 复用） ────────
    parser.add_argument(
        "--bundle",
        type=str,
        default=None,
        help="Path to the platform-issued credential bundle zip (remote teleop).",
    )
    parser.add_argument(
        "--client-config",
        default=None,
        help="Path to the R2C client config YAML (LAN teleop or explicit endpoints).",
    )
    parser.add_argument("--project-id", type=str, default="r2c", help="Project ID")
    parser.add_argument("--device-id", type=str, default="local", help="Target robot device ID")
    parser.add_argument("--client-id", type=str, default=None, help="Client ID")
    parser.add_argument(
        "--endpoints",
        type=str,
        default="",
        help="Comma-separated zenoh endpoints, e.g. tcp/192.168.1.100:7447 (LAN teleop).",
    )
    parser.add_argument(
        "--mode",
        type=str,
        default=None,
        choices=["peer", "client"],
        help="Connection mode (default: peer).",
    )
    parser.add_argument(
        "--endpoint-role",
        type=str,
        default=None,
        choices=["connect", "listen"],
        help="Zenoh endpoint role: 'connect' (active, default) or 'listen' (passive).",
    )
    parser.add_argument(
        "--private-key-password",
        default=None,
        help="Password for encrypted private key in the bundle.",
    )
    parser.add_argument(
        "--private-key-password-env",
        default=None,
        help="Read private key password from this environment variable name.",
    )
    parser.add_argument(
        "--no-prompt-password",
        dest="prompt_password",
        action="store_false",
        help="Never prompt for private key password; fail if one is required.",
    )
    parser.set_defaults(prompt_password=True)

    # ── 遥操运行参数 ─────────────────────────────────────────────────
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Run duration in seconds; 0 means run until Ctrl+C.",
    )
    parser.add_argument(
        "--log-level",
        type=str,
        default="INFO",
        help="Python logging level, e.g. DEBUG/INFO/WARNING.",
    )
    parser.add_argument(
        "--log-file",
        type=str,
        default=None,
        help="Optional path to a rotating log file (100 MB max, 5 backups).",
    )
    return parser.parse_args(argv)


def load_teleop_config(path: str | Path) -> Mapping[str, Any]:
    """加载遥操配置（顶层 ``teleop`` 节）。"""
    file_path = Path(path)
    with file_path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, Mapping):
        raise ValueError(f"Teleop config must contain a mapping: {file_path}")
    teleop_cfg = data.get("teleop")
    if not isinstance(teleop_cfg, Mapping):
        raise ValueError(f"Teleop config missing top-level 'teleop' mapping: {file_path}")
    return teleop_cfg


def run_teleop_operator(args: argparse.Namespace) -> None:
    from cloudrobo_r2c.cloudroboclient import build_session

    args.teleop_config = resolve_config_path(args.teleop_config)
    args.bundle = resolve_config_path(args.bundle)
    args.client_config = resolve_config_path(args.client_config)

    teleop_cfg = load_teleop_config(args.teleop_config)
    operator_config = OperatorConfig.from_cfg(teleop_cfg)

    session = None
    try:
        session = build_session(args)
        operator = TeleopOperator(session, operator_config)
        logger.info(
            "Starting teleop operator (target device=%s/%s, duration=%.1fs)",
            session.project_id,
            session.device_id,
            float(args.duration),
        )
        operator.run(duration_s=float(args.duration))
    finally:
        if session is not None:
            try:
                session.close()
            except Exception:
                logger.debug("session close failed (ignored)")


def main(argv: Optional[Sequence[str]] = None) -> None:
    args = parse_args(argv)
    log_level = getattr(logging, str(args.log_level).upper(), logging.INFO)
    log_format = "%(asctime)s %(levelname)s [%(name)s] %(message)s"
    handlers: list[logging.Handler] = [logging.StreamHandler()]
    if args.log_file:
        log_path = Path(args.log_file)
        log_path.parent.mkdir(parents=True, exist_ok=True)
        handlers.append(RotatingFileHandler(log_path, maxBytes=100 * 1024 * 1024, backupCount=5))
    logging.basicConfig(level=log_level, format=log_format, handlers=handlers)

    try:
        run_teleop_operator(args)
    except KeyboardInterrupt:
        logger.info("Teleop stopped by keyboard interrupt")
    except (ValueError, FileNotFoundError) as exc:
        if logger.isEnabledFor(logging.DEBUG):
            logger.debug("teleop operator exited with recoverable error", exc_info=True)
        logger.error("%s", exc)
        raise SystemExit(1)


if __name__ == "__main__":
    main()
