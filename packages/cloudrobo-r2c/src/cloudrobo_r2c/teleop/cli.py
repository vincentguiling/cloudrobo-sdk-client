"""`cloudrobo r2c teleop ...` command group.

Commands (design §3.3.4):
  gateway       operator runtime: WebXR HUD page + /ws + Zenoh + heartbeat
  mock          convenience alias for `gateway --input mock` (headset-free)
  bench         subscribe teleop/actions and report rate/latency/frame size
  wsdriver      simulated HUD client driving the gateway
  state-inspect diagnose state/current rate and pose fields
  cloud-cmd     debug remote commands (start/pause/resume/finish/quit)

Heavy dependencies (aiohttp, eclipse-zenoh) are imported inside the command
bodies so `cloudrobo r2c --help` stays fast on machines without the teleop
extras installed.
"""

from __future__ import annotations

import asyncio

import click


@click.group()
def teleop() -> None:
    """Teleop operator runtime (WebXR HUD + mock device)."""


def _build_config(
    config_path: str | None,
    *,
    bundle: str | None = None,
    input_source: str | None = None,
    mapping: str | None = None,
    rate: float | None = None,
    port: int | None = None,
    endpoints: str | None = None,
    auto_session: bool | None = None,
):
    from .config import TeleopConfig, apply_bundle

    if config_path:
        cfg = TeleopConfig.load(config_path)
    else:
        cfg = TeleopConfig()
    if input_source:
        cfg.input_source = input_source
    if mapping:
        cfg.mapping = mapping
    if rate:
        cfg.actions_rate_hz = float(rate)
    if port:
        cfg.http_port = int(port)
    if endpoints:
        eps = [e.strip() for e in endpoints.split(",") if e.strip()]
        if eps:
            cfg.zenoh.endpoints = eps
    if auto_session is not None:
        cfg.auto_start = auto_session
    if bundle:
        cfg = apply_bundle(cfg, bundle)
    return cfg


def _run_gateway(cfg, *, cloud_mode: bool) -> None:
    from .gateway import run_gateway

    asyncio.run(run_gateway(cfg, cloud_mode=cloud_mode))


@teleop.command("gateway")
@click.option("--config", "config_path", default=None,
              help="operator config (YAML or JSON); built-in defaults otherwise")
@click.option("--bundle", default=None,
              help="r2c credential bundle (zip or dir); enables cloud-controlled sessions")
@click.option("--input", "input_source", default=None,
              help="input source: webxr (default) | mock | entry point name | module:Class")
@click.option("--mapping", default=None,
              help="mapping strategy: relative_pose | passthrough | entry point name")
@click.option("--rate", type=float, default=None, help="actions rate in Hz (default 20)")
@click.option("--port", type=int, default=None, help="HTTP/WS port (default 8000)")
@click.option("--endpoints", default=None,
              help="comma-separated zenoh endpoints, e.g. tcp/127.0.0.1:7447")
@click.option("--auto-session/--no-auto-session", default=None,
              help="start the session automatically (default: on for mock)")
def gateway_cmd(config_path, bundle, input_source, mapping, rate, port, endpoints,
                auto_session) -> None:
    """Run the operator gateway (HTTP + /ws + Zenoh publishers)."""
    try:
        if auto_session is None and input_source == "mock":
            auto_session = True
        cfg = _build_config(
            config_path,
            bundle=bundle,
            input_source=input_source,
            mapping=mapping,
            rate=rate,
            port=port,
            endpoints=endpoints,
            auto_session=auto_session,
        )
        _run_gateway(cfg, cloud_mode=bool(bundle))
    except KeyboardInterrupt:
        pass


@teleop.command("mock")
@click.option("--config", "config_path", default=None)
@click.option("--bundle", default=None)
@click.option("--mapping", default=None)
@click.option("--rate", type=float, default=None)
@click.option("--port", type=int, default=None)
@click.option("--endpoints", default=None)
@click.option("--amplitude", type=float, default=None,
              help="wobble radius in metres (default 0.02)")
def mock_cmd(config_path, bundle, mapping, rate, port, endpoints, amplitude) -> None:
    """Run the mock teleop device (state/current + small up/down/left/right offset)."""
    try:
        cfg = _build_config(
            config_path,
            bundle=bundle,
            input_source="mock",
            mapping=mapping,
            rate=rate,
            port=port,
            endpoints=endpoints,
            auto_session=True,
        )
        if amplitude is not None:
            cfg.input_params["amplitude_m"] = float(amplitude)
        _run_gateway(cfg, cloud_mode=bool(bundle))
    except KeyboardInterrupt:
        pass


@teleop.command("bench")
@click.option("--seconds", type=float, default=20.0)
@click.option("--label", default="")
@click.option("--endpoints", default=None)
@click.option("--project-id", default="r2c")
@click.option("--device-id", default="local")
def bench_cmd(seconds, label, endpoints, project_id, device_id) -> None:
    """Subscribe teleop/actions and report rate/latency/frame size."""
    from .tools.bench import run_bench

    run_bench(
        seconds=seconds,
        label=label,
        endpoints=endpoints,
        project_id=project_id,
        device_id=device_id,
    )


@teleop.command("wsdriver")
@click.option("--port", type=int, default=8000)
@click.option("--seconds", type=float, default=15.0)
@click.option("--rate", type=float, default=20.0)
def wsdriver_cmd(port, seconds, rate) -> None:
    """Drive the gateway with a simulated WebXR HUD (no headset needed)."""
    from .tools.wsdriver import run_wsdriver

    run_wsdriver(port=port, seconds=seconds, rate=rate)


@teleop.command("state-inspect")
@click.option("--endpoint", default="tcp/127.0.0.1:7447")
@click.option("--project-id", default="r2c")
@click.option("--device-id", default="local")
@click.option("--seconds", type=float, default=10.0)
def state_inspect_cmd(endpoint, project_id, device_id, seconds):
    """Sample state/current and print rate + decoded per-arm poses."""
    from .tools.state_inspect import run_state_inspect

    run_state_inspect(
        endpoint=endpoint,
        project_id=project_id,
        device_id=device_id,
        seconds=seconds,
    )


@teleop.command("cloud-cmd")
@click.argument("kind", type=click.Choice(["start", "pause", "resume", "finish", "quit"]))
@click.option("--session", "session_id", default="")
@click.option("--endpoints", default=None)
@click.option("--project-id", default="r2c")
@click.option("--device-id", default="local")
@click.option("--operator-id", default="cloud-console")
@click.option("--pos-left", default=None, help="9 values, row-major")
@click.option("--rot-left", default=None)
@click.option("--pos-right", default=None)
@click.option("--rot-right", default=None)
def cloud_cmd_cmd(kind, session_id, endpoints, project_id, device_id, operator_id,
                  pos_left, rot_left, pos_right, rot_right) -> None:
    """Publish a remote lifecycle command (debug the cloud-driven session)."""
    from .tools.cloud_cmd import publish_cloud_command

    def mat(v):
        if not v:
            return None
        vals = [float(x) for x in v.replace(",", " ").split()]
        return vals if len(vals) == 9 else None

    publish_cloud_command(
        kind=kind,
        session_id=session_id,
        endpoints=endpoints,
        project_id=project_id,
        device_id=device_id,
        operator_id=operator_id,
        pos_left=mat(pos_left),
        rot_left=mat(rot_left),
        pos_right=mat(pos_right),
        rot_right=mat(rot_right),
    )
