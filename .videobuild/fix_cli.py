# -*- coding: utf-8 -*-
"""cli.py 移除录制相关代码（临时脚本）。"""
import re, pathlib

p = pathlib.Path(r"packages/cloudrobo-r2c/src/cloudrobo_r2c/teleop/cli.py")
c = p.read_text(encoding="utf-8-sig")

c = c.replace("from cloudrobo_r2c.teleop.operator import OperatorConfig, TeleopOperator\nfrom cloudrobo_r2c.teleop.recording import TeleopSessionRecorder\n",
              "from cloudrobo_r2c.teleop.operator import OperatorConfig, TeleopOperator\n")
c = c.replace('publishes Actions to the ro\n            "bot, and records "\n            "(observation, action) pairs for data collection.',
              'publishes Actions to the ro\n            "bot.')
c = c.replace("(leader arm / VR), publishes Actions to the robot, and records \"\n            \"(observation, action) pairs for data collection.",
              "(leader arm / VR), publishes Actions to the robot.")
c = re.sub(r"\n    parser\.add_argument\(\n        \"--record\",\n.*?\n    \)\n", "\n", c, flags=re.S)
c = re.sub(r"\n    parser\.add_argument\(\n        \"--no-record\",\n.*?\n    \)\n", "\n", c, flags=re.S)
c = re.sub(r"\ndef _resolve_recording\(.*?\n\ndef run_teleop_operator", "\ndef run_teleop_operator", c, flags=re.S)
c = c.replace("    operator_config = OperatorConfig.from_cfg(teleop_cfg)\n\n    recorder = _resolve_recording(teleop_cfg, args)\n    session = None",
              "    operator_config = OperatorConfig.from_cfg(teleop_cfg)\n\n    session = None")
c = c.replace("        operator = TeleopOperator(\n            session,\n            operator_config,\n            recorder=recorder,\n        )",
              "        operator = TeleopOperator(session, operator_config)")
c = c.replace("    finally:\n        if recorder is not None:\n            recorder.close()\n        if session is not None:",
              "    finally:\n        if session is not None:")
p.write_text(c, encoding="utf-8")
print("cli.py rewritten")
