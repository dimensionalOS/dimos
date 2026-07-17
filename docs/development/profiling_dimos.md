---
title: "Profiling dimos"
---

You can use py-spy to profile a particular blueprint:

```bash
uv run py-spy record --format speedscope --subprocesses -o profile.speedscope.json -- python -m dimos.robot.cli.dimos run unitree-go2-agentic
```

Hit `Ctrl+C` when you're done. It will write a `profile.speedscope.json` file which you can upload to [speedscope.app](https://www.speedscope.app/) to visualize it.

Before using `py-spy`, it is often useful to first run the bounded subprocess smoke/perf harness in [dimos/robot/tool_blueprint_perf.py](/dimos/robot/tool_blueprint_perf.py). That gives you a cheap way to confirm that a blueprint starts at all, survives warmup, and captures basic CPU/memory signals plus bounded stdout/stderr tails.

```bash
uv run python -m dimos.robot.tool_blueprint_perf --blueprint unitree-go2 --mode replay --viewer none --output /tmp/unitree-go2.json
```

Use that tool to identify which blueprint or mode is slow or unstable first. Then switch to `py-spy` for deeper profiling of the specific command you care about.
