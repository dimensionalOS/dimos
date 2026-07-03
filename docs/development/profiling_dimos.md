---
title: "Profiling DimOS blueprints with py-spy"
description: "Profile a DimOS blueprint with py-spy and speedscope, capture a flamegraph across subprocesses, and visualize hotspots on speedscope.app."
---

You can use py-spy to profile a particular blueprint:

```bash
uv run py-spy record --format speedscope --subprocesses -o profile.speedscope.json -- python -m dimos.robot.cli.dimos run unitree-go2-agentic
```

Hit `Ctrl+C` when you're done. It will write a `profile.speedscope.json` file which you can upload to [speedscope.app](https://www.speedscope.app/) to visualize it.
