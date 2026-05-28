# PawTrack Greeter Hackathon Demo

## Overview

PawTrack adds a DimOS / Unitree Go2 autonomous greeter demo. The robot patrols, detects a target such as a person sitting on a chair, tracks it, approaches to a safe standoff, waves, records the visited location, and resumes patrol. The DimOS blueprint is registered as `greeter-agentic`.

Source lives in `hackathon/pawtrack/src/pawtrack`. The runnable DimOS blueprint is placed at `dimos/robot/unitree/go2/blueprints/agentic/greeter_agentic.py` and registered in `dimos/robot/all_blueprints.py`.

## Run

From this DimOS checkout:

```bash
export DIMOS_HOME="${DIMOS_HOME:-$HOME/sam/dimos}"
export DIMOS_VENV="${DIMOS_VENV:-$HOME/dimos-env}"
export PYTHONPATH="$DIMOS_HOME/hackathon/pawtrack/src:$DIMOS_HOME:${PYTHONPATH:-}"
source "$DIMOS_VENV/bin/activate"
cd "$DIMOS_HOME"
```

Simulation:

```bash
hackathon/pawtrack/scripts/run_greeter_sim.sh
```

Then in another shell:

```bash
dimos mcp call start_greeting --arg target="a person"
dimos mcp call greeter_status
dimos mcp call stop_greeting
```

Real Go2:

```bash
dimos run greeter-agentic --robot-ip <robot_ip>
dimos mcp call start_greeting --arg target="a person sitting on a chair"
dimos mcp call stop_greeting
```

## Test

```bash
export DIMOS_HOME="${DIMOS_HOME:-$HOME/sam/dimos}"
export DIMOS_VENV="${DIMOS_VENV:-$HOME/dimos-env}"
source "$DIMOS_VENV/bin/activate"
cd "$DIMOS_HOME"
PYTHONPATH="$DIMOS_HOME/hackathon/pawtrack/src:$DIMOS_HOME" pytest -o addopts= \
  hackathon/pawtrack/tests/test_identify.py \
  hackathon/pawtrack/tests/test_greeter_state.py \
  hackathon/pawtrack/tests/test_greeter_container.py \
  hackathon/pawtrack/tests/test_occupancy.py
```
