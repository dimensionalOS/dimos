# 5. CLI and Python API

Follow the shared setup in the release testing plan. Go2 commands without `--replay` or `--simulation` require a connected robot and room for the requested motion. Agentic blueprints also need their configured credentials. Run each foreground blueprint separately, and use another terminal for commands that interact with it.

## 5.1 Daemon lifecycle

```bash
uv run dimos --replay run unitree-go2 --daemon
uv run dimos status
uv run dimos log -n 50
uv run dimos log -f   # Ctrl+C
uv run dimos restart
# Wait for the restarted replay to appear in Rerun before stopping it.
uv run dimos stop
```

Expected result: status identifies the running instance, logs can be read and followed, restart brings the replay back in Rerun, and stop shuts it down. Run `uv run dimos status` afterward to confirm the instance is stopped.

## 5.2 MCP CLI

```bash
uv run dimos run unitree-go2-agentic --daemon
uv run dimos mcp call move_to --arg x=0.3 --arg relative=true
uv run dimos mcp call observe
uv run dimos stop
```

Expected result: the robot moves forward by about 30 cm, and observation returns an image from its camera.

## 5.3 Python API

Start the robot blueprint and open a Python REPL:

```bash
uv run dimos run unitree-go2-agentic --daemon
uv run python
```

In the REPL, call a skill and display a camera frame. Press a key in the image window to finish.

```python
import cv2
import numpy as np

from dimos import Dimos

app = Dimos.connect()
try:
    app.skills.move_to(x=0.3, relative=True)
    img = app.peek_stream("color_image", timeout=5.0)
    cv2.imshow("color_image", np.asarray(img.data))
    cv2.waitKey(0)
finally:
    cv2.destroyAllWindows()
    app.stop()
```

After leaving the REPL, stop the blueprint:

```bash
uv run dimos stop
```

Expected result: the robot moves forward by about 30 cm and the window shows its camera image.

## 5.4 dimos shell

```bash
uv run dimos --replay run unitree-go2 --daemon
uv run dimos shell
# After leaving the shell with Ctrl+D:
uv run dimos stop
```

...in the shell:

```python
app.ReplanningAStarPlanner.get_state()
```

Expected result: the planner returns `NavigationState.IDLE`, and Tab completes RPC names after `app.GO2Connection.`.

## 5.5 Module configuration

```bash
uv run dimos --replay run unitree-go2-agentic --daemon --disable speak-skill --mcpserver.port 9992
uv run dimos mcp modules
uv run dimos stop
```

Expected result: `dimos mcp modules` reaches the server on port 9992 and lists no `SpeakSkill`.

## 5.6 Transports and topics

Run the replay blueprint, then use another terminal for the other commands.
The LCM run also needs the host multicast and socket-buffer setup described in
[2.3](02_replay-and-simulation.md#23-dimsim-browser-simulation). If it was skipped
during installation, startup prompts to apply it with `sudo`; arrange
administrator access before starting the LCM run.

```bash
uv run dimos --replay run unitree-go2
uv run dimos spy   # q quits
uv run dimos topic echo /cmd_vel
# In a third terminal:
uv run dimos topic send /cmd_vel 'Twist(Vector3(0.5, 0, 0), Vector3(0, 0, 0))'
# Stop the blueprint, then run it on LCM:
uv run dimos --transport lcm --replay run unitree-go2
uv run dimos spy --transport lcm
```

Expected result: the spy lists the replay topics on zenoh with live rates, the sent Twist appears in the `/cmd_vel` echo, and on the LCM run the spy shows the same topics on LCM.

## 5.7 dtop

```bash
uv run dimos --dtop run unitree-go2
uv run dtop
```

Expected result: the monitoring interface opens and displays the running modules.

## 5.8 Rerun viewer modes

```bash
uv run dimos --replay run unitree-go2 --rerunbridgemodule.rerun-open web
uv run dimos --replay run unitree-go2 --rerunbridgemodule.rerun-open none --rerunbridgemodule.rerun-save true
uv run rerun recordings/<run-id>/rerun.rrd
uv run dimos --viewer none --replay run unitree-go2
# In another terminal:
uv run dimos rerun-bridge --rerun-open web
```

Expected result: the first run opens the web viewer at http://localhost:9878. The headless run opens no viewer and logs the `.rrd` path, which `rerun` plays back. The standalone bridge shows the streams of the run started with `--viewer none`.

## 5.9 Blueprint graph and docs site

Needs Graphviz.

```bash
uv run dimos graph unitree-go2-agentic --rpc --output /tmp/go2-agentic.svg
uv sync --group docs --inexact
uv run dimos docs
```

Expected result: `graph` writes an SVG with the modules grouped by package, their stream channels and the RPC contracts between them. `dimos docs` serves the docs site at http://127.0.0.1:8000 with navigation and search.

## 5.10 Host daemon

```bash
uv run dimos host doctor
uv run dimos host serve --name lab --tag gpu
# In another terminal:
uv run dimos host list
uv run dimos host describe lab
```

Expected result: `doctor` prints PASS for each check. `list` shows `lab` with tag `gpu` and state `available`, and `describe` prints its details. After Ctrl+C on `serve`, `uv run dimos host list` prints "No online Hosts found".

## 5.11 Native module examples

Needs nix with flakes. The first run of each example builds its binaries. Run the examples separately, stopping each with Ctrl+C before starting the next.
The C++ command below uses LCM, so it also needs the privileged multicast
setup described in [2.3](02_replay-and-simulation.md#23-dimsim-browser-simulation).
Arrange administrator access before running that command if setup was skipped
during installation.

```bash
uv run python examples/native-modules/rust_ping_pong.py --transport zenoh
uv run python examples/native-modules/cpp_ping_pong.py --transport lcm
uv run python examples/native-modules/rust_tf.py
```

Expected result: both ping-pong examples log "echo received" lines with a rising seq, and the tf example logs "tf lookup" lines.

## 5.12 External blueprint packages

```bash
# A package whose entry points name built-in dimos objects.
mkdir -p /tmp/my-robot-stack && cat > /tmp/my-robot-stack/pyproject.toml <<'EOF'
[project]
name = "my-robot-stack"
version = "0.1.0"
[project.entry-points."dimos.blueprints"]
go2 = "dimos.robot.unitree.go2.blueprints.smart.unitree_go2:unitree_go2"
keyboard-teleop = "dimos.robot.unitree.keyboard_teleop:KeyboardTeleop"
EOF
uv pip install /tmp/my-robot-stack
uv run dimos list
uv run dimos --replay run my-robot-stack.go2 my-robot-stack.keyboard-teleop
uv pip uninstall my-robot-stack
```

Expected result: `dimos list` prints an "External blueprints:" section with both entries, and the run starts the Go2 replay with the keyboard teleop window open.

## 5.13 Cold-cache arm startup

Use an empty temporary cache to check that the arm assets can be fetched and loaded on first use.

```bash
RELEASE_TEST_CACHE="$(mktemp -d)"
XDG_CACHE_HOME="$RELEASE_TEST_CACHE" uv run dimos run keyboard-teleop-xarm7
# After inspecting the arm and stopping the blueprint with Ctrl+C:
rm -rf "$RELEASE_TEST_CACHE"
```

Expected result: the blueprint clones the xArm assets into the temporary cache and shows the arm at http://localhost:8095.

## 5.14 Cloud sign-in and data

Needs a Dimensional cloud account and a recording from the [recording check](02_replay-and-simulation.md#24-recording-and-playback).

```bash
uv run dimos login
uv run dimos whoami
uv run dimos data upload recordings/<run-id>/memory.db
uv run dimos data ls
uv run dimos data pull latest --dest /tmp/pulled.db
uv run dimos logout
```

* Open https://console.dimensional.org and find the upload with its map, path and camera previews.

Expected result: login prints a code and a URL, and after approval names the account, which `whoami` repeats. The upload prints `memory.db: complete (<id>)` and `console preview: sent`. `data ls` opens a browser that lists it (q quits). `pull` prints `pulled to /tmp/pulled.db`, and `logout` signs out.
