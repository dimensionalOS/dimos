# Python API

The `Dimos` class is the main entry point for using DimOS from Python. There are two modes:

1. **Local** — `Dimos()` creates and runs modules in the current process.
2. **Remote** — `Dimos.connect()` connects to an already-running instance.

## Local mode

(Remember to source `.env`.)

```python skip session=dimos_local
from dimos import Dimos

app = Dimos(n_workers=8)

# Run a blueprint by name.
app.run("unitree-go2-agentic")

# Call skills.
app.skills.relative_move(forward=2.0)

# List all available skills.
print(app.skills)

# Access a module directly.
app.ReplanningAStarPlanner

# Add another module dynamically.
from dimos.robot.unitree.keyboard_teleop import KeyboardTeleop
app.run(KeyboardTeleop)

# Or start it by name. No need for importing.
app.run("keyboard-teleop")  # This will say `KeyboardTeleop is already deployed`

# Stop everything.
app.stop()
```

## Peeking streams

`peek_stream(name, timeout)` pulls the next message from any running
module's stream. Useful for quick inspection without writing a
subscriber:

```python skip
# Grab the image.
img = app.peek_stream("color_image", 1.0)

# Display it in a window.
import cv2
cv2.imshow("color_image", img.data)
cv2.waitKey(0)
```

## Remote mode

Start a daemon first (via CLI or another script), then connect to it:

```bash
dimos run unitree-go2-agentic
```

```python skip
from dimos import Dimos

app = Dimos.connect()

# Everything works the same as local mode
print(app)                     # <Dimos(remote=True, modules=[...])>
print(app.skills)              # list all skills
app.skills.relative_move(forward=2.0)
app.stop()  # closes the connection (does NOT stop the remote process)
```

Connect to a specific instance:

```python skip
# By run ID (from `dimos status`)
app = Dimos.connect(run_id="20260306-143022-unitree-go2")
```

`Dimos.connect()` discovers the daemon over the local LCM bus. Cross-host
connection is set with the standard `LCM_DEFAULT_URL` environment variable.

`run()` and `restart()` also work against a daemon:

```python skip
app = Dimos.connect()

app.run("keyboard-teleop")       # add a module by registry name
app.run(SomeModule)               # or by Module class
app.restart(SomeModule)           # hot-restart it on the daemon
```

Strings and registered Module classes take a name-based fast path. Other
Module classes and `Blueprint` objects are pickled and unpickled on the
daemon, so their module classes must be importable there and all kwargs must
be picklable.

## Limitations

- `stop()` on a connected instance closes the LCM connection but does not terminate the remote process. Use `dimos stop` for that.

## Restarting modules

In local mode, you can hot-restart a module:

```python skip
from dimos.agents.mcp.mcp_server import McpServer

app.restart(McpServer)
```
