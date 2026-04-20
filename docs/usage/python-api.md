# Python API

The `Dimos` class is the main entry point for using DimOS from Python. There are two modes:

1. **Local** — `Dimos()` creates and runs modules in the current process.
2. **Remote** — `Dimos.connect()` connects to an already-running instance.

## Local mode

(Remember to source `.env`.)

```python
from dimos import Dimos

app = Dimos(n_workers=8)

# Run a blueprint by name (requires a full install with that blueprint and its deps).
# app.run("unitree-go2-agentic")

# After you have called run() with an agentic blueprint, skills are available, e.g.:
# app.skills.relative_move(forward=2.0)
# print(app.skills)

print("Dimos", type(app).__name__)

# Add another module dynamically (example import only):
from dimos.robot.unitree.keyboard_teleop import KeyboardTeleop

print("KeyboardTeleop", KeyboardTeleop.__name__)

# Or start it by name after you have a coordinator running, e.g.:
# app.run("keyboard-teleop")
# app.stop()
```

<!--Result:-->
```
Dimos Dimos
pygame 2.6.1 (SDL 2.28.4, Python 3.12.13)
Hello from the pygame community. https://www.pygame.org/contribute.html
KeyboardTeleop KeyboardTeleop
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

# By host and port
app = Dimos.connect(host="192.168.1.50", port=18861)
```

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

- `stop()` on a connected instance closes the RPyC connection but does not terminate the remote process. Use `dimos stop` for that.

## Restarting modules

In local mode, you can hot-restart a module:

```python skip
from dimos.agents.mcp.mcp_server import McpServer

app.restart(McpServer)
```
