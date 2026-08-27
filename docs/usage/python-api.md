---
title: "Python API"
---

The `Dimos` class is the main entry point for using dimOS from Python. There are two modes:

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
app.skills.move_to(x=2.0, relative=True)

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

## RPC calls

Modules can define `@rpc` methods which you can call. Here's an example:

```python skip
from dimos.msgs.geometry_msgs.Twist import Twist
# Rotate right.
app.GO2Connection.move(Twist(linear=(0, 0, 0), angular=(0, 0, -1)), duration=0.05)
# Move forward.
app.GO2Connection.move(Twist(linear=(1, 0, 0), angular=(0, 0, 0)), duration=0.05)
```

## Discovering modules and RPCs

Discovery works in both local and remote mode:

```python skip
# Live structured records for exact deployed instances.
app.list_modules()

# Resolve an exact instance name.
camera = app.get_module("robot0/camera")

# A class name also works when exactly one instance has that class.
planner = app.get_module("ReplanningAStarPlanner")

# Discover every advertised RPC, or filter by a name or proxy.
app.list_rpcs()
app.list_rpcs(camera)

# Describe proxies or a fully qualified RPC name.
app.describe(camera)
app.describe(camera.start)
app.describe("robot0/camera.start")
```

Each discovery call refreshes coordinator descriptors, so newly loaded modules
appear without a refresh step. Module records include `instance_name`,
`class_name`, `qualified_path`, documentation, and RPC records. RPC records
include their module instance, parameters, return type, documentation, and
signature when the module class can be imported locally.

If multiple instances share a class, class-name lookup raises an ambiguity error
that lists their exact instance names. RPC strings passed to `describe()` must be
qualified as `module.rpc`.

RPC proxies preserve the local method signature and docstring for standard Python
inspection:

```python skip
import inspect

move = app.get_module("GO2Connection").move
print(inspect.signature(move))
help(move)
```

When the client cannot import the deployed module class, advertised RPC names
remain callable, but unavailable parameter and documentation metadata is reported
as unknown.

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

Start a coordinator first (via CLI or another script), then connect to it:

```bash
dimos run unitree-go2-agentic
```

```python skip
from dimos import Dimos

app = Dimos.connect()

# Everything works the same as local mode
print(app)                     # <Dimos(remote=True, modules=[...])>
print(app.skills)              # list all skills
app.skills.move_to(x=2.0, relative=True)
app.stop()  # closes the connection (does NOT stop the remote process)
```

`Dimos.connect()` probes the coordinator on the configured transport bus. It does
not require a CLI run-registry entry, so it also attaches to
`ModuleCoordinator.build(...).loop()` launched directly from Python. dimOS
supports one coordinator per bus; configure the transport bus consistently to
connect across processes or hosts.

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

You can use this in development. You can write a module, load it, gather feedback from running it, change the code, and restart the module to see if it has improved.

### What needs a daemon restart

Hot-restart (`app.restart(MyModule)`) reloads the module's source, so the body of `start()`, handlers, and `@rpc` methods all pick up changes. But the following require a full daemon restart (`dimos stop` then `dimos run ...`):

- Adding or removing `In[T]` / `Out[T]` stream declarations on any module (autoconnect wiring is computed at coordinator build time).
- Adding or removing module-ref / Spec declarations (`_thing: SomeSpec`).
- Changing the blueprint's set of modules.

If you find yourself needing data from an existing module that isn't on its `Out` streams, the canonical fix is to add an `Out[T]` to that module and restart the daemon — don't spin up a parallel connection to the underlying hardware.

### Operational gotchas

- `--daemon` does not detach right away. Background it with `&` or `nohup` if you want the terminal back.
- `dimos stop` reads its target from a registry under `$XDG_STATE_HOME/dimos/runs`. If the registry file is removed but the process is alive, `dimos stop` won't see it — kill the PID directly (find it with `ps aux | grep "dimos.*--daemon"`).
- `load_blueprint` over LCM has a 120s RPC timeout. If it raises `TimeoutError` after that long, the module may still have been deployed and started — check the daemon log for the `Deployed module` entry before assuming failure.
