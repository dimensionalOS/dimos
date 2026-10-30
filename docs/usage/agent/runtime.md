# Python agent runtime

Inspect a running dimOS system, deploy Python modules, and send notices to the conversation that created them through
[HarnessRuntime](/dimos/agents/harness/runtime/harness_runtime.py).

## Quickstart

### Load the runtime

With a dimOS system already running, load the runtime and list the available modules, RPCs, and streams:

```sh skip
dimos agent load
dimos mcp call inventory --arg compact=true
```

`dimos agent load` starts any missing relay bridge, `HarnessRuntime`, and `McpServer`. Its JSON output lists the
modules in `loaded` and `already`. Repeating the command preserves running modules.

### Start a runtime without robot hardware

If you do not have a system running, save this as `runtime.py` and run `python runtime.py` instead. It starts a local
runtime and MCP server without robot hardware or a model provider. Leave it running for the examples below, then stop
it with Ctrl-C.

```python skip
from dimos.agents.harness.runtime.harness_runtime import HarnessRuntime
from dimos.agents.harness.runtime.load import mcp_server_blueprint
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator

if __name__ == "__main__":
    ModuleCoordinator.build(
        autoconnect(HarnessRuntime.blueprint(), mcp_server_blueprint())
    ).loop()
```

### Inspect modules and RPCs

Inspect one module, then read the documentation for a specific RPC:

```sh skip
dimos mcp call inventory --arg module=HarnessRuntime
dimos mcp call inventory --arg module=HarnessRuntime --arg rpc=agent_module
```

### Create a module and send a notice

Save this module as `reporter.py`. Its `report()` RPC sends a notice through the runtime.

```python
from dimos.agents.harness.runtime.agent_module import AgentModule
from dimos.core.core import rpc


class Reporter(AgentModule):
    @rpc
    def report(self) -> bool:
        """Publish one notice for this module's owning conversation."""
        return self.notify("Example is running")
```

From the directory containing `reporter.py`, create the module:

```sh skip
dimos mcp call agent_module --arg op=create --arg name=reporter --arg path="$PWD/reporter.py"
```

The result reports the running revision and the managed source path. Call the module's RPC through the Python API:

```python skip
from dimos.porcelain.dimos import Dimos

app = Dimos.connect()
try:
    print(app.get_module("Reporter").report())
finally:
    app.stop()
```

`report()` returns `True` when it publishes the notice. `Dimos.stop()` closes the attached client while the coordinator
stays up. An MCP listener receives `dimos/harness_event`. Modules created through the CLI have no conversation owner,
so their notices carry `conversation: null`.

### Edit, restart, and remove a module

Edit the managed source path returned by `create`, then restart the module to run the updated code. List the saved
modules and restart this example:

```sh skip
dimos mcp call agent_module --arg op=list
dimos mcp call agent_module --arg op=restart --arg name=reporter
```

Unload the module to stop it, then delete it to remove its saved files:

```sh skip
dimos mcp call agent_module --arg op=unload --arg name=reporter
dimos mcp call agent_module --arg op=delete --arg name=reporter
```

## API reference

Results are [SkillResult](/dimos/agents/skill_result.py) objects, which McpServer sends as one JSON object in both `content`
and `structuredContent`. The runtime's data is the object's `metadata`. The module and RPC forms of `inventory` return
plain text. A call that raises, such as one naming an unknown module, sets `isError`. An `agent_module` operation that
ran reports a failure in its outcome instead. A caller can put an integer conversation ID in
`params._meta["dimos/conversation"]` when creating a module. The runtime saves that owner and includes it in the module's
notices; without the metadata, the owner is `null`.

### Runtime loading and configuration

The Python entry points are `load()`, `mcp_server_blueprint()`, and `mcp_server_url()` in
[load.py](/dimos/agents/harness/runtime/load.py). The blueprint helper applies `MCPSERVER__*` environment settings when
composing a system in Python. `load()` also applies `RELAYBRIDGEMODULE__*` settings to a newly loaded relay bridge and
preserves an existing authored bridge. A running MCP server must match the configured client endpoint.

### `inventory(module="", rpc="", compact=False)`

Every call reads the running system. With no module argument, the result contains the environment, modules, RPCs,
native streams, and saved agent modules. The environment includes the connected relay's endpoint when one is present.
`compact=True` returns only keyed one-line facts, which clients compare to report what changed.

Passing `module` returns, as text, its documentation, RPC signatures, and stream types with import paths where
available. Passing both `module` and `rpc` returns that RPC's full documentation. A module can be named by its instance name or unique
class name. An RPC name without a module is an error. The result types and formatting live in
[inventory.py](/dimos/agents/harness/runtime/inventory.py).

### `agent_module(op, name="", path="")`

| Operation | Result |
| --- | --- |
| `create` | Copies `path` into the managed source file and starts a new module. Without `path`, uses the managed file already there. |
| `restart` | Compiles and imports the current source, then replaces the running revision. |
| `unload` | Stops the module and keeps its source and saved record. |
| `delete` | Stops the module and removes its source, revisions, and saved record. |
| `list` | Returns saved module records with their current running or stopped state. |

Names start with a lowercase letter, contain only lowercase letters, digits, and underscores, and have at most 64
characters. A name cannot end in `_r` followed by digits. `path` is accepted only for `create`. The file must define one
`AgentModule` subclass. Its Python class name must be unique in the running system.

Operation results include the resulting state, source path, revision, and an error and traceback when available. A
compile or import failure during restart leaves the old revision running. If the replacement fails during startup,
the old revision has already stopped. Modules stop with the system; their saved files remain for a later restart.

### `AgentModule`

`AgentModule` runs in a dedicated worker by default. `subscribe(stream, callback)` registers cleanup and logs callback
exceptions through the normal dimOS logger. Ordinary `@rpc`, `@skill`, stream handlers, and module lifecycle methods
remain available. Declare `In[T]` and `Out[T]` ports to join streams by name and type, and use
[Specs](/docs/usage/modules.md) to call other modules.

`notify(text, **data)` publishes an `AgentNotice` and returns `True`. It returns `False` when rate-limited by the class
attribute `notify_interval_s`, which defaults to 10 seconds. The next published notice includes the number suppressed.
Nested `Image` and `PointCloud2` values become exported files and path references. Other values must be JSON with finite
numbers. Images are saved as JPEG; clouds are saved as XYZ JSON, with a uniform stride above 200,000 points. Points with
a NaN or infinite coordinate are dropped. The export directory retains at most 50 files.

### Notifications and trajectory entries

MCP's GET event stream carries two notifications:

- `dimos/harness_event` contains the module name, owning conversation, text, timestamp, JSON data, exported files, and
  suppressed-notice count. Notices are live and are not replayed after a disconnect.
- `dimos/harness_status` reports `agent_module` or `system_modules` as its cause. Clients read `inventory` again for the
  current state.

`publish_steps(steps)` validates the whole batch before publishing any `AgentStep` messages. Each entry has
`conversation` (integer), `entry` (string), `kind`, `text`, and `ts` (Unix seconds). The result is a published count.
Recording follows the system's normal topic selection and retention; the count does not acknowledge durable storage.

### Files on disk

The paths are defined in [constants.py](/dimos/agents/harness/runtime/constants.py) and the shared [dimOS constants](/dimos/constants.py).

| Path | Contents |
| --- | --- |
| `RUNTIME_MODULES_DIR/dimos_agent/NAME.py` | Editable module source. Operation results return its absolute path. |
| `RUNTIME_MODULES_DIR/dimos_agent/NAME_rN.pyc` | Compiled revision loaded into a worker. |
| `HARNESS_STATE_DIR/modules/NAME/module.json` | Owner, class name, and last error. |
| `HARNESS_STATE_DIR/exports/` | Notice attachments. |

By default, `RUNTIME_MODULES_DIR` is `~/.local/state/dimos/modules` and `HARNESS_STATE_DIR` is
`~/.local/state/dimos/harness`. `XDG_STATE_HOME` changes their state root.

## See also

- [The MCP server README](/dimos/agents/mcp/README.md), for the server and its tool protocol.
- [Modules](/docs/usage/modules.md), for streams, lifecycle, and typed RPC dependencies.
- [The Python API](/docs/usage/python-api.md), for connecting to and changing a running system.
- [Configuration](/docs/usage/configuration.md), for module settings and environment variables.
