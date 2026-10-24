# Python agent runtime

Inspect a running DimOS system, deploy Python modules, and send notices to the conversation that created them through
[HarnessRuntime](/dimos/agents/harness/runtime/harness_runtime.py).

## Quickstart

### Load the runtime

With a DimOS system already running, load the runtime and list the available modules, RPCs, and streams:

```sh skip
dimos agent load
dimos mcp call inventory --arg compact=true
```

`dimos agent load` starts any missing relay bridge, `HarnessRuntime`, and `McpServer`. Its JSON output lists the
modules in `loaded` and `already`. Repeating the command preserves running modules. Use the same `MCPSERVER__*`
settings for the loader and clients.

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

### Read the inventory from Python

Run the following in a second Python process with the same `MCPSERVER__*` settings. It prints a system summary, then
lists one module's RPCs. Reuse `call_tool` for the later client examples.

```python skip
from typing import Any

import requests

from dimos.agents.harness.runtime.load import mcp_server_url


def call_tool(name: str, arguments: dict[str, Any]) -> dict[str, Any]:
    response = requests.post(
        mcp_server_url(),
        json={
            "jsonrpc": "2.0",
            "id": 1,
            "method": "tools/call",
            "params": {
                "name": name,
                "arguments": arguments,
                "_meta": {"dimos/conversation": 42},
            },
        },
        timeout=30,
    )
    response.raise_for_status()
    result = response.json()["result"]
    if result.get("isError"):
        raise RuntimeError(result["content"])
    return result


print(call_tool("inventory", {"compact": True})["structuredContent"]["text"])
print(call_tool("inventory", {"module": "HarnessRuntime"})["structuredContent"]["rpcs"])
```

### Create a module and send a notice

Save this module as `reporter.py`. Calling its `report()` RPC publishes a notice for the conversation that creates it.

```python
from dimos.agents.harness.runtime.agent_module import AgentModule
from dimos.core.core import rpc


class Reporter(AgentModule):
    @rpc
    def report(self) -> bool:
        """Publish one notice for this module's owning conversation."""
        return self.notify("Example is running")
```

In the client process, create the module and call its RPC. The result includes `state: "running"` and the path of the
managed source file. `report()` returns `True` when it publishes the notice. `Dimos.stop()` closes the attached client
while the coordinator stays up.

To change the module, edit the managed source path returned by `create`, then call `restart`. The final calls below
list, restart, unload, and delete the example module.

```python skip
from pathlib import Path

from dimos.porcelain.dimos import Dimos

created = call_tool(
    "agent_module",
    {"op": "create", "name": "reporter", "path": str(Path("reporter.py").resolve())},
)
print(created["structuredContent"])

app = Dimos.connect()
try:
    print(app.get_module("Reporter").report())
finally:
    app.stop()

print(call_tool("agent_module", {"op": "list"})["structuredContent"])
call_tool("agent_module", {"op": "restart", "name": "reporter"})
call_tool("agent_module", {"op": "unload", "name": "reporter"})
call_tool("agent_module", {"op": "delete", "name": "reporter"})
```

An MCP client listening on the GET event stream receives `dimos/harness_event` with `conversation: 42`. The
[integration tests](/dimos/agents/harness/runtime/test_runtime.py) show a Python listener and exercise notices, image exports, lifecycle failures,
module discovery, and ordinary recording against a local system.

## API reference

MCP calls return both human-readable `content` and JSON `structuredContent`, supplied by
[ToolResult](/dimos/agents/mcp/tool_result.py). Failed operations set `isError`. A caller can put an integer conversation ID in
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
`compact=True` adds keyed facts and a shorter text summary without removing the full structured result.

Passing `module` returns its documentation, RPC signatures, and stream types with import paths where available. Passing
both `module` and `rpc` returns that RPC's full documentation. A module can be named by its instance name or unique
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
exceptions through the normal DimOS logger. Ordinary `@rpc`, `@skill`, stream handlers, and module lifecycle methods
remain available. Declare `In[T]` and `Out[T]` ports to join streams by name and type, and use
[Specs](/docs/usage/modules.md) to call other modules.

`notify(text, **data)` publishes an `AgentNotice` and returns `True`. It returns `False` when rate-limited by the class
attribute `notify_interval_s`, which defaults to 10 seconds. The next published notice includes the number suppressed.
Nested `Image` and `PointCloud2` values become exported files and path references. Other values must be JSON with finite
numbers. Images are saved as JPEG; clouds are saved as XYZ JSON, with a uniform stride above 200,000 points. The export
directory retains at most 50 files.

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

The paths are defined in [constants.py](/dimos/agents/harness/runtime/constants.py) and the shared [DimOS constants](/dimos/constants.py).

| Path | Contents |
| --- | --- |
| `MODULES_DIR/dimos_agent/NAME.py` | Editable module source. Operation results return its absolute path. |
| `MODULES_DIR/dimos_agent/NAME_rN.pyc` | Compiled revision loaded into a worker. |
| `HARNESS_STATE_DIR/modules/NAME/module.json` | Owner, class name, and last error. |
| `HARNESS_STATE_DIR/exports/` | Notice attachments. |

By default, `MODULES_DIR` is `~/.local/state/dimos/modules` and `HARNESS_STATE_DIR` is
`~/.local/state/dimos/harness`. `XDG_STATE_HOME` changes their state root.

## See also

- [The MCP server README](/dimos/agents/mcp/README.md), for the server and its tool protocol.
- [Modules](/docs/usage/modules.md), for streams, lifecycle, and typed RPC dependencies.
- [The Python API](/docs/usage/python-api.md), for connecting to and changing a running system.
- [Configuration](/docs/usage/configuration.md), for module settings and environment variables.
