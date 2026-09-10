# Tool Streams

Some tools return quickly but keep doing work in the background. For example,
`look_out_for` starts a perception loop and waits minutes for a match;
`follow_person` returns "started following" right away and then keeps publishing
status until the target is lost or the skill is cancelled.

Tool streams are the channel those background updates travel on. Every update is
routed to the MCP client that made the original `tools/call` so it can display
the progress alongside the tool's response.

Under the hood, tool streams turn into MCP `notifications/progress` frames bound
to the client's `progressToken`.

For the dimos-internal `McpClient`, each update lands on the agent's message
queue as a `HumanMessage` tagged `[tool:<tool_name>]` so the model can reason
about it. If a client didn't send a `progressToken` (raw curl, older tools), the
stream falls back to `notifications/message` log frames so updates still arrive,
just without the per-call UI affordance.

Skills never touch the wire format. `Module` exposes three helpers:
`start_tool`, `tool_update`, `stop_tool`. The framework handles the rest.

## Quick example: inline updates

```python
import time

from dimos.agents.annotation import skill
from dimos.core.module import Module


class Counter(Module):
    @skill
    def count_to(self, n: int) -> str:
        """Count to `n`, streaming a status update per step."""
        self.start_tool("count_to")
        for i in range(n):
            self.tool_update("count_to", f"At {i + 1} of {n}")
            time.sleep(0.1)
        self.stop_tool("count_to")
        return f"Counted to {n}."
```

Each `tool_update` shows up in the MCP client as a progress notification bound
to the original `count_to` call. The skill's return value is still the final
`tools/call` result; the streamed updates are *in addition* to it, not *instead
of* it.

## Background-thread example

Most real skills don't block their `tools/call` response. They kick off
background work and return immediately. The background thread publishes updates
for as long as the work is running. This is the `follow_person` / `look_out_for`
shape.

```python
import time
from threading import Thread

from dimos.agents.annotation import skill
from dimos.core.module import Module


class Streamer(Module):
    @skill
    def start_streaming(self, count: int) -> str:
        """Kick off `count` updates from a background thread."""
        self.start_tool("start_streaming")

        def _loop() -> None:
            try:
                for i in range(count):
                    time.sleep(0.1)
                    self.tool_update("start_streaming", f"Update {i + 1} of {count}")
            finally:
                self.stop_tool("start_streaming")

        # Thread cleanup ignored for this example.
        Thread(target=_loop, daemon=True).start()
        return f"Started streaming {count} updates."
```

The skill returns right away with `"Started streaming 3 updates."`. The
background loop then fires `tool_update` calls from a worker thread, and each
one reaches the client as a progress frame attached to the original call. When
the loop finishes (or errors), `stop_tool` tears the channel down.

## The rules

- **`start_tool` must run on the skill's main thread.** The `@skill` wrapper
establishes a per-call context on the thread where the skill is invoked;
`start_tool` reads that context to capture the caller's `progressToken`.
Constructing a tool stream from a detached thread raises `RuntimeError` with a
message that names the invariant.

- **`start_tool("x")` while another `"x"` is already open on the same module
raises.** Two concurrent streams for the same logical tool is almost always a
bug. If you really need to restart, call `stop_tool("x")` first.

- **`tool_update` and `stop_tool` are thread-safe.** Background workers can fire
updates from any thread once `start_tool` has registered the channel.

## What a client sees

When an MCP client calls `tools/call` with a `progressToken`, every
`tool_update` on that call is delivered as a `notifications/progress` JSON-RPC
frame:

```json
{
  "jsonrpc": "2.0",
  "method": "notifications/progress",
  "params": {
    "progressToken": "pt-abc123",
    "progress": 1,
    "message": "Update 1 of 3",
    "_meta": {"tool_name": "start_streaming"}
  }
}
```

The `progressToken` is the one the client supplied on its `tools/call` request.
`progress` is a per-stream monotonic counter (1, 2, 3, ...). The dimos-specific
`_meta.tool_name` hint is used by our internal `McpClient` to route the update
into the agent transcript as `[tool:start_streaming] Update 1 of 3`; external
clients that don't recognize it simply pass it through.
