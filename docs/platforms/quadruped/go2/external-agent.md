# Control Go2 from an external MCP agent

`unitree-go2-mcp` exposes the existing Go2 tools without starting a DimOS model
client. An external agent calls its MCP server on the laptop. DimOS handles camera/lidar streams, mapping, path planning, and WebRTC
robot communication. CUDA is optional: voxel mapping automatically falls back
to its NumPy CPU backend.

```bash
# Recorded sensors, no physical robot:
dimos --replay --viewer none --listen-host 127.0.0.1 run unitree-go2-mcp

# Or hardware (connection stands the robot up):
dimos --robot-ip <discovered-ip> --listen-host 127.0.0.1 run unitree-go2-mcp
```

Use `dimos go2tool discover --lan` to find the current address. From a second
terminal, run `dimos mcp list-tools`. The MCP HTTP URL is
`http://127.0.0.1:9990/mcp` by default; `--mcp-port` changes the port.

The principal tools are:

- `observe()` returns a fresh camera image.
- `move_to(x, y, degrees=None, relative=False)` navigates, waits, and reports
  the outcome and final pose. With `relative=True`, x is forward, y is left,
  and degrees is a relative turn. Distances are meters.
- `stop_navigation()` cancels the active navigation goal.
- `execute_sport_command(command_name)` executes an existing firmware action.

The MCP server may expose additional skills from the composed modules. The
external client chooses which to declare to its model. Use one controlling
agent, execute movement calls sequentially, and explicitly call
`stop_navigation` when interrupting a navigation call: disconnecting HTTP does
not cancel an RPC already in progress. A `move_to` timeout now cancels its goal.
Navigation cancellation does not interrupt every firmware sport routine.

The blueprint combines the normal Go2 navigation stack with
`UnitreeSkillContainer`, `ObserveSkill`, `NavigationStopSkill`, and `McpServer`.
It does not load semantic-navigation models. `NavigationSkillContainer` also
inherits the same stop skill, preserving its existing tool name.

Replay is a finite sensor recording, so camera calls after it ends return a
frame timeout. Restart it for fresh frames. Replay validates tool wiring and
mapping; movement commands cannot change the recorded trajectory.
