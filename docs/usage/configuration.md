# Configuration

Dimos provides a `Configurable` base class. See [`service/spec.py`](/dimos/protocol/service/spec.py#L22).

This allows using pydantic models to specify configuration structure and default values per module.

```python
from pydantic import ValidationError

from dimos.protocol.service.spec import BaseConfig, Configurable
from rich import print

class Config(BaseConfig):
    x: int = 3
    hello: str = "world"

class MyClass(Configurable):
    config: Config

myclass1 = MyClass()
print(myclass1.config)

# can easily override
myclass2 = MyClass(hello="override")
print(myclass2.config)

# we will raise an error for unspecified keys
try:
    myclass3 = MyClass(something="else")
except (TypeError, ValidationError) as e:
    print(f"Error: {e}")

```

```results
Config(x=3, hello='world')
Config(x=3, hello='override')
Error: 1 validation error for Config
something
  Extra inputs are not permitted
    For further information visit
https://errors.pydantic.dev/2.12/v/extra_forbidden
```

## Configurable modules

[Modules](/docs/usage/modules.md) inherit from `Configurable`. Their configuration classes inherit from `ModuleConfig`, which provides `g` for shared runtime settings. Put a setting on the module that consumes it; helper classes receive the relevant config or scalar arguments explicitly.

For `dimos run`, values resolve in this order: schema defaults → blueprint values → JSON config → `.env` → environment → CLI flags. Python callers can also pass `overrides` to `BlueprintConfigParser.parse()`; these apply after environment values and before CLI flags. Explicit `global_overrides` apply last.

Module flags go **after the blueprint names**. Use the module's instance name, lowercase, with hyphens in field names. Environment variables use double underscores between path components. A unique short flag is also accepted; qualified names avoid ambiguity.

```bash skip
dimos --simulation mujoco run unitree-go2 \
  --go2connection.mujoco.start-pos '[2.0, -1.0]' \
  --go2connection.mujoco.shadows off

dimos --simulation mujoco run xarm-perception-sim --mujocosimmodule.shadows on

GO2CONNECTION__MUJOCO__STEPS_PER_FRAME=5 dimos --simulation mujoco run unitree-go2

dimos run unitree-go2-temporal-memory --temporalmemory.new-memory true
dimos run unitree-go2 --help
```

Both MuJoCo sims time a shadowed camera render at start and turn shadows off when the renderer is too slow for them, for example software GL on a machine without a GPU. `shadows` takes `auto` (the default), `on` or `off`.

Use `--config robot.json` for structured settings:

```json
{
  "g": {"simulation": "mujoco", "viewer": "rerun"},
  "go2connection": {"mujoco": {"start_pos": [2.0, -1.0], "shadows": "off"}},
  "rerunbridgemodule": {"rerun_open": "none", "rerun_save": true}
}
```

Python module construction and `.blueprint(...)` accept the same fields as keyword arguments. Direct `ModuleCoordinator.build(...)` uses blueprint values; use `BlueprintConfigParser` when resolving JSON, environment, and CLI overrides in Python.

## Global settings

[`GlobalConfig`](/dimos/core/global_config.py) owns process management, shared transport, and choices that affect stack composition. It retains robot addresses used by shared transport, simulation/replay selection, recording, Zenoh settings, coordinator RPC serving, viewer selection, workers, scene packages, robot models, robot geometry (`robot_width`, `robot_rotation_diameter`, which the navigation and patrol modules share), native builds, dtop, the default listen host, and relay composition (`local_relay`, `relay_url`).

`dimos show-config` shows only these global settings. Global environment names are uppercase field names, such as `ROBOT_IP` and `N_WORKERS`; transport also accepts `DIMOS_TRANSPORT`. Use `dimos run <blueprint> --help` to inspect that stack's module settings.

`listen_host` defaults to `127.0.0.1`, but does not restrict Rerun's gRPC or
web viewer listeners. They bind to all interfaces because the pinned SDK has
no bind address option. See [Network exposure](/docs/usage/visualization.md#network-exposure)
for firewall guidance and how to disable the Rerun bridge.

## Migrating former global settings

The former global names have been removed. Update CLI arguments, `.env`, JSON `g` sections, and Python `.global_config(...)` calls. Unknown fields in `g` are rejected. For environment variables and `.env` entries, `dimos run` and `dimos rerun-bridge` require migration of old credentials and hardware addresses, naming their replacements. Other former names produce a warning and are ignored. `MCP_TIMEOUT` is ignored without a migration warning; use `dimos mcp call --timeout` for each call.

Scoped module environment variables are resolved by `BlueprintConfigParser`, including when invoked by the CLI. They do not configure direct Python module construction or `ModuleCoordinator.build(...)` calls. Scripts and notebooks must pass module kwargs, such as `McpServer.blueprint(port=9992)` or `TypeSafeNavigationAgent.blueprint(api_key=key)`, or parse the blueprint with `BlueprintConfigParser` before building it.

| Former global setting | Owner / replacement |
|---|---|
| `unitree_aes_128_key` | `aes_128_key` on `GO2Connection`, `Go2FleetConnection`, or `G1HighLevelWebRtc`, as applicable |
| `obstacle_avoidance`, `replay_exit` | Go2 connection config (including fleet) |
| `mujoco_camera_position`, `mujoco_room`, `mujoco_room_from_occupancy`, `mujoco_start_pos`, `mujoco_steps_per_frame`, `mujoco_shadows` | Nested `mujoco` config on `GO2Connection`; drop the `mujoco_` prefix. Positions are JSON arrays of six and two numbers respectively. |
| `dimsim_scene`, `dimsim_port`, `dimsim_headless`, `dimsim_ready_timeout` | `go2connection.dimsim.scene`, `.port`, `.headless`, `.ready_timeout` |
| `mujoco_global_costmap_from_occupancy`, `mujoco_global_map_from_pointcloud` | `map.global_costmap_from_occupancy`, `map.global_map_from_pointcloud` |
| `nerf_speed` | `replanningastarplanner.nerf_speed` |
| `new_memory` | `temporalmemory.new_memory` |
| `detection_model` | `perceiveloopskill.detection_model` |
| `typesafe_api_key` | `typesafenavigationagent.api_key` (or another TypeSafe subclass's `api_key`) |
| `rerun_open`, `rerun_web`, `rerun_save` | `rerunbridgemodule` config, with the same field names |
| `rerun_host` | `rerunbridgemodule.host` sets the default gRPC connection URL. `rerunwebsocketserver.host` sets the websocket bind address. Both default to `g.listen_host`. Neither restricts the Rerun gRPC or web viewer listeners. |
| `rerun_websocket_server_port` | `rerunwebsocketserver.port`; a `RerunBridgeModule` in the same blueprint takes the viewer URL from that server. `rerunbridgemodule.websocket_url` is only for a server elsewhere |
| `memory_limit` | Removed unused global. The existing `rerunbridgemodule.memory_limit` controls viewer memory. |
| `mcp_port` | `mcpserver.port`; an `Agent` in the same blueprint takes the endpoint from that server. `agent.mcp_server_url` is only for a server elsewhere |
| `mcp_timeout` | `McpAdapter(timeout=...)` or `dimos mcp call --timeout ...`; the `Agent`'s own MCP calls use a fixed 120 s |
| `robot_id`, `relay_ca`, `relay_key` | `relaybridgemodule` config, with the same field names |
| `xarm7_ip`, `xarm6_ip`, `lite6_ip` | `xarmconnection.ip`, or the connection's own name in a two-arm blueprint (see below) |
| `can_port`, `device_path` | the arm connection's own field, e.g. `piperconnection.can_port` or `a750connection.device_path` (see below) |
| `mujoco_scene` | `mujocosimmodule.address` |
| `dimos_cloud_url`, `dimos_api_key`, `dimos_http_timeout`, `dimos_upload_*`, `dimos_staging_dir` | [`CloudConfig`](/dimos/cloud/config.py): `url`, `api_key`, `http_timeout`, `upload_*`, `staging_dir`; environment prefix `DIMOS_CLOUD_` |

For example, `UNITREE_AES_128_KEY` becomes `GO2CONNECTION__AES_128_KEY`, and `RERUN_OPEN` becomes `RERUNBRIDGEMODULE__RERUN_OPEN`. Configure each connection separately when a stack has multiple robot connections. The legacy websocket dashboard follows the Rerun bridge's `web` or `both` mode by default; `websocketvismodule.open_browser` and `show_rerun_dashboard` override that behavior independently.

MCP's bound endpoint is stored in run metadata, so `dimos mcp` finds the newest running server on any host or port, and an `Agent` asks the `McpServer` in its blueprint for the endpoint:

```bash skip
dimos run unitree-go2-agentic --mcpserver.port 9992
```

## Hardware configuration

Each robot's connection module owns its address (an IP, a CAN interface, a serial port). Select an xArm with `--xarmconnection.ip 192.168.1.185` (the dual xArm blueprint takes `--left_arm.ip` and `--right_arm.ip`, the Piper + xArm6 ones `--xarm_arm.ip` and `--piper_arm.can_port`), YAM with `--openyamconnection.can-port can1`, the Piper with `--piperconnection.can-port can0`, the A1Z with `--a1zconnection.can-port a1zcan`, the A-750 with `--a750connection.device-path /dev/ttyACM0`, or OpenArm with both `--openarmconnection.left-can-port can1` and `--openarmconnection.right-can-port can0`.
