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

GO2CONNECTION__MUJOCO__STEPS_PER_FRAME=5 dimos --simulation mujoco run unitree-go2

dimos run unitree-go2-temporal-memory --temporalmemory.new-memory true
dimos run unitree-go2 --help
```

Use `--config robot.json` for structured settings:

```json
{
  "g": {"simulation": "mujoco", "viewer": "rerun"},
  "go2connection": {"mujoco": {"start_pos": [2.0, -1.0], "shadows": "off"}},
  "rerunbridgemodule": {"rerun_open": "none", "rerun_save": true}
}
```

Python module construction and `.blueprint(...)` accept the same fields as keyword arguments. Direct `.build()` uses blueprint values; use `BlueprintConfigParser` when resolving JSON, environment, and CLI overrides in Python.

## Global settings

[`GlobalConfig`](/dimos/core/global_config.py) owns process management, shared transport, and choices that affect stack composition. It retains robot addresses used by shared transport, simulation/replay selection, recording, Zenoh settings, coordinator RPC serving, viewer selection, workers, scene packages, robot models, robot geometry (`robot_width`, `robot_rotation_diameter`, which the navigation and patrol modules share), native builds, dtop, the default listen host, and relay composition (`local_relay`, `relay_url`).

`dimos show-config` shows only these global settings. Global environment names are uppercase field names, such as `ROBOT_IP` and `N_WORKERS`; transport also accepts `DIMOS_TRANSPORT`. Use `dimos run <blueprint> --help` to inspect that stack's module settings.

## Migrating former global settings

The former global names have been removed. Update CLI arguments, `.env`, JSON `g` sections, and Python `.global_config(...)` calls. Unknown fields in `g` are rejected. For environment variables and `.env` entries, `dimos run` and `dimos rerun-bridge` require migration of old credentials and hardware addresses, naming their replacements. Other former names produce a warning and are ignored. `MCP_TIMEOUT` is ignored without a migration warning; use `dimos mcp call --timeout` for each call.

Scoped module environment variables are resolved by `BlueprintConfigParser`, including when invoked by the CLI. They do not configure direct Python module construction or `.build()` calls. Scripts and notebooks must pass module kwargs, such as `McpServer.blueprint(port=9992)` or `TypeSafeNavigationAgent.blueprint(api_key=key)`, or parse the blueprint with `BlueprintConfigParser` before building it.

| Former global setting | Owner / replacement |
|---|---|
| `unitree_aes_128_key` | `aes_128_key` on `GO2Connection`, `Go2FleetConnection`, `G1Connection`, or `G1HighLevelWebRtc`, as applicable |
| `obstacle_avoidance`, `replay_exit` | Go2 connection config (including fleet) |
| `mujoco_camera_position`, `mujoco_room`, `mujoco_room_from_occupancy`, `mujoco_start_pos`, `mujoco_steps_per_frame`, `mujoco_shadows` | Nested `mujoco` config on `GO2Connection` or `G1SimConnection`; drop the `mujoco_` prefix. Positions are JSON arrays of six and two numbers respectively. |
| `dimsim_scene`, `dimsim_port`, `dimsim_headless`, `dimsim_ready_timeout` | `go2connection.dimsim.scene`, `.port`, `.headless`, `.ready_timeout` |
| `mujoco_global_costmap_from_occupancy`, `mujoco_global_map_from_pointcloud` | `map.global_costmap_from_occupancy`, `map.global_map_from_pointcloud` |
| `nerf_speed` | `replanningastarplanner.nerf_speed` |
| `new_memory` | `temporalmemory.new_memory` |
| `detection_model` | `perceiveloopskill.detection_model` |
| `typesafe_api_key` | `typesafenavigationagent.api_key` (or another TypeSafe subclass's `api_key`) |
| `rerun_open`, `rerun_web`, `rerun_save` | `rerunbridgemodule` config, with the same field names |
| `rerun_host` | `rerunbridgemodule.host` and `rerunwebsocketserver.host`, independently; both default to `g.listen_host` |
| `rerun_websocket_server_port` | `rerunwebsocketserver.port`; a `RerunBridgeModule` in the same blueprint takes the viewer URL from that server. `rerunbridgemodule.websocket_url` is only for a server elsewhere |
| `memory_limit` | Removed unused global. The existing `rerunbridgemodule.memory_limit` controls viewer memory. |
| `mcp_port` | `mcpserver.port`; an `McpClient` in the same blueprint takes the endpoint from that server. `mcpclient.mcp_server_url` is only for a server elsewhere |
| `mcp_timeout` | `McpAdapter(timeout=...)` or `dimos mcp call --timeout ...`; `McpClient.timeout` remains independent |
| `robot_id`, `relay_ca`, `relay_key` | `relaybridgemodule` config, with the same field names |
| `xarm7_ip`, `xarm6_ip`, `lite6_ip`, `can_port`, `device_path` | `controlcoordinator.addresses`, keyed by hardware id (see below) |
| `mujoco_scene` | `mujocosimmodule.address`; the coordinator's simulated arm attaches to that scene |
| `dimos_cloud_url`, `dimos_api_key`, `dimos_http_timeout`, `dimos_upload_*`, `dimos_staging_dir` | [`CloudConfig`](/dimos/cloud/config.py): `url`, `api_key`, `http_timeout`, `upload_*`, `staging_dir`; environment prefix `DIMOS_CLOUD_` |

For example, `UNITREE_AES_128_KEY` becomes `GO2CONNECTION__AES_128_KEY`, and `RERUN_OPEN` becomes `RERUNBRIDGEMODULE__RERUN_OPEN`. Configure each connection separately when a stack has multiple robot connections. The legacy websocket dashboard follows the Rerun bridge's `web` or `both` mode by default; `websocketvismodule.open_browser` and `show_rerun_dashboard` override that behavior independently.

MCP's bound endpoint is stored in run metadata, so `dimos mcp` finds the newest running server on any host or port, and an `McpClient` asks the `McpServer` in its blueprint for the endpoint:

```bash skip
dimos run unitree-go2-agentic --mcpserver.port 9992
```

## Hardware configuration

A coordinator's `hardware` list names each device's adapter. Device addresses (an IP, a CAN interface, a serial port) live in a separate map keyed by hardware id, so one saved robot configuration serves every blueprint for that robot in both execution modes:

```json
{"controlcoordinator": {"addresses": {"arm": "192.0.2.10"}}}
```

```bash skip
dimos run xarm-grasp-agent --config robot.json
CONTROLCOORDINATOR__ADDRESSES='{"arm": "192.0.2.10"}' dimos run coordinator-xarm7
dimos run keyboard-teleop-piper --controlcoordinator.addresses '{"arm": "can0"}'
```

The coordinator settles each component when it starts, once the execution mode is known:

- On hardware, a component takes its address from `addresses`. A component left without one runs the mock adapter when its blueprint allows that (`mock_without_address` in the robot's config helper), and otherwise fails to connect.
- With `--simulation`, the blueprint's simulated adapters stay and `addresses` is ignored. A simulated arm attaches to the `MujocoSimModule` in its blueprint, so a custom scene is set once, as `mujocosimmodule.address`.

The robot config helpers (`xarm7_hardware`, `piper_hardware`, `a1z_hardware`, `a750_hardware`, ...) also take an explicit `address=` for a blueprint of your own. OpenYAM selects its CAN device with `openyam_hardware(bus_devices={"openyam": "can1"})`; that lands in `adapter_kwargs.runtime_config.bus_devices`, not in `addresses`. A JSON or environment override of `hardware` itself replaces the entire list.
