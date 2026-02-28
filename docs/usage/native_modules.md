# Native Modules

Prerequisite for this is to understand dimos [Modules](/docs/usage/modules.md) and [Blueprints](/docs/usage/blueprints.md).

Native modules let you wrap **any executable** as a first-class DimOS module, given it speaks LCM.

Python will handle blueprint wiring, lifecycle, and logging. Native binary handles the actual computation, publishing and subscribing directly on LCM.

Python module **never touches the pubsub data**. It just passes configuration and LCM topic to use via CLI args to your executable.

On how to speak LCM with the rest of dimos, you can read our [LCM intro](/docs/usage/lcm.md)

## Defining a native module

Python side native module is just a definition of a **config** dataclass and **module** class specifying pubsub I/O.

Both the config dataclass and pubsub topics get converted to CLI args passed down to your executable once the module is started.

```python no-result session=nativemodule
from dataclasses import dataclass
from dimos.core import Out, LCMTransport
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.sensor_msgs.Imu import Imu
import time

@dataclass(kw_only=True)
class MyLidarConfig(NativeModuleConfig):
    executable: str = "./build/my_lidar"
    host_ip: str = "192.168.1.5"
    frequency: float = 10.0

class MyLidar(NativeModule):
    default_config = MyLidarConfig
    pointcloud: Out[PointCloud2]
    imu: Out[Imu]


```

That's it. `MyLidar` is a full DimOS module. You can use it with `autoconnect`, blueprints, transport overrides, and specs. Once this module is started, your `./build/my_lidar` will get called with specific CLI args.


## How it works

When `start()` is called, NativeModule:

1. **Builds the executable** if it doesn't exist and `build_command` is set.
2. **Collects topics** from blueprint-assigned transports on each declared port.
3. **Builds the command line**: `<executable> --<port> <topic> ... --<config_field> <value> ...`
4. **Launches the subprocess** with `Popen`, piping stdout/stderr.
5. **Starts a watchdog** thread that calls `stop()` if the process crashes.

For the example above, the launched command would look like:

```sh
./build/my_lidar \
    --pointcloud '/pointcloud#sensor_msgs.PointCloud2' \
    --imu '/imu#sensor_msgs.Imu' \
    --host_ip 192.168.1.5 \
    --frequency 10.0
```

```python ansi=false session=nativemodule skip
mylidar = MyLidar()
mylidar.pointcloud.transport = LCMTransport("/lidar", PointCloud2)
mylidar.imu.transport = LCMTransport("/imu", Imu)
mylidar.start()
```

<!--Result:-->
```
2026-02-14T11:22:12.123963Z [info     ] Starting native process   [dimos/core/native_module.py] cmd='./build/my_lidar --pointcloud /lidar#sensor_msgs.PointCloud2 --imu /imu#sensor_msgs.Imu --host_ip 192.168.1.5 --frequency 10.0' cwd=/home/lesh/coding/dimos/docs/usage/build
```

Topic strings use the format `/<name>#<msg_type>`, which is the LCM channel name that Python `LCMTransport` subscribers use. The native binary publishes on these exact channels.

When `stop()` is called, the process receives SIGTERM. If it doesn't exit within `shutdown_timeout` seconds (default 10), it gets SIGKILL.

## Config

`NativeModuleConfig` extends `ModuleConfig` with subprocess fields:

| Field              | Type             | Default       | Description                                                 |
|--------------------|------------------|---------------|-------------------------------------------------------------|
| `executable`       | `str`            | *(required)*  | Path to the native binary (relative to `cwd` if set)        |
| `build_command`    | `str \| None`    | `None`        | Shell command to run if executable is missing (auto-build)  |
| `cwd`              | `str \| None`    | `None`        | Working directory for build and runtime. Relative paths are resolved against the Python file defining the module |
| `extra_args`       | `list[str]`      | `[]`          | Additional CLI arguments appended after auto-generated ones |
| `extra_env`        | `dict[str, str]` | `{}`          | Extra environment variables for the subprocess              |
| `shutdown_timeout` | `float`          | `10.0`        | Seconds to wait for SIGTERM before SIGKILL                  |
| `log_format`       | `LogFormat`      | `TEXT`        | How to parse subprocess output (`TEXT` or `JSON`)           |
| `cli_exclude`      | `frozenset[str]` | `frozenset()` | Config fields to skip when generating CLI args              |

### Auto CLI arg generation

Any field you add to your config subclass automatically becomes a `--name value` CLI arg. Fields from `NativeModuleConfig` itself (like `executable`, `extra_args`, `cwd`) are **not** passed — they're for Python-side orchestration only.

```python skip

class LogFormat(enum.Enum):
    TEXT = "text"
    JSON = "json"

@dataclass(kw_only=True)
class MyConfig(NativeModuleConfig):
    executable: str = "./build/my_module" # relative or absolute path to your executable
    host_ip: str = "192.168.1.5"     # becomes --host_ip 192.168.1.5
    frequency: float = 10.0           # becomes --frequency 10.0
    enable_imu: bool = True           # becomes --enable_imu true
    filters: list[str] = field(default_factory=lambda: ["a", "b"])  # becomes --filters a,b
```

- `None` values are skipped.
- Booleans are lowercased (`true`/`false`).
- Lists are comma-joined.

### Excluding fields

If a config field shouldn't be a CLI arg, add it to `cli_exclude`:

```python skip
@dataclass(kw_only=True)
class FastLio2Config(NativeModuleConfig):
    executable: str = "./build/fastlio2"
    config: str = "mid360.yaml"                          # human-friendly name
    config_path: str | None = None                       # resolved absolute path
    cli_exclude: frozenset[str] = frozenset({"config"})  # only config_path is passed

    def __post_init__(self) -> None:
        if self.config_path is None:
            self.config_path = str(Path(self.config).resolve())
```

## Using with blueprints

Native modules work with `autoconnect` exactly like Python modules:

```python skip
from dimos.core.blueprints import autoconnect

class PointCloudConsumer(Module):
    pointcloud: In[PointCloud2]
    imu: In[Imu]

autoconnect(
    MyLidar.blueprint(host_ip="192.168.1.10"),
    PointCloudConsumer.blueprint(),
).build().loop()
```

`autoconnect` matches ports by `(name, type)`, assigns LCM topics, and passes them to the native binary as CLI args. You can override transports as usual:

```python skip
blueprint = autoconnect(
    MyLidar.blueprint(),
    PointCloudConsumer.blueprint(),
).transports({
    ("pointcloud", PointCloud2): LCMTransport("/my/custom/lidar", PointCloud2),
})
```

## Logging

NativeModule pipes subprocess stdout and stderr through structlog:

- **stdout** is logged at `info` level.
- **stderr** is logged at `warning` level.

### JSON log format

If your native binary outputs structured JSON lines, set `log_format=LogFormat.JSON`:

```python skip
@dataclass(kw_only=True)
class MyConfig(NativeModuleConfig):
    executable: str = "./build/my_module"
    log_format: LogFormat = LogFormat.JSON
```

The module will parse each line as JSON and feed the key-value pairs into structlog. The `event` key becomes the log message:

```json
{"event": "sensor initialized", "device": "/dev/ttyUSB0", "baud": 115200}
```

Malformed lines fall back to plain text logging.

## Writing the C++ side

A header-only helper is provided at [`dimos/hardware/sensors/lidar/common/dimos_native_module.hpp`](/dimos/hardware/sensors/lidar/common/dimos_native_module.hpp):

```cpp
#include "dimos_native_module.hpp"
#include "sensor_msgs/PointCloud2.hpp"

int main(int argc, char** argv) {
    dimos::NativeModule mod(argc, argv);

    // Get the LCM channel for a declared port
    std::string pc_topic = mod.topic("pointcloud");

    // Get config values
    float freq = mod.arg_float("frequency", 10.0);
    std::string ip = mod.arg("host_ip", "192.168.1.5");

    // Set up LCM publisher and publish on pc_topic...
}
```

The helper provides:

| Method                    | Description                                                    |
|---------------------------|----------------------------------------------------------------|
| `topic(port)`             | Get the full LCM channel string (`/topic#msg_type`) for a port |
| `arg(key, default)`       | Get a string config value                                      |
| `arg_float(key, default)` | Get a float config value                                       |
| `arg_int(key, default)`   | Get an int config value                                        |
| `has(key)`                | Check if a port/arg was provided                               |

It also includes `make_header()` and `time_from_seconds()` for building ROS-compatible stamped messages.

## Examples

For language interop examples (subscribing to DimOS topics from C++, TypeScript, Lua), see [/examples/language-interop/](/examples/language-interop/README.md).

### Livox Mid-360 Module

The Livox Mid-360 LiDAR driver is a complete example at [`dimos/hardware/sensors/lidar/livox/module.py`](/dimos/hardware/sensors/lidar/livox/module.py):

```python skip
from dimos.core import Out
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.spec import perception

@dataclass(kw_only=True)
class Mid360Config(NativeModuleConfig):
    cwd: str | None = "cpp"
    executable: str = "result/bin/mid360_native"
    build_command: str | None = "nix build .#mid360_native"
    host_ip: str = "192.168.1.5"
    lidar_ip: str = "192.168.1.155"
    frequency: float = 10.0
    enable_imu: bool = True
    frame_id: str = "lidar_link"
    # ... SDK port configuration

class Mid360(NativeModule, perception.Lidar, perception.IMU):
    default_config = Mid360Config
    lidar: Out[PointCloud2]
    imu: Out[Imu]
```

Usage:

```python skip
from dimos.hardware.sensors.lidar.livox.module import Mid360

autoconnect(
    Mid360.blueprint(host_ip="192.168.1.5"),
    SomeConsumer.blueprint(),
)
```

## Auto Building

If `build_command` is set in the module config, and the executable doesn't exist when `start()` is called, NativeModule runs the build command automatically.
Build output is piped through structlog (stdout at `info`, stderr at `warning`).

```python skip
@dataclass(kw_only=True)
class MyLidarConfig(NativeModuleConfig):
    cwd: str | None = "cpp"
    executable: str = "result/bin/my_lidar"
    build_command: str | None = "nix build .#my_lidar"
```

`cwd` is used for both the build command and the runtime subprocess. Relative paths are resolved against the directory of the Python file that defines the module

If the executable already exists, the build step is skipped entirely.
