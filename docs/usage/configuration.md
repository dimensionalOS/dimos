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

<!--Result:-->
```
Config(x=3, hello='world')
Config(x=3, hello='override')
Error: 1 validation error for Config
something
  Extra inputs are not permitted
    For further information visit
https://errors.pydantic.dev/2.12/v/extra_forbidden
```

# Configurable Modules

[Modules](/docs/usage/modules.md) inherit from `Configurable`, so all of the above applies. Module configs should inherit from `ModuleConfig` ([`core/module.py`](/dimos/core/module.py#L40)), which includes shared configuration for all modules like transport protocols, frame IDs, etc.

```python
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from rich import print

class Config(ModuleConfig):
    frame_id: str = "world"
    publish_interval: float = 0
    voxel_size: float = 0.05
    device: str = "CUDA:0"

class MyModule(Module):
    config: Config

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        print(self.config)


myModule = MyModule(frame_id="frame_id_override", device="CPU")

# In production, use dimos.deploy() instead:
# myModule = dimos.deploy(MyModule, frame_id="frame_id_override")


```

<!--Result:-->
```
Config(
    rpc_transport=<class 'dimos.protocol.rpc.pubsubrpc.LCMRPC'>,
    default_rpc_timeout=120.0,
    rpc_timeouts={'build': 86400.0, 'start': 1200.0},
    tf_transport=<class 'dimos.protocol.tf.tf.LCMTF'>,
    frame_id_prefix=None,
    frame_id='frame_id_override',
    g=GlobalConfig(
        robot_ip=None,
        robot_ips=None,
        xarm7_ip=None,
        xarm6_ip=None,
        can_port=None,
        simulation=False,
        replay=False,
        replay_dir='go2_sf_office',
        new_memory=False,
        viewer='rerun',
        n_workers=2,
        memory_limit='auto',
        mujoco_camera_position=None,
        mujoco_room=None,
        mujoco_room_from_occupancy=None,
        mujoco_global_costmap_from_occupancy=None,
        mujoco_global_map_from_pointcloud=None,
        mujoco_start_pos='-1.0, 1.0',
        mujoco_steps_per_frame=7,
        robot_model=None,
        robot_width=0.3,
        robot_rotation_diameter=0.6,
        nerf_speed=1.0,
        planner_robot_speed=None,
        mcp_port=9990,
        dtop=False,
        obstacle_avoidance=True,
        detection_model='moondream',
        listen_host='127.0.0.1'
    ),
    publish_interval=0,
    voxel_size=0.05,
    device='CPU'
)
```
