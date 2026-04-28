# Configuration

Dimos provides a `Configurable` base class. See [`service/spec.py`](/dimos/protocol/service/spec.py#L22).

This allows using pydantic models to specify configuration structure and default values per module.

```python
from dimos.protocol.service import Configurable
from dimos.protocol.service.spec import BaseConfig
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
except TypeError as e:
    print(f"Error: {e}")


```

<!--Result:-->
```
Config(x=3, hello='world')
Config(x=3, hello='override')
Error: Config.__init__() got an unexpected keyword argument 'something'
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
    tf_transport=<class 'dimos.protocol.tf.tf.LCMTF'>,
    frame_id_prefix=None,
    frame_id='frame_id_override',
    publish_interval=0,
    voxel_size=0.05,
    device='CPU'
)
```
