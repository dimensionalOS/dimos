# Configuration

We offer a simple `Configurable` class, see [`service/spec.py`](/dimos/protocol/service/spec.py#L22)

They allow us to use dataclasses to specify configuration structure and default values per module.

```python
from dimos.protocol.service import Configurable
from rich import print
from dataclasses import dataclass

@dataclass
class Config():
    x: int = 3
    hello: str = "world"

class MyClass(Configurable):
    default_config = Config
    config: Config
    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)

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

[Modules]() inherit from `Configurable` so all of the above applies

```python
from dataclasses import dataclass
from dimos.core import In, Module, Out, rpc, ModuleConfig
from rich import print

@dataclass
class Config(ModuleConfig):
    frame_id: str = "world"
    publish_interval: float = 0
    voxel_size: float = 0.05
    device: str = "CUDA:0"

class MyModule(Module):
    default_config = Config
    config: Config

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        print(self.config)


myModule = MyModule(frame_id="frame_id_override", device="CPU")

# note in production we would actually call
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
