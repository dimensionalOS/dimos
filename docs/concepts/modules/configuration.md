

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
