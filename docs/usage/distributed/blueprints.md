# Placing modules

`.hosted(...)` marks part of a blueprint to run on a Host. Modules without it stay on the
blueprint root's own Host.

```python skip
from dimos.core.coordination.blueprints import autoconnect
from dimos.hosted.tags import Tag

app = autoconnect(
    robot_stack.hosted(tags={"go2": True}), # any Host tagged go2
    MeshModule.blueprint().hosted(tags={"gpu": True}, prefer=Tag.gpu_mem), # the GPU with most memory
    viewer, # stays here
)
```

| Argument                                  | Picks                                                             |
|-------------------------------------------|-------------------------------------------------------------------|
| none                                      | any Host                                                          |
| `host="jetson1"`                          | that Host, by name or ID                                          |
| `tags={"gpu": True, "site": "frankfurt"}` | Hosts with every tag: `True` for any value, else that exact value |
| `minimum={Tag.gpu_mem: 11}`               | Hosts with at least that much of each numeric tag                 |
| `prefer=Tag.gpu_mem`                      | among the matches, the highest value of that numeric tag          |
| `local=True`                              | the blueprint root's own Host                                         |

Hosts tag themselves; `HOST__TAGS` adds your own. What three machines might advertise:

| Machine    | Tags                                                                                    |
|------------|-----------------------------------------------------------------------------------------|
| laptop     | `arch=x86_64 cpus=20 ram_gb=62 gpu=RTX 5070 Laptop GPU gpu_count=1 gpu_mem_gb=8`        |
| GPU box    | `arch=x86_64 cpus=32 ram_gb=128 gpu=RTX 4090 gpu_count=2 gpu_mem_gb=24 site=athens`     |
| Go2 Jetson | `arch=aarch64 cpus=8 ram_gb=16 gpu=jetson gpu_count=1 jetson=NVIDIA Jetson Orin NX go2` |

`site=athens` and `go2` came from `HOST__TAGS`. Against these, `tags={"go2": True}` picks the
Jetson, `tags={"arch": "x86_64"}` the laptop or the GPU box, and `tags={"gpu": True},
prefer=Tag.gpu_mem` the GPU box: every machine has a `gpu`, the Jetson's memory is shared
so it reports none and ranks last. Among equal candidates the Host with the fewest active
runs wins. `dimos host ls` shows each Host's tags.

`Tag` names the numeric ones: `gpu_mem`, `gpu_count`, `ram`, `cpus`, and `free_cpus`,
`free_ram`, `free_gpu_mem`, which a Host measures fresh each time it is asked. Any other
numeric tag works by name, e.g. `minimum={"bla": 22}` against `HOST__TAGS=bla=30`.

Streams and module references cross Hosts on their own: a module that calls another
module's RPC on a different Host gets a remote client. Pin a chatty pair the same way to
keep it on one machine.

Globals set with `.global_config(...)` reach every Host. Leave per-machine ones such as
`n_workers` to each Host's `.env`.

[`go2-nav-hosted`](/dimos/mapping/experimental/blueprints.py) is a complete example.
