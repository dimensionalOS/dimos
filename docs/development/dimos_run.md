# DimOS Run

#### Warning: If you just want to run a blueprint you don't need to add it to `dimos run`:

`your_code.py`
```py
from dimos.robot.unitree_webrtc.unitree_go2_blueprints import basic as example_blueprint

if __name__ == "__main__":
    example_blueprint.build().loop()
```

```sh
python ./your_code.py
```

## Usage

For example, to run the standard Unitree Go2 blueprint run:

```bash
dimos run unitree-go2
```

For the one with agents run:

```bash
dimos run unitree-go2-agentic
```

You can dynamically connect additional modules. For example:

```bash
dimos run unitree-go2 --extra-module agent --extra-module navigation_skill
```

## Adding your own

Blueprints can be defined anywhere, but they're all linked together in `dimos/robot/all_blueprints.py`. E.g.:

```python
all_blueprints = {
    "unitree-go2": "dimos.robot.unitree_webrtc.unitree_go2_blueprints:standard",
    "unitree-go2-agentic": "dimos.robot.unitree_webrtc.unitree_go2_blueprints:agentic",
    ...
}
```

(They are defined as imports to avoid triggering unrelated imports.)

## `GlobalConfig`

This tool also initializes the global config and passes it to the blueprint.

`GlobalConfig` contains configuration options that are useful across many modules. For example:

```python
class GlobalConfig(BaseSettings):
    robot_ip: str | None = None
    simulation: bool = False
    replay: bool = False
    n_dask_workers: int = 2
```

Configuration values can be set from multiple places in order of precedence (later entries override earlier ones):

- Default value defined on GlobalConfig. (`simulation = False`)
- Value defined in `.env` (`SIMULATION=true`)
- Value in the environment variable (`SIMULATION=true`)
- Value defined on the blueprint (`blueprint.global_config(simulation=True)`)
- Value coming from the CLI (`--simulation` or `--no-simulation`)

For environment variables/`.env` values, you have to prefix the name with `DIMOS_`.

For the command line, you call it like this:

```bash
dimos --simulation run unitree-go2
```
