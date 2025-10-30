# Robot CLI

To avoid having so many runfiles, I created a common script to run any blueprint.

For example, to run the standard Unitree Go2 blueprint run:

```bash
dimos-robot run unitree-go2
```

For the one with agents run:

```bash
dimos-robot run unitree-go2-agentic
```

You can dynamically connect additional modules. For example:

```bash
dimos-robot run unitree-go2 --extra-module llm_agent --extra-module human_input --extra-module navigation_skill
```

## Definitions

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
    use_simulation: bool = False
    use_replay: bool = False
    n_dask_workers: int = 2
```

Configuration values can be set from multiple places in order of precedence (later entries override earlier ones):

- Default value defined on GlobalConfig. (`use_simulation = False`)
- Value defined in `.env` (`USE_SIMULATION=true`)
- Value in the environment variable (`USE_SIMULATION=true`)
- Value coming from the CLI (`--use-simulation` or `--no-use-simulation`)
- Value defined on the blueprint (`blueprint.global_config(use_simulation=True)`)

For environment variables/`.env` values, you have to prefix the name with `DIMOS_`.

For the command line, you call it like this:

```bash
dimos-robot --use-simulation run unitree-go2
```