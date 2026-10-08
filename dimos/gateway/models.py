# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ruff: noqa: N815  (field names are the wire format's, camelCase as Desktop's)

"""The /dimos API's request bodies, answers and events, as pydantic models: what openapi.json is generated from.

The gateway builds its answers as plain dicts; FastAPI validates each against the route's model. Models allow extra
fields (an answer never loses one), and the tests check no answer carries one the model doesn't declare.
"""

from __future__ import annotations

import typing
from typing import Annotated, Any, Literal

from pydantic import BaseModel, ConfigDict, Field, JsonValue, WithJsonSchema, create_model

from dimos.core.coordination.blueprints import StreamRef
from dimos.core.run_registry import RunEntry
from dimos.gateway.diagnose import ProblemCode, StepCode


class ApiModel(BaseModel):
    model_config = ConfigDict(extra="allow", populate_by_name=True)


class ErrorResponse(ApiModel):
    """Every error answer (4xx and 5xx)."""

    error: str = Field(
        description="What went wrong, in words a person can act on",
        examples=["no upload u7"],
    )


# server


class Info(ApiModel):
    """The dimos checkout this gateway launches runs from."""

    dir: str = Field(description="The checkout's folder", examples=["/home/me/dimos"])
    found: bool = Field(description="A dimos checkout (pyproject.toml naming dimos) is there")
    installed: bool = Field(description="Its `.venv/bin/dimos` exists, so it can run blueprints")
    version: str | None = Field(
        description="Its pyproject version (null: none found)", examples=["0.0.14"]
    )
    range: str = Field(
        description="The dimos versions Desktop supports ($DESKTOP_DIMOS_RANGE); empty = any",
        examples=[">=0.0.14b1 <0.1"],
    )
    inRange: bool = Field(description="`version` is inside `range` (a launch is refused when not)")


class TopicRate(ApiModel):
    """One topic the gateway has heard (or seen declared) on the bus."""

    topic: str = Field(
        description="Its name, from the key `dimos/<topic>/<type>`", examples=["/lidar"]
    )
    type: str = Field(description="Its message type", examples=["sensor_msgs.PointCloud2"])
    hz: float = Field(description="Messages per second over the last 2 s (0 when quiet)")
    bps: float = Field(description="Bytes per second over the last 2 s")
    messages: int = Field(description="Messages heard since the gateway started")
    lastSeen: float | None = Field(
        description="Seconds since its last message; null if never heard (declared only)"
    )
    declared: bool = Field(
        description="A publisher of it is declared on the bus (zenoh liveliness)"
    )


class TopicRates(ApiModel):
    """Every topic on the bus the gateway has heard since it started, busiest first."""

    up: bool = Field(description="The gateway is listening to the bus")
    error: str | None = Field(description="Why it isn't, or null")
    topics: list[TopicRate] = Field(description="Busiest first, then by name")


class Paths(ApiModel):
    """Where dimos keeps its things."""

    dimosDir: str = Field(description="The checkout", examples=["/home/me/dimos"])
    runsDir: str = Field(
        description="dimos's run registry (one <run_id>.json per live run)",
        examples=["/home/me/.local/state/dimos/runs"],
    )
    logsDirs: list[str] = Field(
        description="Where runs' logs are, searched in order: the checkout's logs/, then the library install's",
        examples=[["/home/me/dimos/logs", "/home/me/.local/state/dimos/logs"]],
    )
    recordingsDir: str = Field(
        description="Desktop's recordings folder (config.yaml `recordings.dir`), else where dimos records "
        "(its RECORDINGS_DIR)",
        examples=["/home/me/.dimos/recordings"],
    )
    server: ServerProgram = Field(description="What this gateway runs")


class PythonCommand(ApiModel):
    """The python that runs dimos, for an agent to use dimos's python API."""

    python: str = Field(
        description="Absolute path of the interpreter dimos runs with (not symlink-resolved: a venv's python)",
        examples=["/home/me/dimos/.venv/bin/python"],
    )
    command: list[str] = Field(
        description="The argv to start it, before your own arguments",
        examples=[["/home/me/dimos/.venv/bin/python"]],
    )
    dimosDir: str = Field(
        description="The checkout it imports dimos from", examples=["/home/me/dimos"]
    )
    version: str = Field(description="Its python version", examples=["3.12.11"])
    dimosVersion: str | None = Field(
        description="The checkout's pyproject version (null: none found)", examples=["0.0.14"]
    )
    env: dict[str, str] = Field(
        description="Environment variables to set for `import dimos` to find the checkout; usually empty (the "
        "checkout is installed in its venv), else `PYTHONPATH`",
        examples=[{}],
    )
    example: str = Field(
        description="A shell command that runs it, with `env`",
        examples=["/home/me/dimos/.venv/bin/python -c 'import dimos; print(dimos.__file__)'"],
    )


class ServerProgram(ApiModel):
    exe: str | None = Field(
        description="The program this gateway runs (for this gateway, its python)",
        examples=["/home/me/dimos/.venv/bin/python"],
    )
    exeModified: int | None = Field(
        description="Its modification time when the gateway started (Unix s): Desktop restarts its built-in gateway "
        "when its own binary has been replaced since"
    )
    kind: Literal["dimos", "builtin"] = Field(
        description="Whose gateway answers: `dimos` (this one, dimos's own) or `builtin` (Desktop's)"
    )
    startedAt: int | None = Field(
        description="When this gateway started (Unix s): Desktop restarts it once a file of the checkout's "
        "dimos/gateway/ or dimos.yaml is newer",
    )
    zenohNamespace: str | None = Field(
        description="The namespace this gateway publishes its events under (`<ns>/dimos/events/<type>`), "
        "null when it publishes on SSE only: Desktop relays the SSE stream onto zenoh only then",
        examples=["dimos-desktop/jeffs-mac-5555"],
    )


class Stopping(ApiModel):
    stopping: Literal[True] = Field(description="Always true: the gateway exits in 0.2 s")


# blueprints


class BlueprintName(ApiModel):
    name: str = Field(description="The name `dimos run` takes", examples=["unitree-go2-basic"])
    kind: Literal["builtin", "external"] = Field(
        description="builtin: in dimos itself; external: from an installed package's entry points"
    )
    importable: bool | None = Field(
        default=None,
        description="From the discovery cache: it imports (null: not scanned yet; GET /dimos/discovery)",
    )
    import_error: str | None = Field(
        default=None, description="From the discovery cache: why it doesn't import"
    )
    missing_module: str | None = Field(
        default=None, description="From the discovery cache: the module its import couldn't find"
    )


class BlueprintList(ApiModel):
    blueprints: list[BlueprintName] = Field(
        description="Built-in blueprints (sorted, without demo-*), then external ones"
    )


# dimos's own StreamRef.direction (a Literal; mypy sees the str it is)
if typing.TYPE_CHECKING:
    StreamDirection = str
else:
    StreamDirection = typing.get_type_hints(StreamRef)["direction"]


class Stream(ApiModel):
    name: str = Field(description="The stream's name on its module", examples=["color_image"])
    type: str = Field(description="Its message type", examples=["dimos.msgs.sensor_msgs.Image"])
    direction: StreamDirection = Field(
        description="in: the module reads it; out: it publishes it; inout: both"
    )
    topic: str | None = Field(
        default=None,
        description="The topic it's wired to in this blueprint (a transport_map pin, else /<name> after remapping when "
        "that name has one type; null when it gets a random topic at run time)",
        examples=["/color_image"],
    )


class SkillParam(ApiModel):
    name: str = Field(description="The parameter", examples=["distance"])
    type: str | None = Field(description="Its annotation, if any", examples=["float"])
    default: str | None = Field(description="repr() of its default (null: none)", examples=["1.0"])


class ModuleMethod(ApiModel):
    """An RPC method (or a skill: an RPC an agent can call) on a module."""

    name: str = Field(description="The method", examples=["get_battery_soc"])
    params: list[SkillParam] = Field(description="Its parameters (self left out)")
    return_type: str | None = Field(description="Its return annotation, if any", examples=["float"])
    doc: str = Field(description="Its whole docstring (empty when it has none)")


class BlueprintModule(ApiModel):
    name: str = Field(description="The module's name in the blueprint", examples=["camera"])
    class_: str = Field(
        alias="class",
        description="Its Python class",
        examples=["dimos.hardware.camera.CameraModule"],
    )
    streams: list[Stream] = Field(description="Its streams")
    doc: str | None = Field(
        default=None,
        description="The class's whole docstring (or the nearest base's that isn't dimos's Module plumbing; empty when "
        "none); absent from servers before API 1.7",
    )
    summary: str | None = Field(
        default=None,
        description="The docstring's first paragraph, on one line (at most 300 characters)",
    )
    file: str | None = Field(
        default=None,
        description="The file defining the class: relative to the dimos checkout when inside it (GET /dimos/source "
        "reads it), else absolute; null when unknown",
        examples=["dimos/hardware/camera/module.py"],
    )
    line: int | None = Field(
        default=None, description="The class statement's line in that file (1-based)"
    )
    rpcs: list[ModuleMethod] | None = Field(
        default=None,
        description="Its RPC methods that aren't skills, less the ones every module has (build, start, stop, ...)",
    )
    skills: list[ModuleMethod] | None = Field(default=None, description="Its skills")


class SourceFile(ApiModel):
    file: str = Field(
        description="The file, as asked for", examples=["dimos/hardware/camera/module.py"]
    )
    text: str = Field(description="Its contents")


class Blueprint(ApiModel):
    name: str = Field(description="The blueprint", examples=["unitree-go2-basic"])
    file: str | None = Field(
        default=None,
        description="The file defining the blueprint (a module run as one: its class): relative to the dimos checkout "
        "when inside it (GET /dimos/source reads it), else absolute; null when unknown; absent from servers before "
        "API 1.8",
        examples=["dimos/robot/unitree/go2/blueprints/basic/unitree_go2_basic.py"],
    )
    line: int | None = Field(
        default=None, description="The line that defines it in that file (1-based)"
    )
    modules: list[BlueprintModule] = Field(description="Its modules, in blueprint order")


class ConfigArg(ApiModel):
    """One field of a module's pydantic `config` model."""

    name: str = Field(description="The field", examples=["robot_ip"])
    type: str | None = Field(description="Its annotation", examples=["str | None"])
    default: JsonValue = Field(description="Its default (null when required)", examples=[None])
    description: str | None = Field(description="The field's description, if it has one")
    required: bool = Field(description="It has no default")
    base: bool = Field(description="Inherited from dimos's ModuleConfig (every module has it)")
    choices: list[JsonValue] | None = Field(
        default=None,
        description="Only for an Enum or Literal: the values it takes",
        examples=[["webrtc", "ros"]],
    )
    value: JsonValue = Field(
        default=None,
        description="Only when the blueprint sets it: the value it sets (••• for a secret)",
    )
    json_compatible: bool | None = Field(
        default=None,
        description="Absent when pydantic can't be imported. It can be set from a launch or saved config: its `schema` could be built, its default "
        "survives JSON, and it isn't a nested model (dimos sets those field by field)",
    )
    json_schema: dict[str, JsonValue] | None = Field(
        default=None,
        alias="schema",
        description="Only when it can be built: the field's own JSON Schema (its $refs point into its own $defs); "
        "a launch's value for it is checked against it",
        examples=[{"type": "boolean"}],
    )
    secret: bool = Field(
        description="Its name says it's a secret (key, *_key, secret, token, password, passwd): shown as ••• and "
        "passed to dimos in the environment, never on the command line"
    )


class ModuleConfig(ApiModel):
    module: str = Field(description="The module's name in the blueprint", examples=["camera"])
    class_: str = Field(alias="class", description="Its Python class")
    args: list[ConfigArg] = Field(description="Its configurable args (empty when `error`)")
    error: str | None = Field(
        default=None,
        description="Only when its config couldn't be read: why",
        examples=["TypeError: CameraModule.config (dict) isn't a pydantic model"],
    )


class BlueprintConfig(ApiModel):
    name: str = Field(description="The blueprint", examples=["unitree-go2"])
    modules: list[ModuleConfig] = Field(description="Each module's configurable args")
    overrides: dict[str, dict[str, JsonValue]] = Field(
        description="Desktop's saved module config for this blueprint (config.yaml `dimos.module_config.<name>`), "
        "`--<module>.<field>=value` on every launch of it; secrets as •••",
        examples=[{"go2connection": {"lidar": False}}],
    )


class BlueprintConfigUpdate(ApiModel):
    overrides: dict[str, dict[str, JsonValue]] = Field(
        description="The blueprint's new saved module config, {module: {field: value}}, replacing the old one; null "
        "drops a field; ••• keeps a saved secret",
        examples=[{"go2connection": {"lidar": False}, "voxelgridmapper": {"voxel_size": 0.1}}],
    )


class Port(ApiModel):
    name: str = Field(description="The stream's name", examples=["odom"])
    type: str = Field(description="Its message type's name", examples=["Odometry"])


class CatalogBlueprint(ApiModel):
    name: str = Field(description="The blueprint", examples=["unitree-go2"])
    ref: str = Field(
        description="Where it's defined, `module:attribute`",
        examples=["dimos.robot.unitree.go2.blueprints.basic:unitree_go2"],
    )
    robot: str | None = Field(
        description="The robot folder it's under (null: none)", examples=["go2"]
    )
    modules: list[str] = Field(description="Its modules' catalog names")
    doc: str = Field(
        description="Its file's docstring's first paragraph, when the file defines only this blueprint (else empty)"
    )


class CatalogModule(ApiModel):
    name: str = Field(description="The module's catalog name", examples=["CameraModule"])
    class_: str = Field(alias="class", description="Its Python class, `module.QualName`")
    doc: str = Field(description="Its docstring's first paragraph (at most 300 characters)")
    robots: list[str] = Field(description="Robots whose blueprints use it")
    inputs: list[Port] = Field(description="Streams it reads")
    outputs: list[Port] = Field(description="Streams it publishes")
    skills: list[str] = Field(description="Its skills' names")


class CatalogSkill(ApiModel):
    name: str = Field(description="The skill", examples=["move"])
    doc: str = Field(description="Its docstring's first paragraph")
    params: list[SkillParam] = Field(description="Its parameters")
    module: str = Field(description="The module it's on")
    robots: list[str] = Field(description="Robots whose blueprints have that module")


class Catalog(ApiModel):
    blueprints: list[CatalogBlueprint] = Field(description="Every built-in blueprint")
    modules: list[CatalogModule] = Field(description="Every module")
    skills: list[CatalogSkill] = Field(description="Every skill, on every module")
    errors: list[str] = Field(
        description="What couldn't be imported (at most 50)",
        examples=[["blueprint g1-sim: ImportError: no mujoco"]],
    )


# robots (dimos/gateway/robots.json, resolved)


class RobotType(ApiModel):
    label: str = Field(
        description="The heading a launcher groups robots of this type under", examples=["Dogs"]
    )


class SettingChoice(ApiModel):
    label: str = Field(description="What to call it", examples=["Simulator", "Qwen"])
    value: JsonValue = Field(
        default=None, description="The value it sets (a config value's choice)", examples=["qwen"]
    )
    set: dict[str, JsonValue] | None = Field(
        default=None,
        description="The config values it sets together (a pick's choice): GlobalConfig field or <module>.<field> to "
        "its value",
        examples=[{"replay": False, "simulation": "mujoco"}],
    )


class RecommendedSetting(ApiModel):
    """A setting to decide before running a blueprint: one config value (an arg: `key`, `scope`, ...), or a pick
    (`kind` "pick") whose choices each set several values. Either may be an enum (`choices`) and shown only `when`
    some config values hold."""

    id: str = Field(
        description="Its key (a GlobalConfig field or <module>.<field>); pick-<n> for a pick",
        examples=["go2_ip", "detection_model", "pick-0"],
    )
    key: str | None = Field(
        description="The config value, as dimos's options name it (null for a pick)",
        examples=["robot_ip", "spothighlevel.ip"],
    )
    scope: Literal["global", "module"] | None = Field(
        description="GlobalConfig, or a module's config (null for a pick)"
    )
    global_: str | None = Field(
        default=None, alias="global", description="The GlobalConfig field (scope global)"
    )
    module: str | None = Field(
        default=None,
        description="The module's name in the blueprint (scope module)",
        examples=["spothighlevel"],
    )
    field: str | None = Field(
        default=None, description="The module's config field (scope module)", examples=["ip"]
    )
    label: str = Field(description="What to ask", examples=["Robot IP", "Run it on"])
    kind: Literal["text", "number", "bool", "recording", "pick"] = Field(
        description="text, number, bool, recording (pick a dimos recording), or pick (choose one of `choices`, "
        "each setting several values)"
    )
    placeholder: str | None = Field(
        default=None, description="An example value", examples=["192.168.12.1"]
    )
    default: JsonValue = Field(
        default=None,
        description="The value it starts with; for a pick, the label of the choice it starts on",
    )
    required: bool = Field(description="The blueprint won't run without it (while it is shown)")
    type: Literal["string", "boolean", "integer", "number", "json"] | None = Field(
        default=None,
        description="The GlobalConfig field's type (read from GlobalConfig; null for a module field)",
    )
    nullable: bool | None = Field(default=None, description="The GlobalConfig field takes null")
    description: str | None = Field(
        default=None, description="The GlobalConfig field's description"
    )
    streams: list[list[str]] | None = Field(
        default=None,
        description="recording: the streams it must have, each a list of acceptable names",
        examples=[[["go2_lidar", "lidar"], ["color_image"]]],
    )
    docs: str | None = Field(
        default=None,
        description="A page on how to find this value (null: none)",
        examples=["https://docs.dimensional.org/platforms/quadruped/go2/setup/"],
    )
    choices: list[SettingChoice] | None = Field(
        default=None,
        description="The options (an enum: buttons for up to 4, else a select); null: a free value",
    )
    when: dict[str, JsonValue] | None = Field(
        default=None,
        description="Show it only while these config values hold (null: always)",
        examples=[{"replay": True}],
    )


class RobotBlueprint(ApiModel):
    title: str = Field(description="A short name", examples=["Go2 basic"])
    description: str = Field(description="What it does and what it needs, plainly")
    starter: int | None = Field(
        description='Its rank among the "start here" picks (1 first), or null'
    )
    hidden: bool = Field(
        description="A test, benchmark, mock or building block: list it only on request"
    )
    recommended_config: list[RecommendedSetting] = Field(
        default_factory=list,
        description="The settings to decide before running it, in order (its own, else its robot's): where it runs "
        "(robot, a recording, a simulator), its IP, ...",
    )
    robot: str = Field(description="Its robot's id", examples=["go2"])
    registered: bool = Field(description="dimos's blueprint registry has it")


class Robot(ApiModel):
    name: str = Field(description="Its name", examples=["Unitree Go2"])
    description: str = Field(description="What it is, in a sentence")
    type: str | None = Field(
        description="What kind of robot: a key of `types` (dog, humanoid, wheeled, arm, drone); null when it isn't a "
        "robot dimos drives (sensors, demos, simulators, coordinators)",
        examples=["dog"],
    )
    manufacturer: str | None = Field(
        description="Who makes it (how its name starts), or null (DIY, open-source, generic, not a robot)",
        examples=["Unitree"],
    )
    dirs: list[str] = Field(
        description="Its code directories in the checkout", examples=[["dimos/robot/unitree/go2"]]
    )
    recommended: list[str] = Field(
        default_factory=list,
        description="The blueprints to suggest first for it, best first (each one of its `blueprints`); empty "
        "when robots.json names none",
        examples=[["unitree-go2-basic", "unitree-go2"]],
    )
    blueprints: dict[str, RobotBlueprint] = Field(description="Its blueprints by name, in order")


class Robots(ApiModel):
    """dimos/gateway/robots.json with its defaults applied."""

    about: str | None = Field(default=None, description="What the file is")
    types: dict[str, RobotType] = Field(
        description="The robot types, in the order a launcher groups robots by (robots without one come last)"
    )
    robots: dict[str, Robot] = Field(description="Every robot by id, in display order")
    excluded: dict[str, str] = Field(
        description="Directories under dimos/robot that aren't robots, and why",
        examples=[{"dimos/robot/assets": "robot model assets"}],
    )
    unlisted: list[str] = Field(
        description="Registered blueprints no robot lists (outside the robots' directories)",
        examples=[["demo-new-thing"]],
    )


# global config


class GlobalConfig(ApiModel):
    json_schema: dict[str, JsonValue] = Field(
        alias="schema", description="dimos's GlobalConfig as a JSON Schema (draft 2020-12)"
    )
    defaults: dict[str, JsonValue] = Field(
        description="Each field's default (fields whose default is computed are left out)",
        examples=[{"n_workers": 2, "simulation": False}],
    )
    overrides: dict[str, JsonValue] = Field(
        description="Desktop's overrides (config.yaml `dimos.global_config`): `--key value` on every launch; "
        "secrets as •••",
        examples=[{"robot_ip": "192.168.12.1"}],
    )
    secrets: list[str] = Field(
        description="The GlobalConfig keys named like a secret: shown as •••, passed in the environment",
        examples=[["unitree_aes_128_key", "dimos_api_key"]],
    )


class GlobalConfigUpdate(ApiModel):
    overrides: dict[str, JsonValue] = Field(
        description="The new overrides, {key: value}, replacing the old ones; a null value removes that key. "
        "Keys are GlobalConfig fields (each one `dimos` takes as a `--key` flag)",
        examples=[{"robot_ip": "192.168.12.1", "simulation": None}],
    )


# runs


def _run_entry_fields(docs: dict[str, tuple[str, list[Any]]]) -> dict[str, Any]:
    """These fields of dimos's RunEntry, with its types: the answer can't drift from what the registry holds."""
    hints = typing.get_type_hints(RunEntry)
    return {
        name: (hints[name], Field(description=text, examples=examples))
        for name, (text, examples) in docs.items()
    }


RegistryRun = create_model(
    "RegistryRun",
    __base__=ApiModel,
    __doc__="A live run in dimos's run registry (also runs started from a terminal): a RunEntry's fields.",
    **_run_entry_fields(
        {
            "run_id": ("The run's id", ["20260101-120000-unitree-go2"]),
            "pid": ("Its process id", [41233]),
            "blueprint": ("What it runs", ["unitree-go2"]),
            "started_at": ("When it started (ISO 8601)", ["2026-01-01T12:00:00+00:00"]),
            "log_dir": (
                "Its log folder (main.jsonl is there)",
                ["/home/me/dimos/logs/20260101-120000-unitree-go2"],
            ),
        }
    ),
)


class LaunchStep(ApiModel):
    """A startup step, from the `stage` records dimos logs as it starts. The words for it are the client's."""

    code: StepCode = Field(
        description="starting (dimos began), building (the blueprint), starting_modules, then running (in the run "
        "registry) or stopped",
        examples=["starting_modules"],
    )
    state: Literal["done", "now", "todo", "failed"] = Field(description="How far it got")
    data: dict[str, JsonValue] = Field(
        description="For starting_modules: `deployed` (modules started so far) and `total` (null until known)",
        examples=[{"deployed": 3, "total": 7}],
    )


class LaunchProblem(ApiModel):
    """Something that went wrong, from the launch's error records: a stable code and that record's data. The words
    and the fix for each code are the client's (Desktop's)."""

    code: ProblemCode = Field(
        description="What kind of problem; `error` is one without a known kind (its `message` says what)",
        examples=["missing_python_package"],
    )
    level: Literal["error"] = Field(description="How bad")
    message: str = Field(
        description="The record's own message (the exception's, else the log event's), for a person who wants the "
        "detail",
        examples=["No module named 'unitree_sdk2py'"],
    )
    data: dict[str, JsonValue] = Field(
        description="The record's fields, e.g. `missing_module`, `exception_code` (an errno name), `module` and "
        "`method` (where in a worker it failed), `requirement`, `name`",
        examples=[{"missing_module": "unitree_sdk2py", "module": "G1Connection", "method": None}],
    )
    timestamp: str = Field(description="When it was logged (ISO 8601)")
    logger: str = Field(
        description="Where it was logged", examples=["dimos/core/coordination/python_worker.py"]
    )


class LaunchOneOff(ApiModel):
    global_: dict[str, JsonValue] = Field(alias="global", description="{GlobalConfig key: value}")
    modules: dict[str, dict[str, JsonValue]] = Field(description="{module: {field: value}}")
    secrets: list[str] | None = Field(
        default=None, description="Only when the request listed some: its extra secret paths"
    )


class Launch(ApiModel):
    """The last launch this gateway started; its phase is worked out from disk on every call."""

    blueprint: str = Field(description="What was launched", examples=["unitree-go2"])
    phase: Literal["starting", "running", "stopping", "stopped", "failed"] = Field(
        description="starting: alive, not registered yet; running: in dimos's run registry (every module built); "
        "stopping: asked to stop (or out of the registry) and still exiting; stopped: ran, or was stopped, and is "
        "gone; failed: exited before running without being asked to"
    )
    startedAt: str = Field(
        description="When it was launched (ISO 8601, UTC)", examples=["2026-01-01T12:00:00Z"]
    )
    pid: int = Field(
        description="The `dimos run` process (its own process group)", examples=[41233]
    )
    output: str = Field(
        description="The end of its output (at most 200 kB), starting with the command line"
    )
    runId: str | None = Field(description="Its registry run id, once running")
    logDir: str | None = Field(description="Its log folder, once running")
    error: str | None = Field(
        description="When failed: the output's first `Error: ` line, else its last line",
        examples=["Error: no blueprint named unitree-go3"],
    )
    overrides: dict[str, JsonValue] = Field(
        description="The GlobalConfig it was launched with (Desktop's saved overrides, then the launch's own, then "
        "replay); POST /dimos/runs/restart launches with them again",
        examples=[{"replay": True, "n_workers": 2}],
    )
    modules: dict[str, dict[str, JsonValue]] = Field(
        description="The module config it was launched with (Desktop's saved one for this blueprint, then the "
        "launch's own); secrets as •••",
        examples=[{"go2connection": {"lidar": False}}],
    )
    oneOff: LaunchOneOff = Field(
        description="What the launch request itself set, as sent (nulls kept, `replay` added); secrets as •••"
    )
    steps: list[LaunchStep] = Field(
        description="starting, building, starting_modules, then running or stopped: how far startup got"
    )
    problems: list[LaunchProblem] = Field(
        description="What went wrong: the errors of a known kind, else the last three errors"
    )


class RunList(ApiModel):
    runs: list[RegistryRun] = Field(description="Live runs, newest first")  # type: ignore[valid-type]
    launch: Launch | None = Field(description="The launch this gateway started (null: none yet)")


class LaunchRequest(ApiModel):
    blueprint: str = Field(description="blueprint name", examples=["unitree-go2"])
    replay: bool = Field(default=False, description="Run on a recording: adds `--replay`")
    # any JSON, so a wrong shape gets the gateway's own 400 message (`overrides must be an object, not list`)
    overrides: Annotated[
        JsonValue,
        WithJsonSchema(
            {
                "type": "object",
                "properties": {
                    "global": {"type": "object", "description": "{GlobalConfig key: value}"},
                    "modules": {
                        "type": "object",
                        "description": "{module (its name in the blueprint): {field: value}}",
                    },
                    "secrets": {
                        "type": "array",
                        "items": {"type": "string"},
                        "description": "More paths to treat as secrets (`robot_ip`, `<module>.<field>`)",
                    },
                },
            }
        ),
    ] = Field(
        default=None,
        description="This launch's own config, on top of Desktop's saved config and never saved: "
        "`{global?, modules?, secrets?}`, or (the older form) a flat `{GlobalConfig key: value}`. null drops a "
        "saved value for this launch; ••• keeps a saved secret. Checked against dimos's schemas: 400 with the path "
        "and why. Secret values (by name, or listed) go to `dimos run` as environment variables, never argv.",
        examples=[
            {
                "global": {"n_workers": 4, "robot_ip": None},
                "modules": {"go2connection": {"lidar": False}},
            },
            {"simulation": "mujoco"},
        ],
    )


class StopRequest(ApiModel):
    runId: str | None = Field(
        default=None,
        description="run id (any live run in the registry); none: the launch this gateway started",
        examples=["20260101-120000-unitree-go2"],
    )


class StopResult(ApiModel):
    output: str = Field(
        description="What was stopped", examples=["stopped unitree-go2 (pid 41233)"]
    )


# skills: the running blueprints' `@skill` methods, through each run's MCP server (skills.py)


class Skill(ApiModel):
    name: str = Field(
        description="The skill (the module's method name)", examples=["execute_sport_command"]
    )
    module: str = Field(
        description="The module it belongs to: its RPC name (the class name, or the instance name a blueprint gave "
        "it)",
        examples=["UnitreeSkillContainer"],
    )
    description: str = Field(
        description="Its docstring", examples=["Execute a Unitree sport command."]
    )
    params: dict[str, Any] = Field(
        description="Its arguments as a JSON Schema object (`properties`, `required`), as McpServer gives an agent",
        examples=[
            {
                "type": "object",
                "properties": {"command_name": {"type": "string"}},
                "required": ["command_name"],
            }
        ],
    )
    required: list[str] = Field(
        description="The arguments it can't go without", examples=[["command_name"]]
    )
    lifecycle: str = Field(
        description="`instant` (the call answers when it's done) or `background` (it answers at once and keeps "
        "running until its stop skill)",
        examples=["instant"],
    )
    uses: list[str] = Field(
        description="Capabilities it holds while it runs (another skill using one waits or is refused)",
        examples=[["locomotion"]],
    )
    runId: str | None = Field(
        description="The run it belongs to (null: a coordinator not started by `dimos run`)",
        examples=["20260101-120000-unitree-go2"],
    )
    blueprint: str | None = Field(description="That run's blueprint", examples=["unitree-go2"])


class SkillRun(ApiModel):
    runId: str | None = Field(
        description="The running blueprint's run (null: a coordinator not started by `dimos run`)",
        examples=["20260101-120000-unitree-go2"],
    )
    blueprint: str | None = Field(description="Its blueprint", examples=["unitree-go2"])


class SkillModuleError(ApiModel):
    module: str = Field(description="A module that didn't list its skills", examples=["NativeSlam"])
    error: str = Field(
        description="Why", examples=["TimeoutError: RPC call timed out after 3.0 seconds"]
    )


class SkillList(ApiModel):
    skills: list[Skill] = Field(
        description="Every skill of the running blueprint's modules, by name; empty when none runs"
    )
    run: SkillRun | None = Field(description="What runs; null when nothing answers on the RPC bus")
    errors: list[SkillModuleError] = Field(
        description="Modules whose skills couldn't be read (their skills aren't listed)"
    )


class SkillCallRequest(ApiModel):
    skill: str = Field(
        description="The skill's name, as GET /dimos/skills lists it",
        examples=["execute_sport_command"],
    )
    args: dict[str, Any] = Field(
        default_factory=dict,
        description="Its arguments, by its `params` schema",
        examples=[{"command_name": "FrontJump"}],
    )
    module: str | None = Field(default=None, description="Only this module's skill of that name")
    runId: str | None = Field(
        default=None, description="Refuse unless this is the running run (default: whichever runs)"
    )


class SkillCallResult(ApiModel):
    skill: str = Field(description="The skill called", examples=["execute_sport_command"])
    module: str = Field(description="Its module", examples=["UnitreeSkillContainer"])
    runId: str | None = Field(
        description="The run it ran in", examples=["20260101-120000-unitree-go2"]
    )
    blueprint: str | None = Field(description="That run's blueprint", examples=["unitree-go2"])
    via: str = Field(
        description="`rpc` (dimos's module RPC) or `mcp` (the run's McpServer, for a skill that holds a capability)",
        examples=["rpc"],
    )
    ok: bool = Field(description="False when the skill failed (`text` says why)")
    text: str = Field(
        description="Its answer's text", examples=["'FrontJump' command executed successfully."]
    )
    content: list[dict[str, Any]] = Field(
        description="Its whole answer as MCP content blocks (`{type: text, text}`, `{type: image, data, mimeType}`)"
    )


# logs


class LogRecord(ApiModel):
    """One line of a run's main.jsonl (structlog JSON); a line that isn't JSON is a `raw` record."""

    timestamp: str = Field(description="As logged", examples=["2026-01-01T12:00:01.123Z"])
    level: str = Field(
        description="debug, info, warning, error or critical (lowercased); raw for a non-JSON line",
        examples=["warning"],
    )
    logger: str = Field(description="The logger's name", examples=["dimos.navigation"])
    event: str = Field(description="The message", examples=["no path to goal"])
    extra: dict[str, JsonValue] = Field(description="Every other field of the record")
    raw: str = Field(description="The line as written")


class LogPage(ApiModel):
    runId: str | None = Field(
        description="The run read (`latest` resolved)", examples=["20260101-120000-unitree-go2"]
    )
    records: list[LogRecord] = Field(description="Matching records, oldest first")
    offset: int = Field(description="Byte offset to pass back as `after` for only newer records")
    loggers: list[str] = Field(description="Every logger name seen, sorted (for a filter menu)")


# cloud


class Account(ApiModel):
    loggedIn: bool = Field(
        description="A cloud key is stored (or in the environment) and wasn't refused"
    )
    email: str | None = Field(description="Who it belongs to", examples=["me@example.com"])
    scopes: str | list[str] | None = Field(
        description="What it may do, as the cloud reports it (today a string, e.g. `data`)",
        examples=["data"],
    )
    source: Literal["env", "stored"] | None = Field(
        description="env: DIMOS_API_KEY; stored: `dimos login`'s saved key; null: none"
    )
    cloudUrl: str = Field(
        description="The Dimensional cloud asked", examples=["https://api.dimensional.org"]
    )
    error: str | None = Field(
        description="Why it couldn't be checked, or that the key was revoked",
        examples=["The saved login was revoked or is invalid: log in again."],
    )


class Login(ApiModel):
    """The device login: a URL and a code the user approves in any browser signed in to Dimensional cloud."""

    state: Literal["idle", "starting", "pending", "approved", "denied", "expired", "failed"] = (
        Field(
            description="idle: none; starting: asking for a code; pending: waiting for approval; then approved, denied, "
            "expired or failed"
        )
    )
    url: str | None = Field(
        description="Where to approve it", examples=["https://console.dimensional.org/device"]
    )
    urlComplete: str | None = Field(description="The same URL with the code filled in")
    code: str | None = Field(description="The code to show", examples=["ABCD-EFGH"])
    expiresAt: int | None = Field(description="When the code expires (Unix ms)")
    email: str | None = Field(description="Who approved it")
    error: str | None = Field(description="Why it failed")


# uploads


class Upload(ApiModel):
    """One upload in the queue."""

    id: str = Field(description="The upload's id", examples=["u3"])
    path: str = Field(
        description="The recording", examples=["/home/me/.dimos/recordings/walk.mcap"]
    )
    name: str = Field(description="Its file name", examples=["walk.mcap"])
    size: int = Field(description="Its size in bytes when queued")
    robotId: str | None = Field(description="The robot id it's tagged with", examples=["go2-lab"])
    kind: str | None = Field(
        description="recording, video, pointcloud, log or blob (null: from the file)"
    )
    state: Literal["queued", "uploading", "done", "failed", "cancelled"] = Field(
        description="queued (first in, first out), uploading (one at a time), then done, failed or cancelled"
    )
    phase: str | None = Field(
        description="While uploading: preparing, compress, hash, upload, then finishing",
        examples=["upload"],
    )
    bytesDone: int = Field(description="Bytes done in this phase")
    bytesTotal: int = Field(description="Bytes in this phase (0: indeterminate)")
    rateBps: float | None = Field(description="Smoothed speed in bytes a second")
    etaSeconds: float | None = Field(
        description="Time left in this phase (null for the first second)"
    )
    uploadId: str | None = Field(description="The cloud's id for it, once done")
    skipped: bool = Field(description="The cloud had it already")
    notice: str | None = Field(description="Something to tell the user, e.g. about the quota")
    error: str | None = Field(description="Why it failed, or that it waits for a login")
    errorCode: str | None = Field(
        description="not_logged_in, network, quota, file_missing or failed",
        examples=["not_logged_in"],
    )
    log: str | None = Field(description="The log with the details of a failure")
    createdAt: int = Field(description="When it was queued (Unix ms)")
    startedAt: int | None = Field(description="When its upload started (Unix ms)")
    finishedAt: int | None = Field(description="When it finished (Unix ms)")
    link: str | None = Field(description="Its page in the Dimensional console, once done")


class UploadList(ApiModel):
    uploads: list[Upload] = Field(description="The queue, in order")
    waitingForLogin: bool = Field(
        description="An upload found no cloud login: the queue waits for one"
    )


class UploadRequest(ApiModel):
    path: str = Field(
        description="the recording's absolute path (a /recordings entry's path): an .mcap or a .db",
        examples=["/home/me/.dimos/recordings/walk.mcap"],
    )
    robotId: str | None = Field(
        default=None, description="robot id to tag it with", examples=["go2-lab"]
    )
    kind: str | None = Field(
        default=None,
        description="recording (default for .mcap/.db), video, pointcloud, log, blob",
        examples=["recording"],
    )


class Uploaded(ApiModel):
    """A recording that is in the cloud (remembered even after the upload list is cleared)."""

    path: str = Field(
        description="The recording", examples=["/home/me/.dimos/recordings/walk.mcap"]
    )
    uploadId: str = Field(description="The cloud's id for it")
    size: int = Field(description="Its size when uploaded")
    mtimeMs: int = Field(description="Its modification time when uploaded (Unix ms)")
    uploadedAt: int = Field(description="When the upload finished (Unix ms)")
    link: str | None = Field(description="Its page in the Dimensional console")
    changed: bool = Field(
        description="The file's size or modification time differs from the uploaded one"
    )


class UploadedByPath(ApiModel):
    byPath: dict[str, Uploaded] = Field(description="Every uploaded recording, by path")


class Ok(ApiModel):
    ok: Literal[True] = Field(description="Always true")


# discovery: the cache of every blueprint, module and message type (discovery.py)


class DiscoveryStatus(ApiModel):
    """Where the discovery scan is."""

    state: Literal["idle", "scanning", "done", "failed"] = Field(
        description="idle: not started; scanning: a scan runs; done: the answer is complete; failed: the scan itself "
        "failed (see errors)"
    )
    reason: str | None = Field(
        description="Why the last scan started: startup, changed (checkout or packages), requested, extras installed",
        examples=["startup"],
    )
    key: str | None = Field(
        description="The cache key the answer is for (checkout commit + dirty files + installed packages, hashed)",
        examples=["d44db6593b78cd25"],
    )
    stale: bool = Field(
        description="The checkout or its packages changed since the answer was made; a new one is being made"
    )
    started_at: str | None = Field(description="When the last scan started (UTC, ISO 8601)")
    finished_at: str | None = Field(description="When the answer was completed (UTC, ISO 8601)")
    blueprints_total: int = Field(
        description="Blueprints in dimos's registry (and installed packages)"
    )
    blueprints_done: int = Field(description="Blueprints with an answer so far")
    importable: int = Field(description="Of those, how many import")
    not_importable: int = Field(
        description="Of those, how many don't (see each one's import_error)"
    )
    modules_total: int = Field(description="Modules in dimos's module registry")
    modules_done: int = Field(description="Registry modules with an answer so far")
    current: str | None = Field(
        description="What the scan is importing now (a blueprint, or module:<name>)",
        examples=["unitree-go2"],
    )
    cached_from: str | None = Field(
        description="When the answer being served was saved, while it comes from the disk cache"
    )
    errors: list[str] = Field(
        description="Problems with the scan itself (a crash, a hang), newest last, at most 50; a blueprint that "
        "just doesn't import is not one"
    )


class DiscoveryRefresh(ApiModel):
    full: bool = Field(
        default=False,
        description="Forget the answer and import everything again (else only what's missing or changed)",
    )


class DiscoveredStream(ApiModel):
    name: str = Field(description="The stream's name on its module", examples=["color_image"])
    type: str = Field(
        description="Its message type, `module.Class`",
        examples=["dimos.msgs.sensor_msgs.Image.Image"],
    )
    direction: Literal["in", "out", "inout"] = Field(
        description="in: the module reads it; out: it publishes it; inout: both"
    )
    topic: str | None = Field(
        description="The topic it's on when the blueprint runs: a transport the blueprint pins, else `/<name>` "
        "(after remapping) when only one type uses that name; null: a random topic at run time",
        examples=["/color_image"],
    )


class DiscoveredModuleRef(ApiModel):
    name: str = Field(description="The module's name in the blueprint", examples=["go2connection"])
    class_: str = Field(alias="class", description="Its Python class, `module.QualName`")
    module: str = Field(
        description="Its name in dimos's module registry (else its class name): what /dimos/modules/{module} takes",
        examples=["go2-connection"],
    )
    streams: list[DiscoveredStream] = Field(description="Its streams")


class DiscoveredBlueprint(ApiModel):
    name: str = Field(description="The blueprint", examples=["unitree-go2"])
    ref: str | None = Field(
        description="Where a built-in one is defined, `module:attribute` (null: external, or the scan failed)"
    )
    builtin: bool | None = Field(
        description="In dimos's own registry (false: an installed package's)"
    )
    robot: str | None = Field(
        description="The robot it's for: the robots.json robot (GET /dimos/robots) that lists it or whose dirs hold "
        "its file; without a robots.json, the robot folder under dimos/robot/ it lives in (null: none)",
        examples=["go2"],
    )
    importable: bool = Field(description="It imports in the checkout's python")
    optional_dependency: bool = Field(
        description="It doesn't import because an optional dependency (an extra) is missing, by dimos's own rule"
    )
    import_error: str | None = Field(
        description="Why it doesn't import, `<Exception>: <message>`",
        examples=["ModuleNotFoundError: No module named 'unitree_sdk2py'"],
    )
    import_traceback: str | None = Field(
        description="The import's traceback (its last 4000 characters)"
    )
    missing_module: str | None = Field(
        description="The top-level module an import couldn't find", examples=["unitree_sdk2py"]
    )
    suggested_extras: list[str] = Field(
        description="dimos extras that would install missing_module, smallest first: those requiring its package "
        "(by name, `cv2` as opencv-python), else bringing it as a dependency (uv.lock); none for a package dimos "
        "needs without extras. A hint",
        examples=[["unitree-dds"]],
    )
    modules: list[DiscoveredModuleRef] = Field(
        description="Its modules, in blueprint order (empty when it doesn't import)"
    )


class DiscoveredBlueprints(ApiModel):
    stale: bool = Field(
        description="From before the checkout or packages changed (a scan is running)"
    )
    blueprints: list[DiscoveredBlueprint] = Field(description="Every blueprint scanned so far")


class ModuleSummary(ApiModel):
    name: str = Field(
        description="Its name in dimos's module registry (else its class name)",
        examples=["go2-connection"],
    )
    class_: str = Field(alias="class", description="Its Python class, `module.QualName`")
    doc: str = Field(description="Its own docstring's first paragraph (empty: none)")
    inputs: list[Port] = Field(description="Streams it reads (type: `module.Class`)")
    outputs: list[Port] = Field(description="Streams it publishes (type: `module.Class`)")
    skills: list[str] = Field(description="Its skills' names")
    blueprint_count: int = Field(
        description="How many importable blueprints use it (RerunBridgeModule: most)", examples=[89]
    )
    robots: list[str] = Field(description="Robots whose blueprints use it")
    error: str | None = Field(
        default=None, description="Only when its streams couldn't be read: why"
    )
    config_error: str | None = Field(
        default=None, description="Only when its config couldn't be read: why"
    )


class ModuleList(ApiModel):
    stale: bool = Field(
        description="From before the checkout or packages changed (a scan is running)"
    )
    modules: list[ModuleSummary] = Field(
        description="Every module class: the registry's and any a blueprint uses, by name"
    )


class ConfigField(ApiModel):
    """One field of a module's config class (pydantic model or dataclass)."""

    name: str = Field(description="The field", examples=["robot_ip"])
    type: str | None = Field(description="Its annotation", examples=["str | None"])
    default: JsonValue = Field(
        description="Its default as JSON (null when required); when not json_compatible, its text"
    )
    description: str | None = Field(description="The field's description, if it has one")
    required: bool = Field(description="It has no default")
    base: bool = Field(description="Inherited from dimos's ModuleConfig (every module has it)")
    enum: list[JsonValue] | None = Field(
        description="An Enum's values or a Literal's options (also inside Optional); null otherwise",
        examples=[["webrtc", "ros"]],
    )
    json_compatible: bool = Field(
        description="Its type has a JSON Schema and its default survives a JSON round trip unchanged; a UI leaves "
        "out the fields that don't"
    )
    reason: str | None = Field(
        description="Why it isn't json_compatible", examples=["its type has no JSON form"]
    )


class ModuleConfigAnswer(ApiModel):
    module: str = Field(
        description="The module's registry name (else class name)", examples=["go2-connection"]
    )
    class_: str = Field(alias="class", description="Its Python class")
    fields: list[ConfigField] = Field(
        description="Its config's fields, but ModuleConfig's internal ones (g, rpc_transport, ...)"
    )
    error: str | None = Field(description="Why its config couldn't be read (then fields is empty)")


class MessageType(ApiModel):
    type: str = Field(
        description="The message type, `module.Class`",
        examples=["dimos.msgs.geometry_msgs.Twist.Twist"],
    )
    publishers: list[str] = Field(description="Modules with an output of this type")
    subscribers: list[str] = Field(description="Modules with an input of this type")


class MessageTypes(ApiModel):
    types: list[MessageType] = Field(description="Every message type a module's stream has, sorted")


class RankedModule(ApiModel):
    name: str = Field(
        description="The module (registry name, else class name)", examples=["go2-connection"]
    )
    class_: str = Field(alias="class", description="Its Python class")
    score: float = Field(
        description="How specific it is to this robot (see `formula`); higher first"
    )
    in_robot_blueprints: int = Field(description="This robot's importable blueprints that use it")
    robot_blueprints: int = Field(description="This robot's importable blueprints")
    robots_using: int = Field(description="Robots with a blueprint that uses it")
    blueprint_count: int = Field(description="Importable blueprints (any robot) that use it")


class RobotModules(ApiModel):
    robot: str = Field(description="The robot", examples=["go2"])
    blueprints: int = Field(description="Its blueprints (importable or not)")
    blueprints_importable: int = Field(description="Of those, how many import (only these count)")
    robots_total: int = Field(description="Robots with an importable blueprint")
    formula: str = Field(description="How `score` is computed")
    modules: list[RankedModule] = Field(description="Its blueprints' modules, most specific first")


# docs


class DocPage(ApiModel):
    title: str = Field(description="The page's first heading")
    source_path: str = Field(
        description="The file, relative to the checkout", examples=["docs/usage/cli.md"]
    )
    url: str | None = Field(
        description="Where the docs site publishes it (null: no site_url in mkdocs.yml)"
    )


class CustomRobotDoc(ApiModel):
    title: str = Field(
        description="The guide's first heading", examples=["How to Integrate a New Manipulator Arm"]
    )
    markdown: str = Field(
        description="The guide, links and images made absolute (docs site, else GitHub)"
    )
    html: str | None = Field(
        description="The same rendered to HTML with markdown-it (CommonMark + tables, raw HTML off); null when "
        "markdown-it isn't installed"
    )
    source_path: str = Field(description="The file, relative to the checkout")
    url: str | None = Field(description="Where the docs site publishes it")
    others: list[DocPage] = Field(description="Other pages that matched, best first")


class DocLinks(ApiModel):
    """Links into dimos's published docs, found in the checkout's docs/ and mkdocs.yml; null where there's no page."""

    site: str | None = Field(
        description="The docs site (mkdocs.yml site_url)",
        examples=["https://docs.dimensional.org/"],
    )
    repo: str | None = Field(description="The repo (mkdocs.yml repo_url)")
    configure_robot: str | None = Field(
        description="Configuring dimos for a robot (GlobalConfig, flags, env)",
        examples=["https://docs.dimensional.org/usage/configuration/"],
    )
    custom_robot: str | None = Field(
        description="Adding a robot of your own (GET /dimos/docs/custom-robot)"
    )
    blueprints: str | None = Field(description="Blueprints")
    modules: str | None = Field(description="Modules")
    installation: str | None = Field(description="Installing dimos")
    quickstart: str | None = Field(description="Quickstart")
    cli: str | None = Field(description="The dimos CLI")


# extras and jobs


class Extra(ApiModel):
    name: str = Field(description="The extra, as in `dimos[<name>]`", examples=["sim"])
    installed: bool = Field(
        description="Every requirement that applies here is installed at an allowed version (and every included "
        "extra is installed)"
    )
    applicable: bool = Field(
        description="Some requirement applies on this machine (false: e.g. cuda on a Mac; nothing to install)"
    )
    requires: list[str] = Field(description="Its requirements, as pyproject.toml has them")
    includes: list[str] = Field(description="Other extras it pulls in (`dimos[base,mapping]`)")
    missing: list[str] = Field(
        description="Packages (its own and its includes') not installed or at a wrong version"
    )
    download_bytes: int | None = Field(
        description="A hint: bytes to download for what's missing and what that depends on, from uv.lock's wheel "
        "sizes for this OS and CPU (an upper bound; null without a uv.lock)"
    )


class ExtrasList(ApiModel):
    mode: Literal["checkout", "library"] = Field(
        description="checkout: a source checkout installed with `uv sync`; library: dimos installed as a package"
    )
    python: str = Field(description="The python the extras are checked in (the checkout's .venv)")
    extras: list[Extra] = Field(description="Every extra, in pyproject.toml order")


class ExtrasInstall(ApiModel):
    extras: list[str] = Field(description="Extras to add", min_length=1, examples=[["sim"]])
    app: str | None = Field(
        None,
        description="The Desktop app whose page shows the install (default `launcher`)",
        examples=["launcher"],
    )


class ExtrasInstallStarted(ApiModel):
    shell: str | None = Field(
        description="Desktop's shell session running it: follow Desktop's GET /api/desktop/shell/{id}?wait= "
        "(null without Desktop)"
    )
    job: str | None = Field(
        description="Without Desktop, the job running it: follow `<ns>/dimos/jobs/<job>` or GET /dimos/jobs/{job}/log"
    )
    command: list[str] = Field(description="The uv command it runs")


class JobStarted(ApiModel):
    job: str = Field(
        description="The job's id: follow `<ns>/dimos/jobs/<job>` or GET /dimos/jobs/{job}/log"
    )
    command: list[str] = Field(description="What it runs")


class JobSummary(ApiModel):
    job: str = Field(description="The job's id", examples=["extras-1-1791000000"])
    title: str = Field(description="What it does, for a person", examples=["Install extras: sim"])
    kind: str = Field(description="What sort of job", examples=["extras"])
    done: bool = Field(description="It finished")
    ok: bool | None = Field(description="It succeeded (null while running)")
    started_at: str = Field(description="When it started (UTC, ISO 8601)")
    finished_at: str | None = Field(description="When it finished")


class JobList(ApiModel):
    jobs: list[JobSummary] = Field(
        description="Running jobs and those finished in the last 30 minutes"
    )


class JobLog(JobSummary):
    command: list[str] = Field(description="What it runs")
    lines: list[str] = Field(
        description="Its output lines from `after` on (stdout and stderr together)"
    )
    next: int = Field(description="The `n` the next line will have (pass it as `after`)")
    error: str | None = Field(
        description="That it failed, and its exit code (or `cancelled`, or why it couldn't start)",
        examples=["Install extras: sim failed (exit 2)"],
    )
    failure: list[str] = Field(
        description="Only after a failure: the output's last 15 non-empty lines, where the command says why (the "
        "whole output is `lines`)"
    )
    code: Literal["cyclonedds_missing"] | None = Field(
        description="Why it couldn't run, when its preparation found something missing (`error` says what to do): "
        "`cyclonedds_missing` = the cyclonedds package must be built against the CycloneDDS C library and none was "
        "found or could be fetched (no nix, no Homebrew one, no $CYCLONEDDS_HOME). null otherwise",
        examples=[None],
    )


# events: on zenoh at <ns>/dimos/events/<type>, and (deprecated) the SSE stream /dimos/events


def _zenoh(kind: str, text: str) -> ConfigDict:
    return ConfigDict(
        json_schema_extra={"description": text, "x-zenoh-key": f"<ns>/dimos/events/{kind}"},
    )


class LaunchEvent(ApiModel):
    model_config = _zenoh(
        "launch", "The launch's (blueprint, phase, runId) changed; also first on every SSE connect"
    )
    type: Literal["launch"]
    launch: Launch | None = Field(description="The launch now (null: none)")


class LogEvent(ApiModel):
    model_config = _zenoh("log", "A new warning-or-worse record in the current launch's main.jsonl")
    type: Literal["log"]
    runId: str | None = Field(description="The launch's run id")
    record: LogRecord = Field(description="The record")


class UploadEvent(ApiModel):
    model_config = _zenoh("upload", "An upload changed (progress: a few times a second at most)")
    type: Literal["upload"]
    upload: Upload = Field(description="The upload now")


class UploadsEvent(ApiModel):
    model_config = _zenoh("uploads", "The queue as a whole changed: GET /dimos/uploads for it")
    type: Literal["uploads"]
    waitingForLogin: bool = Field(description="The queue waits for a cloud login")
    cleared: bool | None = Field(default=None, description="Only after DELETE /dimos/uploads: true")


class UploadRemovedEvent(ApiModel):
    model_config = _zenoh("upload-removed", "A finished upload was removed from the list")
    type: Literal["upload-removed"]
    id: str = Field(description="Its id", examples=["u3"])


class CloudLoginEvent(ApiModel):
    model_config = _zenoh("cloud-login", "The device login's state changed")
    type: Literal["cloud-login"]
    login: Login = Field(description="The login now")


class DiscoveryEvent(ApiModel):
    model_config = _zenoh(
        "discovery",
        "The discovery scan started, moved (at most twice a second) or finished: GET /dimos/discovery/blueprints "
        "and friends for the data",
    )
    type: Literal["discovery"]
    status: DiscoveryStatus = Field(description="The scan now (GET /dimos/discovery)")


class JobEvent(ApiModel):
    model_config = _zenoh(
        "job",
        "A job started: its lines are on `<ns>/dimos/jobs/<job>` ({type: line, n, line}, then {type: done, ok, "
        "error, failure, lines})",
    )
    type: Literal["job"]
    job: str = Field(description="The job's id")
    title: str = Field(description="What it does")
    kind: str = Field(description="What sort of job", examples=["extras"])


class BlueprintsEvent(ApiModel):
    model_config = _zenoh(
        "blueprints",
        "The blueprint list changed (a file under dimos/robot or a package in site-packages, which the gateway "
        "watches): GET /dimos/blueprints for the new list",
    )
    type: Literal["blueprints"]
    added: list[str] = Field(
        description="Blueprints that weren't listed before", examples=[["unitree-g1"]]
    )
    removed: list[str] = Field(description="Blueprints no longer listed")


EVENT_MODELS: tuple[type[ApiModel], ...] = (
    LaunchEvent,
    LogEvent,
    UploadEvent,
    UploadsEvent,
    UploadRemovedEvent,
    CloudLoginEvent,
    DiscoveryEvent,
    JobEvent,
    BlueprintsEvent,
)

DimosEvent = Annotated[
    LaunchEvent
    | LogEvent
    | UploadEvent
    | UploadsEvent
    | UploadRemovedEvent
    | CloudLoginEvent
    | DiscoveryEvent
    | JobEvent
    | BlueprintsEvent,
    Field(discriminator="type"),
]
