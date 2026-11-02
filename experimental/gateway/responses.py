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

from __future__ import annotations

import os
from typing import Any, Literal

from pydantic import ConfigDict, with_config
from typing_extensions import NotRequired, TypedDict

# Under pytest (which exports PYTEST_VERSION, as dimos's .env rule uses) a field a response
# doesn't declare fails the test; a running gateway passes it through rather than drop it.
CONFIG = ConfigDict(extra="forbid" if "PYTEST_VERSION" in os.environ else "allow")


@with_config(CONFIG)
class Body(TypedDict):
    pass


class ServerInfo(Body):
    exe: str
    exeModified: int | None
    kind: Literal["dimos"]
    startedAt: int
    zenohNamespace: str | None


class Paths(Body):
    """Where dimos keeps its runs, logs and recordings."""

    dimosDir: str
    runsDir: str
    logsDirs: list[str]
    recordingsDir: str


class Info(Body):
    """The dimos checkout this gateway drives, the python that runs it, its directories and this process."""

    dir: str
    found: bool
    installed: bool
    version: str | None
    api: str
    commit: str | None
    tag: str | None
    python: str
    paths: Paths
    server: ServerInfo


class Stopping(Body):
    stopping: bool


class Param(Body):
    """A parameter: its type's name, its default as python source, and its choices (a Literal's or an Enum's values)."""

    name: str
    type: str | None
    default: str | None
    required: bool
    choices: list[Any] | None


class Method(Body):
    """An RPC; `skill` says whether it's also a skill."""

    name: str
    params: list[Param]
    return_type: str | None
    doc: str
    skill: bool


class Stream(Body):
    name: str
    type: str
    direction: str
    wire: str | None
    topic: str | None


BlueprintModule = TypedDict(
    "BlueprintModule",
    {
        "name": str,
        "class": str,
        "doc": str,
        "summary": str,
        "streams": list[Stream],
        "rpcs": list[Method],
        "file": str | None,
        "line": int | None,
    },
)
with_config(CONFIG)(BlueprintModule)


class Arg(Body):
    """One config field: its type, default and description, and whether it's secret."""

    type: str | None
    default: Any
    description: str | None
    required: bool
    secret: bool
    base: bool
    json_compatible: bool
    choices: NotRequired[list[Any]]
    schema: NotRequired[dict[str, Any]]
    value: NotRequired[Any]


ModuleArgs = TypedDict(
    "ModuleArgs", {"class": str, "args": dict[str, Arg], "error": NotRequired[str]}
)
with_config(CONFIG)(ModuleArgs)

BlueprintSchema = TypedDict(
    "BlueprintSchema",
    {"name": str, "global": dict[str, Arg], "modules": dict[str, ModuleArgs]},
)
"""A blueprint's config schema: GlobalConfig's fields, and each module's by instance name."""
with_config(CONFIG)(BlueprintSchema)

ModuleSchema = TypedDict("ModuleSchema", {"name": str, "class": str, "args": dict[str, Arg]})
with_config(CONFIG)(ModuleSchema)


class GlobalConfig(Body):
    """dimos's GlobalConfig: its JSON schema, defaults and which keys are secret."""

    schema: dict[str, Any]
    defaults: dict[str, Any]
    secrets: list[str]


class ScanStatus(Body):
    """The blueprint scan's cache and how stale it is: when it finished, how long ago, and whether the checkout or
    venv changed since (stale); and the running or last scan's job."""

    job: str | None
    state: Literal["idle", "scanning", "failed"]
    scanned: bool
    finished_at: str | None
    age_seconds: int | None
    stale: bool


class ScanStarted(Body):
    """The scan's job: follow it at /jobs/{job}."""

    job: str


# every /catalog answer takes ?fields=, so none of their fields are required
class BlueprintFields(Body, total=False):
    name: str
    kind: Literal["builtin", "external"]
    ref: str | None
    robot: str | None
    doc: str | None
    title: str | None
    description: str | None
    starter: int | None
    hidden: bool
    recommended_config: list[dict[str, Any]]
    importable: bool | None
    import_error: str | None
    missing_module: str | None
    optional_dependency: bool | None
    suggested_extras: list[str]
    scanned_at: str | None


class CatalogBlueprint(BlueprintFields, total=False):
    """A blueprint: what annotations.yaml says of it, and what the last scan found (null before one)."""

    modules: list[str]


class BlueprintDetail(BlueprintFields, total=False):
    """A blueprint with its file and line, and its modules' streams and RPCs."""

    file: str | None
    line: int | None
    modules: list[BlueprintModule]
    # set while it runs from another checkout: the one it was read from, or why that failed
    checkout: str | None
    checkoutError: str | None


class BlueprintList(Body, total=False):
    blueprints: list[CatalogBlueprint]
    scan: ScanStatus


class PortType(Body):
    name: str
    type: str


ModuleFields = TypedDict(
    "ModuleFields",
    {
        "name": str,
        "class": str,
        "doc": str,
        "inputs": list[PortType],
        "outputs": list[PortType],
        "robots": list[str],
        "blueprints": list[str],
    },
    total=False,
)
with_config(CONFIG)(ModuleFields)


class CatalogModule(ModuleFields, total=False):
    rpcs: list[str]


class ModuleDetail(ModuleFields, total=False):
    """A module, the blueprints using it, and its RPCs."""

    rpcs: list[Method]
    file: str | None
    line: int | None


class ModuleList(Body, total=False):
    modules: list[CatalogModule]
    scan: ScanStatus


class CatalogRpc(Method, total=False):
    module: str
    robots: list[str]
    rpc: str


class CatalogRpcs(Body, total=False):
    rpcs: list[CatalogRpc]
    scan: ScanStatus


class CatalogRobot(Body, total=False):
    name: str
    description: str
    type: str | None
    manufacturer: str | None
    dirs: list[str]
    recommended: list[str]
    blueprints: list[str]


class Robots(Body, total=False):
    """Every robot dimos supports, by type, with its blueprints by name."""

    types: dict[str, dict[str, Any]]
    robots: dict[str, CatalogRobot]


class Catalog(Body, total=False):
    """Every blueprint, module, RPC and robot the last scan found."""

    blueprints: list[CatalogBlueprint]
    modules: list[CatalogModule]
    rpcs: list[CatalogRpc]
    robots: dict[str, CatalogRobot]
    types: dict[str, dict[str, Any]]
    errors: list[str]
    scan: ScanStatus


class MessageType(Body):
    """A message type's definition (not filled in yet)."""

    name: str
    definition: dict[str, Any] | None


class Extra(Body):
    name: str
    installed: bool
    applicable: bool
    requires: list[str]
    includes: list[str]
    missing: list[str]
    download_bytes: int | None


class Extras(Body):
    """dimos's optional extras, and which this python has installed."""

    mode: Literal["checkout", "library"]
    python: str
    extras: list[Extra]


class JobStarted(Body):
    """The job running the install: follow it at /jobs/{job}?after=."""

    shell: None
    job: str
    command: list[str]


class JobProgress(Body):
    done: int
    total: int
    current: str | None


class JobStatus(Body):
    """A job, whether it finished and how it went, and its progress when it has one."""

    job: str
    title: str
    kind: str
    done: bool
    ok: bool | None
    progress: JobProgress | None
    error: str | None
    started_at: str
    finished_at: str | None


class JobLog(JobStatus):
    """A job, whether it finished, its progress, and its output from line `after` on."""

    command: list[str]
    lines: list[str]
    next: int
    failure: list[str]


LaunchConfigShown = TypedDict(
    "LaunchConfigShown", {"global": dict[str, Any], "modules": dict[str, dict[str, Any]]}
)
"""A launch's config, with secret values shown as •••."""
with_config(CONFIG)(LaunchConfigShown)


class Launch(Body):
    """The blueprint this gateway launched last: its phase, output so far and the config it ran with. `runId` is
    the run registry's, null until the run registers."""

    blueprint: str
    phase: Literal["starting", "running", "stopping", "stopped", "failed"]
    startedAt: str
    pid: int
    output: str
    runId: str | None
    logDir: str | None
    error: str | None
    config: LaunchConfigShown
    args: NotRequired[list[str]]


class Active(Body):
    """The running blueprint (however it was started), and the gateway's last launch."""

    running: bool
    runId: str | None
    pid: int | None
    blueprint: str | None
    startedAt: str | None
    logDir: str | None
    launch: Launch | None


class ActiveId(Body):
    runId: str


class Stopped(Body):
    """What `dimos stop` said."""

    output: str


class LiveRpc(Body):
    """A running instance's RPC; params, return_type and skill come from the last scan (null if it hasn't seen it)."""

    name: str
    module: str
    doc: str
    params: list[Param] | None
    return_type: str | None
    skill: bool | None


class LiveStream(Body):
    name: str
    type: str
    direction: str | None
    channel: str | None
    module: str


Instance = TypedDict(
    "Instance",
    {
        "id": str,
        "class": str,
        "catalogId": str,
        "namespace": str | None,
        "doc": str,
        "rpcs": list[LiveRpc],
        "streams": list[LiveStream],
    },
)
"""A running module instance: `id` is `namespace:module` (or `module`), `catalogId` its class path for /catalog/modules/{name}."""
with_config(CONFIG)(Instance)


class Instances(Body):
    modules: list[Instance]


class LiveRpcs(Body):
    rpcs: list[LiveRpc]


class Skills(Body):
    skills: list[LiveRpc]


class LiveStreams(Body):
    streams: list[LiveStream]


class CallResult(Body):
    """What the RPC returned (as JSON, or its text if it isn't JSON-able), or the error it raised."""

    ok: bool
    result: Any
    error: str | None


class ModuleAction(Body):
    ok: bool
    error: str | None


class Peeked(Body):
    name: str
    type: str
    value: Any
    ts: float | None


class ActiveBlueprint(Body):
    """What was launched and the config it got (secrets shown as •••), and its running instances."""

    name: str
    config: LaunchConfigShown
    modules: list[str]


class LogRecord(Body):
    timestamp: str
    level: str
    logger: str
    event: str
    extra: dict[str, Any]
    raw: str


class PastRun(Body):
    """A run with a log: `endedAt` is its log's last record's time (null while it runs)."""

    runId: str
    blueprint: str | None
    startedAt: str | None
    endedAt: str | None
    running: bool


class History(Body):
    """Every run with a log, newest first."""

    runs: list[PastRun]


class PastRunDetail(PastRun):
    logDir: str


class Recordings(Body):
    """Where dimos writes recordings (mcap and db files)."""

    path: str


class RunLog(Body):
    """A run's structured log records, from offset `after` on."""

    runId: str
    records: list[LogRecord]
    offset: int
    loggers: list[str]


class ProcessStats(Body):
    """dtop's numbers for the running blueprint (its /resource_stats, sent when it runs with `dtop`): `up` false until heard."""

    up: bool
    age: float | None
    stale: bool
    coordinator: dict[str, Any] | None
    workers: list[dict[str, Any]]
    cpuHistory: dict[str, list[float]]
