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
    range: str
    inRange: bool
    api: str
    commit: str | None
    tag: str | None
    python: str
    paths: Paths
    server: ServerInfo


class Stopping(Body):
    stopping: bool


class Param(Body):
    name: str
    type: str | None
    default: str | None


class Method(Body):
    name: str
    params: list[Param]
    return_type: str | None
    doc: str


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
        "skills": list[Method],
        "file": str | None,
        "line": int | None,
    },
)
with_config(CONFIG)(BlueprintModule)


class ConfigArg(Body):
    name: str
    type: str | None
    default: Any
    description: str | None
    required: bool
    base: bool
    json_compatible: bool
    secret: bool
    choices: NotRequired[list[Any]]
    schema: NotRequired[dict[str, Any]]
    value: NotRequired[Any]


ModuleConfig = TypedDict(
    "ModuleConfig",
    {"module": str, "class": str, "args": list[ConfigArg], "error": NotRequired[str]},
)
with_config(CONFIG)(ModuleConfig)


class CheckedArg(Body):
    tokens: list[str]
    target: str | None
    error: str | None


class CheckedArgs(Body):
    """Each group of CLI args, what it sets and why dimos would refuse it."""

    name: str
    args: list[CheckedArg]


class BlueprintConfig(Body):
    """A blueprint's configurable args per module, and the values Desktop saved for it."""

    name: str
    modules: list[ModuleConfig]
    overrides: dict[str, dict[str, Any]]


class Blueprint(Body):
    """A blueprint's modules: their streams (topic, type, direction), RPCs and skills; and its config."""

    name: str
    file: str | None
    line: int | None
    modules: list[BlueprintModule]
    config: BlueprintConfig


class GlobalConfig(Body):
    """dimos's GlobalConfig: its JSON schema, defaults, the saved overrides and which keys are secret."""

    schema: dict[str, Any]
    defaults: dict[str, Any]
    overrides: dict[str, Any]
    secrets: list[str]


class Robot(Body):
    name: str
    description: str
    type: str | None
    manufacturer: NotRequired[str | None]
    dirs: NotRequired[list[str]]
    recommended: list[str]
    blueprints: dict[str, RobotBlueprint]


class RobotBlueprint(Body):
    title: str
    description: str
    robot: str
    recommended_config: list[dict[str, Any]]
    starter: int | None
    hidden: bool
    registered: bool


class Robots(Body):
    """Every robot dimos supports and its blueprints, with each one's recommended settings."""

    about: None
    types: dict[str, dict[str, Any]]
    robots: dict[str, Robot]


ScannedModule = TypedDict(
    "ScannedModule", {"name": str, "class": str, "module": str, "streams": list[Stream]}
)
with_config(CONFIG)(ScannedModule)


class BlueprintEntry(Body):
    """A blueprint, and (once a scan finished) what the last scan found: whether it imports, and if not why."""

    name: str
    kind: NotRequired[Literal["builtin", "external"]]
    ref: NotRequired[str | None]
    robot: NotRequired[str | None]
    importable: NotRequired[bool]
    import_error: NotRequired[str | None]
    missing_module: NotRequired[str | None]
    optional_dependency: NotRequired[bool]
    suggested_extras: NotRequired[list[str]]
    modules: NotRequired[list[ScannedModule]]


class ScanStatus(Body):
    """The blueprint scan's cache and how stale it is: when it finished, how long ago, and whether the checkout or
    venv changed since (stale); and the running or last scan's job."""

    job: str | None
    state: Literal["idle", "scanning", "failed"]
    scanned: bool
    finished_at: str | None
    age_seconds: int | None
    stale: bool


class BlueprintList(Body):
    """Every blueprint dimos can run, and what the last scan found for each."""

    blueprints: list[BlueprintEntry]
    scan: ScanStatus


class ScanStarted(Body):
    """The scan's job: follow it at /jobs/{job}."""

    job: str


class CatalogBlueprint(Body):
    name: str
    ref: str | None
    robot: str | None
    modules: list[str]
    doc: str


class CatalogSkill(Body):
    name: str
    doc: str
    params: list[Param]
    module: str
    robots: list[str]


class PortType(Body):
    name: str
    type: str


CatalogModule = TypedDict(
    "CatalogModule",
    {
        "name": str,
        "class": str,
        "doc": str,
        "inputs": list[PortType],
        "outputs": list[PortType],
        "robots": list[str],
        "skills": list[str],
    },
)
with_config(CONFIG)(CatalogModule)


class Catalog(Body):
    """The importable built-in blueprints, every module and every skill, for browsing."""

    blueprints: list[CatalogBlueprint]
    modules: list[CatalogModule]
    skills: list[CatalogSkill]
    errors: list[str]
    scan: ScanStatus


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
    code: str | None


class RunEntry(Body):
    run_id: str
    pid: int
    blueprint: str
    started_at: str
    log_dir: str


OneOff = TypedDict(
    "OneOff",
    {
        "global": dict[str, Any],
        "modules": dict[str, dict[str, Any]],
        "secrets": NotRequired[list[str]],
    },
)
with_config(CONFIG)(OneOff)


class Launch(Body):
    """The blueprint this gateway launched: its phase, output so far and the config it ran with."""

    blueprint: str
    phase: Literal["starting", "running", "stopping", "stopped", "failed"]
    startedAt: str
    pid: int
    output: str
    runId: str
    logDir: str | None
    error: str | None
    overrides: dict[str, Any]
    modules: dict[str, dict[str, Any]]
    oneOff: OneOff
    args: NotRequired[list[str]]


class Runs(Body):
    """The running blueprints, and the launch this gateway started."""

    runs: list[RunEntry]
    launch: Launch | None


class Stopped(Body):
    """Which run was stopped."""

    output: str


class LogRecord(Body):
    timestamp: str
    level: str
    logger: str
    event: str
    extra: dict[str, Any]
    raw: str


class RunLog(Body):
    """A run's structured log records, from offset `after` on."""

    runId: str
    records: list[LogRecord]
    offset: int
    loggers: list[str]


class Login(Body):
    """The device login: the URL and code to open, until it's done, expired or cancelled."""

    state: str
    url: str | None
    urlComplete: str | None
    code: str | None
    expiresAt: int | None
    email: str | None
    error: str | None


class Account(Body):
    """Whether this machine is logged in to Dimensional cloud, and as whom."""

    loggedIn: bool
    email: str | None
    scopes: str | list[str] | None
    source: Literal["env", "stored"] | None
    cloudUrl: str
    error: str | None
    login: NotRequired[Login]


class Upload(Body):
    """One upload in the queue: its progress, and its link once done."""

    id: str
    path: str
    name: str
    size: int
    robotId: str | None
    kind: str | None
    state: str
    phase: str | None
    bytesDone: int
    bytesTotal: int
    rateBps: float | None
    etaSeconds: float | None
    uploadId: str | None
    skipped: bool
    notice: str | None
    error: str | None
    errorCode: str | None
    log: str | None
    createdAt: int
    startedAt: int | None
    finishedAt: int | None
    link: str | None


class Uploaded(Body):
    """A file uploaded before, and whether it changed since."""

    path: str
    uploadId: str
    size: int
    mtimeMs: int
    uploadedAt: int
    link: str | None
    changed: bool


class Uploads(Body):
    """The upload queue, whether it waits for a login, and every file uploaded before, by path."""

    uploads: list[Upload]
    waitingForLogin: bool
    byPath: dict[str, Uploaded]


class SkillRun(Body):
    runId: str | None
    blueprint: str | None


class Skill(Body):
    name: str
    module: str
    description: str
    params: dict[str, Any]
    required: list[str]
    lifecycle: str
    uses: list[str]
    runId: str | None
    blueprint: str | None


class SkillError(Body):
    module: str
    error: str


class Skills(Body):
    """The running blueprint's skills, with each one's JSON schema for its args."""

    skills: list[Skill]
    run: SkillRun | None
    errors: list[SkillError]


class SkillResult(Body):
    """What the skill answered, as text and as MCP content blocks."""

    skill: str
    module: str
    runId: str | None
    blueprint: str | None
    via: Literal["mcp", "rpc"]
    ok: bool
    text: str
    content: list[dict[str, Any]]


class SourceFile(Body):
    """A .py file of the dimos checkout."""

    file: str
    text: str
