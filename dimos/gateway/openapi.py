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

"""The /dimos API's OpenAPI document: served at /dimos/openapi.json and checked in as openapi.json (dimos.yaml's
`api.openapi`, so Desktop can read it per tag without running the gateway).

API_VERSION is the API's semver (a breaking change is a major bump) and must equal dimos.yaml's `api.version`.
Regenerate the checked-in file with `python -m dimos.gateway --write-openapi`.
"""

from __future__ import annotations

from importlib.metadata import PackageNotFoundError, version
import json
from pathlib import Path
from typing import TYPE_CHECKING, Any

from fastapi.openapi.utils import get_openapi
from pydantic import TypeAdapter

from dimos.gateway.models import DimosEvent, ErrorResponse

if TYPE_CHECKING:
    from fastapi import FastAPI

API_VERSION = "1.16.0"
SPEC_FILE = Path(__file__).parent / "openapi.json"

DESCRIPTION = """\
The dimos gateway's HTTP API: blueprints, global config, runs and their logs, events, Dimensional cloud uploads,
discovery (blueprints, modules, message types), docs, extras and jobs.

dimOS Desktop starts the gateway (`python -m dimos.gateway`, on a unix socket) and forwards `/dimos/...` to it unchanged.
Every answer is JSON unless the operation says otherwise; every error is `{"error": "<message>"}` (ErrorResponse).
State is HTTP; changes are events, published on zenoh at `<ns>/dimos/events/<type>` (the `events` tag).

Extensions: `x-family: dimos` on every operation; `x-agent: true` marks what Desktop's agent finds through
search_endpoints; `x-mcp-tool` names an MCP tool that does the same; `x-zenoh-key` on an event schema is its key.
"""

TAGS: list[dict[str, Any]] = [
    {
        "name": "server",
        "description": "The gateway itself: liveness, the checkout it serves, where things are, stopping it. Desktop "
        "asks `GET /dimos/paths` whether a running gateway serves the checkout in its config.yaml, and stops one "
        "that serves another.",
    },
    {
        "name": "blueprints",
        "description": "What dimos can run. The list is read in the gateway's process (cached 60 s; `?fresh=1` "
        "refills it). A blueprint's modules, config and the catalog import blueprint code, so they run in a child "
        "process with a 180 s timeout and are cached 10 min: the first call can take seconds.",
    },
    {
        "name": "global-config",
        "description": "dimos's GlobalConfig (robot_ip, simulation, n_workers, ...) and Desktop's saved overrides "
        "(config.yaml `dimos.global_config`), which become `--key value` flags on every launch.",
    },
    {
        "name": "runs",
        "description": "Launching and stopping blueprints. `POST /dimos/runs` runs `dimos [--key value ...] run "
        "<blueprint>` in the checkout, in its own session (it outlives the gateway), output to launch.log. One "
        "launch at a time: a second is refused while the first is starting or running. Its phase (starting, "
        "running, stopped, failed) is worked out from dimos's run registry and the pid on every call. Stopping "
        "sends the process group SIGINT, then SIGTERM after 20 s, then SIGKILL after 10 more.",
    },
    {
        "name": "logs",
        "description": "A run's structured log (`<logs>/<run_id>/main.jsonl`), read off disk. Tail it by passing "
        "the answer's `offset` back as `after`. Warning-or-worse records of the current launch are also `log` "
        "events.",
    },
    {
        "name": "cloud",
        "description": "This machine's Dimensional cloud login: dimos's device flow (`dimos login`). Start it, show "
        "the URL and code, and the user approves it in any signed-in browser; `cloud-login` events (or polling "
        "`GET /dimos/cloud/login`) follow it.",
    },
    {
        "name": "uploads",
        "description": "Uploading recordings to Dimensional cloud with dimos's own code, one at a time, first in "
        "first out. The queue survives a restart (an interrupted upload resumes). With no login an upload goes back "
        "to the front and `waitingForLogin` turns true until a login is approved. `upload`, `uploads` and "
        "`upload-removed` events follow it.",
    },
    {
        "name": "discovery",
        "description": "The discovery cache: every blueprint in dimos's registry imported once (in child processes, a "
        "hang or crash costs one blueprint), whether it imports and why not, its modules and their streams and topics, "
        "every module's config, the message types between them, and modules ranked per robot. It starts with the "
        "server, is saved to disk keyed by the checkout's commit, dirty files and installed packages (a restart "
        "answers at once) and rescans when those change. `discovery` events follow a scan.",
    },
    {
        "name": "docs",
        "description": "dimos's own docs, found in the checkout's docs/ and linked to the published site "
        "(mkdocs.yml site_url).",
    },
    {
        "name": "extras",
        "description": "dimos's optional-dependency extras (pyproject.toml), which are installed in the checkout's "
        "python, and installing more (a job; scripts/install.sh's command, never sudo).",
    },
    {
        "name": "jobs",
        "description": "Long jobs (an extras install), shaped like Desktop's: each output line is published on zenoh "
        'at `<ns>/dimos/jobs/<job>` as `{type: "line", n, line}`, then `{type: "done", ok, error, failure, lines}`; a '
        "`job` event says one started; `GET /dimos/jobs/{job}/log` is the snapshot.",
    },
    {
        "name": "skills",
        "description": "The running blueprint's skills (a module's `@skill` methods), read and called over dimos's "
        "module RPC (`<module>/get_skills`, `<module>/<skill>`), with or without an agent in the blueprint. "
        "`POST /dimos/mcp` offers the same to an agent as two MCP tools that don't change with what runs.",
    },
    {
        "name": "events",
        "description": "The gateway's events, each a JSON object with a `type`: `launch`, `log`, `upload`, "
        "`uploads`, `upload-removed`, `cloud-login`, `discovery`, `job` (schemas: DimosEvent). They are published on zenoh at "
        "`<ns>/dimos/events/<type>` (Desktop's docs/events.md; `<ns>` is Desktop's namespace), which is where to "
        "listen. The SSE stream `GET /dimos/events` carries the same events and is deprecated (kept one release).",
    },
]

ERRORS = {
    400: "Bad request: a missing or malformed body or parameter, or a value the gateway refuses (the message says "
    "which)",
    404: "No such thing (the message names it)",
    409: "Conflict with the current state (the message says what it is)",
    500: "The gateway couldn't do it: a child process, launch, stop or cloud call failed (the message says why)",
    504: "What the gateway waited on (a skill) didn't answer in time",
}


def route_doc(
    tag: str,
    summary: str,
    description: str,
    errors: tuple[int, ...] = (500,),
    agent: bool = False,
    mcp_tool: str | None = None,
    ok: dict[str, Any] | None = None,
    answer: str = "",
) -> dict[str, Any]:
    """FastAPI route kwargs: the docs, error answers and Desktop's extensions; `ok` documents a 200 that isn't JSON."""
    extra: dict[str, Any] = {"x-family": "dimos", "x-agent": agent}
    if mcp_tool:
        extra["x-mcp-tool"] = mcp_tool
    return {
        "tags": [tag],
        "summary": summary,
        "description": description,
        "responses": {
            **({200: ok} if ok else {}),
            **{code: {"model": ErrorResponse, "description": ERRORS[code]} for code in errors},
        },
        "openapi_extra": extra,
        "response_description": answer,
        # an answer has exactly the fields the gateway set: an optional one that's absent stays absent
        "response_model_exclude_unset": True,
    }


def operation_id(route: Any) -> str:
    """Desktop's style: `get_dimos_runs_runId_log`."""
    path = route.path_format.strip("/").replace("{", "").replace("}", "")
    return f"{sorted(route.methods)[0].lower()}_{path.replace('/', '_').replace('-', '_')}"


def dimos_version() -> str | None:
    try:
        return version("dimos")
    except PackageNotFoundError:
        return None


def document(app: FastAPI, runtime: bool = True) -> dict[str, Any]:
    """The OpenAPI document. `runtime`: add the running dimos's version (left out of the checked-in file, so a
    version bump alone doesn't change it)."""
    doc = get_openapi(
        title="dimos gateway",
        version=API_VERSION,
        description=DESCRIPTION,
        routes=app.routes,
        tags=TAGS,
        separate_input_output_schemas=False,
    )
    if runtime:
        doc["info"]["x-dimos-version"] = dimos_version()
    # a bad request is a 400 ErrorResponse here, never FastAPI's 422
    for methods in doc["paths"].values():
        for operation in methods.values():
            responses = operation["responses"]
            responses.pop("422", None)
            for code, response in responses.items():
                content = response.get("content", {})
                if code != "200" and "application/json" not in content:
                    # an error is JSON even where the answer isn't (text, HTML, SSE)
                    response["content"] = {"application/json": next(iter(content.values()))}
                for media in content.values():
                    if "$ref" in media.get("schema", {}):
                        media["schema"].pop("type", None)
    schemas = doc["components"]["schemas"]
    for name in ("HTTPValidationError", "ValidationError"):
        schemas.pop(name, None)
    schemas["JsonValue"] = {"description": "Any JSON value"}
    events = TypeAdapter(DimosEvent).json_schema(ref_template="#/components/schemas/{model}")
    schemas.update(events.pop("$defs", {}))
    schemas["DimosEvent"] = {
        **events,
        "title": "DimosEvent",
        "description": "Any of the gateway's events; `type` says which, and is the last chunk of its zenoh key",
    }
    return doc


def text(doc: dict[str, Any]) -> str:
    """Pretty-printed, keys sorted: stable, and what pre-commit's pretty-format-json leaves alone."""
    return json.dumps(doc, indent=2, sort_keys=True) + "\n"


def spec_app() -> FastAPI:
    """An app to generate the document from (its state is never used)."""
    from dimos.gateway.app import ServerState, create_app
    from dimos.gateway.events import Bus
    from dimos.gateway.uploads import Uploads

    bus = Bus()
    nowhere = Path("/nonexistent")
    return create_app(
        ServerState(nowhere, bus, Uploads(nowhere, bus, None, nowhere / "log")), background=False
    )


def write(path: Path = SPEC_FILE) -> Path:
    path.write_text(text(document(spec_app(), runtime=False)))
    return path
