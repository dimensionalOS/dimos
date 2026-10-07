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

"""The dimos gateway's HTTP API, `/dimos/...`: blueprints, global config, runs and their logs, events, cloud uploads.

dimOS Desktop proxies `/dimos/` to it. Each route's docs, request and answer models (models.py) make its OpenAPI
document (openapi.py), served at /dimos/openapi.json and checked in as openapi.json. fixtures/ holds Desktop's own
document of these paths; test_contract.py and test_openapi.py check this gateway matches it. An error is
`{"error": "<message>"}`.
"""

from __future__ import annotations

import asyncio
from collections.abc import AsyncIterator
from contextlib import asynccontextmanager
from dataclasses import dataclass, field
import os
from pathlib import Path
import re
import sys
import time
from typing import Annotated, Any, Literal

from fastapi import FastAPI, Path as PathParam, Query, Request
from fastapi.exceptions import RequestValidationError
from fastapi.responses import (
    HTMLResponse,
    JSONResponse,
    PlainTextResponse,
    Response,
    StreamingResponse,
)
from pydantic import BeforeValidator
from starlette.exceptions import HTTPException as StarletteHTTPException

from dimos.gateway import (
    blueprints,
    config,
    discovery_routes,
    events,
    logs,
    models,
    overrides as launch_overrides,
    runs,
    skills_routes,
)
from dimos.gateway.blueprint_watch import BlueprintWatch
from dimos.gateway.discovery import Discovery
from dimos.gateway.jobs import Jobs
from dimos.gateway.msgs import routes as msgs_routes
from dimos.gateway.openapi import document, operation_id, route_doc
from dimos.gateway.topic_rates import TopicWatch
from dimos.gateway.uploads import Uploads

LIST_TTL_S = 60.0
# the blueprint view's page and files (GET /dimos/blueprint_view): only these names are served from its folder
VIEW_DIR = Path(__file__).parent / "blueprint_view"
VIEW_FILE = re.compile(r"[a-z_]+\.(js|css)")
INTROSPECT_TTL_S = 600.0


def _started_from() -> tuple[str | None, int | None]:
    """The program this gateway runs (the python running it) and its modification time (Unix s), as it started."""
    try:
        return sys.executable, int(os.stat(sys.executable).st_mtime)
    except (OSError, ValueError):
        return sys.executable or None, None


STARTED_FROM = _started_from()
STARTED_AT = int(time.time())


class EventStreamResponse(StreamingResponse):
    media_type = "text/event-stream"


# `?fresh`, `?fresh=1`, `?fresh=true`: skip the cache
FreshQuery = Annotated[
    bool,
    BeforeValidator(lambda value: True if value == "" else value),
    Query(description="check again now instead of answering from the cache"),
]
BlueprintParam = Annotated[
    str,
    PathParam(description="blueprint name, e.g. unitree-go2-basic", examples=["unitree-go2-basic"]),
]
UploadIdParam = Annotated[str, PathParam(description="upload id, e.g. u3", examples=["u3"])]


class ApiError(Exception):
    def __init__(self, status: int, message: str) -> None:
        super().__init__(message)
        self.status = status


@dataclass
class ServerState:
    dimos_dir: Path
    bus: events.Bus
    uploads: Uploads
    cache: blueprints.Cache = field(default_factory=blueprints.Cache)
    # what the gateway runs in the background (the launch watcher, the upload queue)
    background: list[asyncio.Task[None]] = field(default_factory=list)
    # set by serve(): makes the process exit
    exit: Any = None
    # the discovery cache and background jobs (create_app makes them when not given)
    discovery: Discovery | None = None
    jobs: Jobs | None = None
    # the zenoh namespace the bus publishes under (serve() sets it once its publisher is open), None = SSE only
    zenoh_namespace: str | None = None
    # every topic heard on the bus (serve() starts it), None = not listening
    topics: TopicWatch | None = None
    # re-lists the blueprints when dimos/robot or site-packages change (create_app makes it), None = no watching
    watch: BlueprintWatch | None = None


def default_state(dimos_dir: Path) -> ServerState:
    bus = events.Bus()
    # uploaded.json is read from beside uploads.json
    config.state_file("uploaded.json")
    uploads = Uploads(
        dimos_dir, bus, config.state_file("uploads.json"), config.logs_dir() / "uploads.log"
    )
    return ServerState(dimos_dir=dimos_dir, bus=bus, uploads=uploads)


def create_app(state: ServerState, background: bool = True) -> FastAPI:
    discovered = state.discovery = state.discovery or Discovery(
        state.dimos_dir, lambda event: state.bus.send(event)
    )
    state.jobs = state.jobs or Jobs(
        lambda event: state.bus.send(event), lambda key, payload: state.bus.publish(key, payload)
    )

    async def list_in_child() -> list[dict[str, Any]]:
        answer = await discovered.child_answer("list")
        if "error" in answer:
            raise RuntimeError(answer["error"])
        return list(answer["blueprints"])

    def blueprints_changed(_: list[dict[str, Any]], added: list[str], removed: list[str]) -> None:
        state.cache.forget("list")
        state.bus.send({"type": "blueprints", "added": added, "removed": removed})

    watch = state.watch = state.watch or BlueprintWatch(
        state.dimos_dir,
        list_in_child,
        blueprints_changed,
        lambda: discovered.refresh("files changed"),
    )

    @asynccontextmanager
    async def lifespan(_: FastAPI) -> AsyncIterator[None]:
        if background:
            state.background += [
                asyncio.create_task(events.watch_launch(state.bus)),
                asyncio.create_task(state.uploads.work()),
                asyncio.create_task(discovered.run()),
                asyncio.create_task(watch.run()),
            ]
        yield
        state.uploads.shutdown()
        for task in state.background:
            task.cancel()

    # no Swagger UI: it loads its scripts from a CDN (a robot is often offline) and Desktop already browses and
    # searches the API; /dimos/openapi.json is the one source
    app = FastAPI(
        title="dimos gateway",
        lifespan=lifespan,
        openapi_url="/dimos/openapi.json",
        docs_url=None,
        redoc_url=None,
        generate_unique_id_function=operation_id,
        separate_input_output_schemas=False,
    )
    app.state.server = state
    s = state

    @app.exception_handler(ApiError)
    async def api_error(_: Request, error: ApiError) -> JSONResponse:
        return JSONResponse({"error": str(error)}, status_code=error.status)

    @app.exception_handler(StarletteHTTPException)
    async def http_error(request: Request, error: StarletteHTTPException) -> JSONResponse:
        message = (
            f"no such route: {request.url.path}" if error.status_code == 404 else str(error.detail)
        )
        return JSONResponse({"error": message}, status_code=error.status_code)

    @app.exception_handler(RequestValidationError)
    async def bad_request(_: Request, error: RequestValidationError) -> JSONResponse:
        problems = "; ".join(f"{'.'.join(map(str, e['loc']))}: {e['msg']}" for e in error.errors())
        return JSONResponse({"error": f"bad request: {problems}"}, status_code=400)

    @app.exception_handler(Exception)
    async def failed(_: Request, error: Exception) -> JSONResponse:
        return JSONResponse({"error": str(error) or type(error).__name__}, status_code=500)

    async def introspected(key: str, args: list[str]) -> Any:
        try:
            return await s.cache.get(
                key, INTROSPECT_TTL_S, lambda: blueprints.introspect(s.dimos_dir, args)
            )
        except blueprints.IntrospectError as error:
            raise ApiError(500, str(error))

    def checked_overrides(overrides: dict[str, Any]) -> None:
        try:
            config.check_overrides(overrides)
        except ValueError as error:
            raise ApiError(400, str(error))

    def check_name(name: str) -> None:
        if not blueprints.valid_name(name):
            raise ApiError(400, f"bad blueprint name: {name}")

    @app.get("/healthz", response_class=PlainTextResponse, include_in_schema=False)
    @app.get(
        "/dimos/healthz",
        response_class=PlainTextResponse,
        **route_doc(
            "server",
            "The dimos gateway's liveness: `ok`",
            "Answers `ok` (text/plain) while the gateway runs; no side effects. Desktop polls it after starting the "
            "server.",
            errors=(),
            ok={"content": {"text/plain": {"schema": {"type": "string", "example": "ok"}}}},
            answer="`ok`",
        ),
    )
    async def healthz() -> str:
        return "ok"

    @app.get(
        "/dimos/info",
        response_model=models.Info,
        **route_doc(
            "server",
            "The dimos checkout this gateway drives: dir, version, installed",
            "Reads the checkout's pyproject.toml and whether its `.venv/bin/dimos` exists, and checks the version "
            "against the range Desktop supports ($DESKTOP_DIMOS_RANGE). A launch is refused while `inRange` is "
            "false. No side effects.",
            agent=True,
            answer="`{ dir, found, installed, version, range, inRange }`",
        ),
    )
    async def info() -> dict[str, Any]:
        return config.info(s.dimos_dir).to_json()

    @app.get(
        "/dimos/topics/rates",
        response_model=models.TopicRates,
        **route_doc(
            "server",
            "Every topic on the bus the gateway has heard since it started: rate, throughput, messages, last heard",
            "The gateway listens to zenoh `dimos/**` from its start, so a topic published once (at a blueprint's "
            "startup) is listed too, and a topic stays listed after it goes quiet (0 Hz, `lastSeen` seconds ago). "
            "Rates are over the last 2 s. Not listed: RPC calls (zenoh queries) and LCM-only traffic. `up` is false "
            "(with `error`) when the gateway has no zenoh session. No side effects.",
            answer="`{ up, error, topics: [{ topic, type, hz, bps, messages, lastSeen, declared }] }`",
        ),
    )
    async def topic_rates() -> dict[str, Any]:
        if s.topics is None:
            return {"up": False, "error": "this gateway isn't listening to the bus", "topics": []}
        return await asyncio.to_thread(s.topics.snapshot)

    @app.get(
        "/dimos/paths",
        response_model=models.Paths,
        **route_doc(
            "server",
            "Where dimos keeps its things: the checkout, run registry, log folders, recordings folder",
            "Absolute paths, read from the environment and Desktop's config.yaml. Desktop compares `dimosDir` with "
            "its configured checkout to tell whether this gateway is the right one, and `server` with its own binary "
            "to tell whether its built-in gateway is outdated (this gateway runs python, so it never is). No side "
            "effects.",
            answer="`{ dimosDir, runsDir, logsDirs, recordingsDir, server: { exe, exeModified, kind, startedAt, zenohNamespace } }`",
        ),
    )
    async def paths() -> dict[str, Any]:
        from dimos.core.run_registry import REGISTRY_DIR

        return {
            "dimosDir": str(s.dimos_dir),
            "runsDir": str(REGISTRY_DIR),
            "logsDirs": [str(d) for d in logs.logs_dirs(s.dimos_dir)],
            "recordingsDir": str(config.recordings_dir()),
            "server": {
                "exe": STARTED_FROM[0],
                "exeModified": STARTED_FROM[1],
                "kind": "dimos",
                "startedAt": STARTED_AT,
                "zenohNamespace": s.zenoh_namespace,
            },
        }

    @app.post(
        "/dimos/server/stop",
        response_model=models.Stopping,
        **route_doc(
            "server",
            "Make the dimos gateway exit (Desktop starts it again when needed)",
            "Answers, then exits 0.2 s later. A running upload's worker is killed and the upload is queued again "
            "for the next start (dimos resumes it); launched blueprints keep running (their own sessions).",
            answer="`{ stopping: true }`",
        ),
    )
    async def stop_server() -> dict[str, Any]:
        s.uploads.shutdown()
        loop = asyncio.get_running_loop()
        loop.call_later(0.2, s.exit or (lambda: os._exit(0)))
        return {"stopping": True}

    @app.get(
        "/dimos/blueprints",
        response_model=models.BlueprintList,
        **route_doc(
            "blueprints",
            "Every blueprint dimos can run (name, builtin/external) and whether it imports",
            "What `dimos list` prints: built-in blueprints (without demo-*), then external ones from installed "
            "packages. Cached for 60 s and re-listed (in a child) whenever dimos/robot or site-packages change, "
            "with a `blueprints` event; `fresh` re-lists first. No other side effects. `importable`, "
            "`import_error` and `missing_module` come from the discovery cache (null until the scan reaches the "
            "blueprint: GET /dimos/discovery).",
            errors=(400, 500),
            agent=True,
            answer='`{ blueprints: [{ name, kind: "builtin"|"external", importable, import_error, missing_module }] }`',
        ),
    )
    async def blueprint_list(fresh: FreshQuery = False) -> dict[str, Any]:
        if fresh:
            s.cache.forget("list")
            if s.watch and s.watch.listed is not None:
                await s.watch.relist()

        async def compute() -> dict[str, Any]:
            # the watcher's listing comes from a child, so it follows edits to the registry; this process's own
            # import of it can't
            listed = s.watch.listed if s.watch else None
            return {"blueprints": listed or await asyncio.to_thread(blueprints.blueprint_list)}

        result: dict[str, Any] = await s.cache.get("list", LIST_TTL_S, compute)
        return {"blueprints": [discovered.import_status(entry) for entry in result["blueprints"]]}

    @app.get(
        "/dimos/blueprints/{name}",
        response_model=models.Blueprint,
        **route_doc(
            "blueprints",
            "A blueprint's modules and each module's streams (topics, types, in/out), docstring, RPC methods, "
            "skills and source location",
            "Imports the blueprint in a child process (180 s timeout; cached 10 min) and lists its modules with "
            "their streams' names, message types, directions and wired topics, each module's docstring (whole, and "
            "its first paragraph as `summary`), RPC methods and skills (signature and docstring) and where its class "
            "is defined (`file`, `line`; GET /dimos/source reads the file). 400 for a name that can't be one, 500 "
            "when the blueprint can't be found or imported.",
            errors=(400, 500),
            agent=True,
            answer="`{ name, modules: [{ name, class, doc, summary, file, line, rpcs, skills, streams: [{ name, "
            "type, direction, topic }] }] }`",
        ),
    )
    async def blueprint(name: BlueprintParam) -> Any:
        check_name(name)
        return await introspected(f"bp:{name}", ["blueprint", name])

    @app.get(
        "/dimos/source",
        response_model=models.SourceFile,
        **route_doc(
            "blueprints",
            "A Python file of the dimos checkout, as text (a module's code)",
            "`file` is a path relative to the checkout (as GET /dimos/blueprints/{name} gives a module's `file`) or "
            "an absolute one inside it. Only .py files inside the checkout are read: 400 for anything else, 404 "
            "when there's no such file.",
            errors=(400, 404),
            answer="`{ file, text }`",
        ),
    )
    async def source(
        file: Annotated[
            str, Query(description="The file: relative to the checkout, or absolute inside it")
        ],
    ) -> Any:
        root = s.dimos_dir.resolve()
        path = (root / file).resolve()
        if path.suffix != ".py" or not path.is_relative_to(root):
            raise ApiError(400, f"not a .py file in the dimos checkout: {file}")
        if not path.is_file():
            raise ApiError(404, f"no such file: {file}")
        return {"file": file, "text": await asyncio.to_thread(path.read_text, "utf-8", "replace")}

    @app.get(
        "/dimos/blueprint_view",
        response_class=HTMLResponse,
        **route_doc(
            "blueprints",
            "A page showing a blueprint: its modules (rarest first) beside its module graph, each module's streams, "
            "skills, RPC methods and code",
            "An HTML page (plain JS and CSS, its files at /dimos/blueprint_view/{file}): dimOS Desktop's whole "
            "blueprint Details modal. A top bar (the blueprint and its phase from GET /dimos/runs; Relaunch: POST "
            "/dimos/runs/restart; Stop: POST /dimos/runs/stop; Configure: GET/PUT /dimos/blueprints/{name}/config and "
            "/dimos/global-config, recommended settings from GET /dimos/robots; Show code; Logs: GET "
            "/dimos/runs/{runId}/log), a side panel (Topic rates from GET /dimos/topics/rates; the modules, rarest "
            "first by GET /dimos/catalog) and the module graph drawn from the blueprint's wiring, live rates on its "
            "topics while it runs. It styles itself with Desktop's /theme.css and skin (Portal off Desktop). In an "
            'iframe it posts to its parent, on its own origin: {type:"dimos:chrome"} (it can draw the top bar; a '
            'parent that then shows only the page answers {type:"dimos:chrome-ok"}, and only then does the bar show), '
            '{type:"dimos:open-in-editor", file, line} (the parent answers {type:"dimos:open-in-editor-result", ok, '
            'text}) and {type:"dimos:close"} (its close button, or Escape). 404 for a blueprint dimos doesn\'t list. '
            "The page itself has no side effects; its buttons do what the routes they call say.",
            errors=(400, 404),
            ok={"content": {"text/html": {"schema": {"type": "string"}}}},
            answer="an HTML page",
        ),
    )
    async def blueprint_view(
        name: Annotated[
            str,
            Query(
                description="blueprint name, e.g. unitree-go2-basic", examples=["unitree-go2-basic"]
            ),
        ],
    ) -> str:
        check_name(name)
        listed = await blueprint_list()
        if not any(entry["name"] == name for entry in listed["blueprints"]):
            raise ApiError(404, f"no such blueprint: {name}")
        return await asyncio.to_thread((VIEW_DIR / "index.html").read_text, "utf-8")

    @app.get(
        "/dimos/blueprint_view/{file}",
        response_class=Response,
        **route_doc(
            "blueprints",
            "One of the blueprint view page's own files (its scripts and styles)",
            "Serves app.js, graph.js, layout.js, view.css or portal.css from dimos/gateway/blueprint_view/: only a "
            "plain .js or .css name in that folder (404 for anything else). No side effects.",
            errors=(404,),
            ok={
                "content": {
                    "text/javascript": {"schema": {"type": "string"}},
                    "text/css": {"schema": {"type": "string"}},
                }
            },
            answer="the file",
        ),
    )
    async def blueprint_view_file(
        file: Annotated[
            str, PathParam(description="the file's name, e.g. app.js", examples=["app.js"])
        ],
    ) -> Response:
        path = VIEW_DIR / file
        if not VIEW_FILE.fullmatch(file) or not path.is_file():
            raise ApiError(404, f"no such file: {file}")
        media = "text/css" if file.endswith(".css") else "text/javascript"
        return Response(
            await asyncio.to_thread(path.read_bytes),
            media_type=f"{media}; charset=utf-8",
            headers={"cache-control": "no-cache"},
        )

    @app.get(
        "/dimos/blueprints/{name}/config",
        response_model=models.BlueprintConfig,
        **route_doc(
            "blueprints",
            "A blueprint's configurable args per module: name, type, default, description, and the value the "
            "blueprint sets (a module that can't be read carries its own error)",
            "Imports the blueprint in a child process (180 s timeout; cached 10 min) and reads each module's "
            "pydantic `config` model: every field a person can set, its type, default, description, whether it's "
            "required or inherited from ModuleConfig, its choices (Enum/Literal) and the blueprint's value. A module "
            "whose config can't be read has `error` instead of failing the whole answer.",
            errors=(400, 500),
            agent=True,
            answer="`{ name, modules: [{ module, class, args: [{ name, type, default, description, required, base, choices?, value? }], error? }] }`",
        ),
    )
    async def blueprint_config(name: BlueprintParam) -> Any:
        check_name(name)
        value = await introspected(f"config:{name}", ["config", name])
        return blueprints.shown_config(name, value)

    @app.put(
        "/dimos/blueprints/{name}/config",
        response_model=models.BlueprintConfig,
        **route_doc(
            "blueprints",
            "Save Desktop's module config for a blueprint; it becomes `--<module>.<field>=value` on every launch of it",
            "Replaces config.yaml's `dimos.module_config.<name>` with `overrides` ({module: {field: value}}; null "
            "drops a field, a module left empty is dropped, all empty removes the blueprint's entry), after checking "
            "each module and field against the blueprint's config (as a launch's `overrides.modules`). A secret sent "
            "as ••• keeps its saved value. 400 for a bad name or a value its field refuses. Answers like GET.",
            errors=(400, 500),
            answer="`{ name, modules: [...], overrides }`, with the saved module config",
        ),
    )
    async def put_blueprint_config(
        name: BlueprintParam, update: models.BlueprintConfigUpdate
    ) -> Any:
        check_name(name)
        value = await introspected(f"config:{name}", ["config", name])
        saved = config.module_config(name)
        values = {
            module: launch_overrides.keep_hidden(
                fields, saved.get(module, {}), launch_overrides.is_secret_name
            )
            for module, fields in update.overrides.items()
        }
        try:
            launch_overrides.validate_modules(values, value, "overrides")
        except ValueError as error:
            raise ApiError(400, str(error))
        config.set_module_config(name, launch_overrides.merge_modules({}, values))
        return blueprints.shown_config(name, value)

    @app.get(
        "/dimos/catalog",
        response_model=models.Catalog,
        **route_doc(
            "blueprints",
            "Every blueprint, module and skill (imports them all, in a child process: slow the first time)",
            "Imports every built-in blueprint and module in a child process (180 s timeout; cached 10 min) and "
            "lists blueprints with their robot and modules, modules with their streams and skills, and every skill "
            "with its parameters. What fails to import is listed in `errors`; the rest still answers.",
            answer="`{ blueprints: [{ name, ref, robot, modules }], modules: [{ name, class, doc, robots, inputs, outputs, skills }], skills: [{ name, doc, params, module, robots }], errors }`",
        ),
    )
    async def catalog() -> Any:
        return await introspected("catalog", ["catalog"])

    @app.get(
        "/dimos/robots",
        response_model=models.Robots,
        **route_doc(
            "blueprints",
            "Every robot dimos supports and its blueprints: title, description, the settings to decide before "
            "running each (recommended_config), starter picks, recommended blueprints, hidden ones",
            "The checkout's dimos/gateway/robots.json (dimos.yaml's `robots:`, also readable per tag without a "
            "server) with its defaults applied: each blueprint's recommended_config, every setting resolved: one "
            "config value (`key`, `scope`: global keys are `--key value` before `run`, module keys after the "
            "blueprint name) or a pick (`kind` pick: each choice `set`s several values, e.g. Robot / Replay / "
            "Simulator), either with `choices` (an enum) and `when` (shown only while those values hold); robots' "
            "`type` is a key of `types`, the order a launcher groups them in. `registered` and `unlisted` compare it "
            "with the blueprint registry. CI keeps the file in step with the code (dimos/gateway/robots.py). Read "
            "from disk on every call; no side effects.",
            agent=True,
            answer="`{ about, types, robots: { [id]: { name, description, type, manufacturer, dirs, recommended, blueprints: { [name]: { title, description, starter, hidden, recommended_config, robot, registered } } } }, excluded, unlisted }`",
        ),
    )
    async def robot_list() -> dict[str, Any]:
        from dimos.gateway import robots

        def compute() -> dict[str, Any]:
            from dimos.robot.all_blueprints import all_blueprints

            path = s.dimos_dir / "dimos" / "gateway" / "robots.json"
            return robots.resolved(
                robots.load(path if path.exists() else robots.ROBOTS_FILE), all_blueprints
            )

        return await asyncio.to_thread(compute)

    async def global_config_value() -> dict[str, Any]:
        async def compute() -> dict[str, Any]:
            return await asyncio.to_thread(blueprints.global_config_schema)

        value: dict[str, Any] = await s.cache.get("gc", INTROSPECT_TTL_S, compute)
        secrets = [
            key
            for key in value["schema"].get("properties", {})
            if launch_overrides.is_secret_name(key)
        ]
        shown, _ = launch_overrides.redact(config.global_config_overrides(), {}, secrets)
        defaults = {**value["defaults"], **config.LAUNCH_GLOBAL_DEFAULTS}
        return {**value, "defaults": defaults, "overrides": shown, "secrets": secrets}

    @app.get(
        "/dimos/global-config",
        response_model=models.GlobalConfig,
        **route_doc(
            "global-config",
            "dimos GlobalConfig: JSON schema, defaults, Desktop's overrides",
            "GlobalConfig's JSON Schema and defaults (cached 10 min) and Desktop's saved overrides from config.yaml "
            "`dimos.global_config`. No side effects.",
            agent=True,
            answer="`{ schema, defaults, overrides }`",
        ),
    )
    async def global_config() -> dict[str, Any]:
        return await global_config_value()

    @app.put(
        "/dimos/global-config",
        response_model=models.GlobalConfig,
        **route_doc(
            "global-config",
            "Save Desktop's GlobalConfig overrides (null removes one); they become `--key=value` on every launch",
            "Replaces config.yaml's `dimos.global_config` with `overrides` (null values dropped), keeping the rest "
            "of the file. Takes effect at the next launch; a running blueprint is untouched. 400 for a key that "
            "isn't a GlobalConfig field `dimos` takes as a flag, or a value GlobalConfig refuses. Answers like GET.",
            errors=(400, 500),
            answer="`{ schema, defaults, overrides }`, with the saved overrides",
        ),
    )
    async def put_global_config(update: models.GlobalConfigUpdate) -> dict[str, Any]:
        for key in update.overrides:
            if not re.fullmatch(r"[A-Za-z0-9_]+", key):
                raise ApiError(400, f"bad config key: {key}")
        # ••• for a secret: keep the saved value
        values = launch_overrides.keep_hidden(
            update.overrides, config.global_config_overrides(), launch_overrides.is_secret_name
        )
        schema = (await global_config_value())["schema"]
        try:
            launch_overrides.validate_global(values, schema, "overrides")
        except ValueError as error:
            raise ApiError(400, str(error))
        checked_overrides(values)
        config.set_global_config_overrides(values)
        return await global_config_value()

    @app.get(
        "/dimos/runs",
        response_model=models.RunList,
        **route_doc(
            "runs",
            "Running blueprints (run id, blueprint, pid, log_dir) and the launch this gateway started",
            "Live runs from dimos's run registry (pid alive; also runs started from a terminal), newest first, and "
            "this gateway's last launch with its phase. No side effects.",
            agent=True,
            answer="`{ runs: [{ run_id, pid, blueprint, started_at, log_dir }], launch: Launch | null }`",
        ),
    )
    async def run_list() -> dict[str, Any]:
        return {
            "runs": await asyncio.to_thread(runs.registry_runs),
            "launch": await asyncio.to_thread(runs.current_launch),
        }

    async def launch_config_of(request: models.LaunchRequest) -> runs.LaunchConfig:
        """Desktop's saved global config and this blueprint's saved module config, the request's own values on top
        (checked against dimos's schemas first; null drops a saved value, ••• keeps it), plus `replay`."""
        try:
            one_off = launch_overrides.parse(request.overrides)
        except ValueError as error:
            raise ApiError(400, str(error))
        one_off.global_ = {k: v for k, v in one_off.global_.items() if v != launch_overrides.HIDDEN}
        one_off.modules = {
            m: {k: v for k, v in f.items() if v != launch_overrides.HIDDEN}
            for m, f in one_off.modules.items()
        }
        if request.replay:
            one_off.global_["replay"] = True
        try:
            if one_off.global_:
                schema = (await global_config_value())["schema"]
                launch_overrides.validate_global(
                    one_off.global_, schema, "overrides.global", one_off.secrets
                )
            if one_off.modules:
                value = await introspected(
                    f"config:{request.blueprint}", ["config", request.blueprint]
                )
                launch_overrides.validate_modules(
                    one_off.modules, value, "overrides.modules", one_off.secrets
                )
        except ValueError as error:
            raise ApiError(400, str(error))
        return with_saved(request.blueprint, one_off)

    def with_saved(blueprint: str, one_off: launch_overrides.LaunchOverrides) -> runs.LaunchConfig:
        """A launch's own values on top of Desktop's saved global config and the blueprint's saved module config, as
        saved now."""
        effective = launch_overrides.merge(
            launch_overrides.merge(config.LAUNCH_GLOBAL_DEFAULTS, config.global_config_overrides()),
            one_off.global_,
        )
        checked_overrides(effective)
        return runs.LaunchConfig(
            effective,
            launch_overrides.merge_modules(config.module_config(blueprint), one_off.modules),
            one_off,
        )

    @app.post(
        "/dimos/runs",
        response_model=models.Launch,
        **route_doc(
            "runs",
            "Launch a blueprint (stops nothing; check /dimos/runs first)",
            "Starts `dimos [--key value ...] run <blueprint>` in the checkout, in its own session, with Desktop's "
            "saved GlobalConfig overrides, then the body's, then `--replay` if asked. Answers at once with phase "
            "`starting`; `launch` events (or GET /dimos/runs) follow it to running, stopped or failed. 400 when the "
            "checkout's dimos is outside Desktop's range (unless config.yaml `dimos.ignore_version_range`), the "
            "name is bad, an override (saved or given) isn't a GlobalConfig flag or valid value, and while the last launch is still starting or running (one at a time); 500 when dimos isn't "
            "installed or won't start.",
            errors=(400, 500),
            agent=True,
            mcp_tool="run_blueprint",
            answer="`Launch`: `{ blueprint, phase, startedAt, pid, output, runId, logDir, error, overrides, steps: [{ "
            "label, state: done|now|todo|failed, detail }], problems: [{ level, text, fix, line }] }`",
        ),
    )
    async def launch(request: models.LaunchRequest) -> dict[str, Any]:
        checkout = config.info(s.dimos_dir)
        if checkout.installed and not checkout.in_range and not config.ignore_version_range():
            raise ApiError(
                400,
                f"dimos {checkout.version or '?'} is outside the range Desktop supports ({checkout.range}); "
                "set dimos.ignore_version_range to launch anyway",
            )
        if not request.blueprint or request.blueprint.startswith("-"):
            raise ApiError(400, "bad blueprint name")
        launch_config = await launch_config_of(request)
        try:
            started = runs.start(s.dimos_dir, request.blueprint, launch_config)
        except runs.StillRunningError as error:
            raise ApiError(400, str(error))
        except runs.RunError as error:
            raise ApiError(500, str(error))
        s.bus.launch(started)
        return started

    @app.post(
        "/dimos/runs/restart",
        response_model=models.Launch,
        **route_doc(
            "runs",
            "Stop the blueprint this gateway launched (if it still runs) and launch it again: its own values on top "
            "of the config saved now",
            "Takes the last launch's blueprint and its own (one-off) overrides (kept even after it stopped), stops it "
            "first if it's starting or running (as POST /dimos/runs/stop), then launches it as POST /dimos/runs "
            "would, with Desktop's saved global and module config as saved now (a config change since applies). "
            "Takes no body. 400 when nothing was launched yet; 500 when it won't stop or won't start.",
            errors=(400, 500),
            agent=True,
            answer="`Launch` (as POST /dimos/runs)",
        ),
    )
    async def restart() -> dict[str, Any]:
        last = runs.last_launch_args()
        if last is None:
            raise ApiError(400, "the dimos gateway hasn't launched anything yet")
        blueprint, last_config = last
        launch_config = with_saved(blueprint, last_config.one_off)
        current = await asyncio.to_thread(runs.current_launch)
        try:
            if current and current["phase"] in ("starting", "running", "stopping"):
                await runs.stop(None, lambda: s.bus.launch(runs.current_launch()))
            started = runs.start(s.dimos_dir, blueprint, launch_config)
        except runs.RunError as error:
            raise ApiError(500, str(error))
        s.bus.launch(started)
        return started

    @app.post(
        "/dimos/runs/stop",
        response_model=models.StopResult,
        **route_doc(
            "runs",
            "Stop the blueprint this gateway launched (or runId)",
            "Sends the run's process group SIGINT, then SIGTERM after 20 s, then SIGKILL after 10 more, and answers "
            "once it's gone. Stops `runId` (any live run in the registry) or else this gateway's launch; the body is "
            "optional. 500 when there's nothing running to stop or it won't stop.",
            errors=(400, 500),
            agent=True,
            mcp_tool="stop_blueprint",
            answer="`{ output }`",
        ),
    )
    async def stop(request: models.StopRequest | None = None) -> dict[str, Any]:
        try:
            # `stopping` goes out before the first signal (runs.stop marks the launch), `stopped` once it's gone
            return {
                "output": await runs.stop(
                    request.runId if request else None,
                    lambda: s.bus.launch(runs.current_launch()),
                )
            }
        except runs.RunError as error:
            raise ApiError(500, str(error))
        finally:
            s.bus.launch(await asyncio.to_thread(runs.current_launch))

    @app.get(
        "/dimos/runs/{runId}/log",
        response_model=models.LogPage,
        **route_doc(
            "logs",
            "A run's structured log (main.jsonl): records with level, logger, event",
            "Reads `<logs>/<runId>/main.jsonl` off disk (the checkout's logs/, then the library install's; the "
            "last 4 MB at most). Without `after`: the last `limit` matching records; with `after`: every matching "
            "record past that byte offset (tailing). An unknown run answers no records. No side effects.",
            errors=(400, 500),
            agent=True,
            answer="`{ runId, records: [{ timestamp, level, logger, event, extra, raw }], offset, loggers }`",
        ),
    )
    async def log(
        run_id: Annotated[
            str,
            PathParam(
                alias="runId",
                description="run id or latest",
                examples=["latest", "20260101-120000-unitree-go2"],
            ),
        ],
        after: Annotated[
            int | None,
            Query(description="byte offset from an earlier answer's `offset`: only newer records"),
        ] = None,
        level: Annotated[
            str | None,
            Query(description="minimum level: debug, info, warning, error", examples=["warning"]),
        ] = None,
        q: Annotated[str | None, Query(description="text to match")] = None,
        limit: Annotated[
            int | None,
            Query(description="at most this many records (default 1000; ignored with `after`)"),
        ] = None,
    ) -> dict[str, Any]:
        filter = logs.Filter(query=q or None, min_level=level or None)
        return await asyncio.to_thread(logs.read, s.dimos_dir, run_id, after, limit or 1000, filter)

    @app.get(
        "/dimos/events",
        deprecated=True,
        response_class=EventStreamResponse,
        **route_doc(
            "events",
            "The dimos gateway's live events (SSE): launch phases, warning+ log records, uploads, the cloud login",
            "Deprecated and internal, kept for one release: the same events are published on zenoh at "
            "`<ns>/dimos/events/<type>` (Desktop's docs/events.md), so listen there. The stream starts with the "
            "current `launch` event, then sends every event (a client that can't keep up loses events). Each event "
            "type's payload is a component schema (LaunchEvent, LogEvent, UploadEvent, UploadsEvent, "
            "UploadRemovedEvent, CloudLoginEvent; DimosEvent is any of them).",
            ok={
                "description": "Server-sent events, one `data: <DimosEvent JSON>` each; `:` keep-alives every 15 s",
                "content": {
                    "text/event-stream": {"schema": {"$ref": "#/components/schemas/DimosEvent"}}
                },
            },
        ),
    )
    async def event_stream() -> StreamingResponse:
        stream = s.bus.stream(lambda: {"type": "launch", "launch": runs.current_launch()})
        return EventStreamResponse(
            stream, headers={"Cache-Control": "no-cache", "Deprecation": "true"}
        )

    @app.get(
        "/dimos/cloud/account",
        response_model=models.Account,
        **route_doc(
            "cloud",
            "Whether this machine is logged in to Dimensional cloud, and as whom",
            "Asks Dimensional cloud who the stored key (or DIMOS_API_KEY) belongs to, in a child process (90 s "
            "timeout). Cached for 20 s; `fresh` asks again. A logged-in answer restarts an upload queue that was "
            "waiting for a login.",
            errors=(400, 500),
            agent=True,
            answer="`{ loggedIn, email, scopes, source, cloudUrl, error }`",
        ),
    )
    async def cloud_account(fresh: FreshQuery = False) -> dict[str, Any]:
        return await s.uploads.account(fresh)

    @app.get(
        "/dimos/cloud/login",
        response_model=models.Login,
        **route_doc(
            "cloud",
            "The cloud login in progress: state (idle, starting, pending, approved, denied, expired, failed), url, "
            "code",
            "The device login's state; a pending one past its expiry turns expired. No other side effects.",
            agent=True,
            answer="`Login`: `{ state, url, urlComplete, code, expiresAt, email, error }`",
        ),
    )
    async def cloud_login() -> dict[str, Any]:
        return s.uploads.login_state()

    @app.post(
        "/dimos/cloud/login",
        response_model=models.Login,
        **route_doc(
            "cloud",
            "Start logging this machine in to Dimensional cloud: returns a URL and a code the user approves in any "
            "signed-in browser",
            "Starts dimos's device login in a child process (or returns the one already waiting) and answers once "
            "the code is known (pending), within 30 s. Show the URL and code; `cloud-login` events (or GET "
            "/dimos/cloud/login) follow it. Once approved, dimos stores the key and a waiting upload queue goes on.",
            agent=True,
            answer="`Login`",
        ),
    )
    async def start_cloud_login() -> dict[str, Any]:
        return await s.uploads.start_login()

    @app.delete(
        "/dimos/cloud/login",
        response_model=models.Login,
        **route_doc(
            "cloud",
            "Cancel the pending cloud login",
            "Kills the login's child process; a starting or pending login goes back to idle. Answers the login "
            "state.",
            answer="`Login`",
        ),
    )
    async def cancel_cloud_login() -> dict[str, Any]:
        return s.uploads.cancel_login()

    @app.get(
        "/dimos/cloud/login/page",
        response_class=HTMLResponse,
        **route_doc(
            "cloud",
            'A small HTML page for an app\'s iframe that runs the cloud login and posts {type:"dimos-cloud-login", '
            "state, email} to its parent",
            "A page an app embeds (the console itself refuses to be framed): it starts the login, shows the URL and "
            "code, and posts the outcome to its parent window. No side effects until it's opened.",
            errors=(400,),
            ok={"content": {"text/html": {"schema": {"type": "string"}}}},
            answer="an HTML page",
        ),
    )
    async def cloud_login_page(
        theme: Annotated[
            Literal["light", "dark"] | None,
            Query(description="light or dark (default: the system's)"),
        ] = None,
    ) -> str:
        return (Path(__file__).parent / "login_page.html").read_text()

    @app.post(
        "/dimos/cloud/logout",
        response_model=models.Account,
        **route_doc(
            "cloud",
            "Log this machine out of Dimensional cloud",
            "Forgets the stored cloud key (dimos's `logout`) and resets the login, then answers the account as GET "
            "/dimos/cloud/account?fresh=1 would.",
            answer="`{ loggedIn, email, scopes, source, cloudUrl, error }`, logged out",
        ),
    )
    async def cloud_logout() -> dict[str, Any]:
        return await s.uploads.logout()

    @app.get(
        "/dimos/uploads",
        response_model=models.UploadList,
        **route_doc(
            "uploads",
            "The Dimensional cloud upload queue: each upload's state, progress, speed, time left and error",
            "The queue in order, and whether it waits for a cloud login. No side effects.",
            agent=True,
            answer="`{ uploads: [Upload], waitingForLogin }`",
        ),
    )
    async def upload_list() -> dict[str, Any]:
        return s.uploads.listing()

    @app.post(
        "/dimos/uploads",
        response_model=models.Upload,
        **route_doc(
            "uploads",
            "Upload a recording (.mcap or .db) to Dimensional cloud: it joins the queue (one at a time)",
            "Adds the recording to the end of the queue (saved, so it survives a restart) and answers its upload, "
            "queued; one already queued or uploading for that path is answered instead. Needs a cloud login: "
            "without one it waits. 400 when the path isn't an absolute path to an existing dimos recording (an .mcap, or a .db dimos recorded).",
            errors=(400, 500),
            agent=True,
            answer="`Upload`",
        ),
    )
    async def enqueue_upload(request: models.UploadRequest) -> dict[str, Any]:
        try:
            return s.uploads.enqueue(request.path, request.robotId, request.kind)
        except ValueError as error:
            raise ApiError(400, str(error))

    @app.delete(
        "/dimos/uploads",
        response_model=models.UploadList,
        **route_doc(
            "uploads",
            "Clear the finished uploads (done, failed, cancelled) from the list",
            "Removes every done, failed or cancelled upload from the list (what is in the cloud is still "
            "remembered, see /dimos/uploads/uploaded) and answers the queue.",
            answer="`{ uploads: [Upload], waitingForLogin }`, without the finished ones",
        ),
    )
    async def clear_uploads() -> dict[str, Any]:
        return s.uploads.clear_finished()

    @app.get(
        "/dimos/uploads/uploaded",
        response_model=models.UploadedByPath | models.Uploaded | None,
        **route_doc(
            "uploads",
            "Which recordings are already in Dimensional cloud (uploaded from this machine), by path, with a "
            "console link; ?path= for one",
            "Without `path`: `{byPath}` for every recording uploaded from here. With `path`: that one's entry, or "
            "null when it isn't uploaded. `changed` says the file differs from what was uploaded. No side effects.",
            errors=(400, 500),
            agent=True,
            answer="`{ byPath: { [path]: Uploaded } }`, or with `?path=` that one `Uploaded` or null",
        ),
    )
    async def uploaded(
        path: Annotated[str | None, Query(description="a recording's absolute path")] = None,
    ) -> Any:
        return s.uploads.uploaded() if path is None else s.uploads.uploaded_one(path)

    @app.delete(
        "/dimos/uploads/{id}",
        response_model=models.Ok,
        **route_doc(
            "uploads",
            "Cancel a queued or running upload, or remove a finished one from the list",
            "A queued upload turns cancelled; a running one's worker is killed and it turns cancelled; a finished "
            "one is removed (an `upload-removed` event). 404 for an unknown id.",
            errors=(404, 500),
            agent=True,
            answer="`{ ok: true }`",
        ),
    )
    async def cancel_upload(id: UploadIdParam) -> dict[str, Any]:
        try:
            s.uploads.cancel(id)
        except KeyError as error:
            raise ApiError(404, error.args[0])
        return {"ok": True}

    @app.post(
        "/dimos/uploads/{id}/retry",
        response_model=models.Upload,
        **route_doc(
            "uploads",
            "Queue a failed or cancelled upload again",
            "Moves a finished (done, failed or cancelled) upload to the back of the queue, reset, and answers it. "
            "404 for an unknown id; 409 while it's still queued or uploading.",
            errors=(404, 409, 500),
            agent=True,
            answer="`Upload`, queued",
        ),
    )
    async def retry_upload(id: UploadIdParam) -> dict[str, Any]:
        try:
            return s.uploads.retry(id)
        except KeyError as error:
            raise ApiError(404, error.args[0])
        except ValueError as error:
            raise ApiError(409, str(error))

    discovery_routes.add(app, state)
    msgs_routes.add(app)
    skills_routes.add(app)

    def openapi() -> dict[str, Any]:
        if app.openapi_schema is None:
            app.openapi_schema = document(app)
        return app.openapi_schema

    app.openapi = openapi  # type: ignore[method-assign]
    return app
