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


"""What the /dimos API offers, as dimos.yaml's `provides:` (the shape apps declare theirs in, paths relative to
/dimos/): Desktop reads it from the checkout to check apps' `uses: "@dimos-gateway"` without running anything.

Each route's `route_doc` summary is its one-line description. Regenerate dimos.yaml's block with
`python -m dimos.gateway --write-provides`; test_provides.py fails while it's stale.
"""

from __future__ import annotations

import json
from pathlib import Path
import re
from typing import TYPE_CHECKING, Any

from dimos.gateway.models import ErrorResponse

if TYPE_CHECKING:
    from fastapi import FastAPI

DIMOS_YAML = Path(__file__).parents[2] / "dimos.yaml"
WRITE_COMMAND = "python -m dimos.gateway --write-provides"
DESCRIPTION = (
    "The dimos gateway: blueprints, global config, runs and their logs, events, Dimensional cloud uploads, discovery "
    "(blueprints, modules, message types), docs, extras, jobs and the running blueprint's skills and module RPC methods"
)
PREFIX = "/dimos/"
# the `provides:` block: its key and every indented line under it
BLOCK = re.compile(r"^provides:\n(?: .*\n)*", re.MULTILINE)

ERRORS = {
    400: "Bad request: a missing or malformed body or parameter, or a value the gateway refuses (the message says "
    "which)",
    404: "No such thing (the message names it)",
    409: "Conflict with the current state (the message says what it is)",
    500: "The gateway couldn't do it: a child process, launch, stop or cloud call failed (the message says why)",
}


def route_doc(
    tag: str,
    summary: str,
    description: str,
    errors: tuple[int, ...] = (500,),
    ok: dict[str, Any] | None = None,
    answer: str = "",
) -> dict[str, Any]:
    """FastAPI route kwargs: the summary (`provides:`'s description), docs and error answers; `ok` documents a 200
    that isn't JSON."""
    return {
        "tags": [tag],
        "summary": summary,
        "description": description,
        "responses": {
            **({200: ok} if ok else {}),
            **{code: {"model": ErrorResponse, "description": ERRORS[code]} for code in errors},
        },
        "response_description": answer,
        # an answer has exactly the fields the gateway set: an optional one that's absent stays absent
        "response_model_exclude_unset": True,
    }


def gateway_app() -> FastAPI:
    """An app to read the routes from (its state is never used)."""
    from dimos.gateway.app import ServerState, create_app
    from dimos.gateway.events import Bus
    from dimos.gateway.uploads import Uploads

    bus = Bus()
    nowhere = Path("/nonexistent")
    return create_app(
        ServerState(nowhere, bus, Uploads(nowhere, bus, None, nowhere / "log")), background=False
    )


def provides(app: FastAPI) -> dict[str, Any]:
    """`{description, endpoints: [{method, path, description}]}` for every /dimos/ route, in the app's order."""
    from fastapi.routing import APIRoute

    endpoints = [
        {"method": method, "path": route.path.removeprefix(PREFIX), "description": route.summary}
        for route in app.routes
        if isinstance(route, APIRoute) and route.path.startswith(PREFIX)
        for method in sorted(route.methods - {"HEAD", "OPTIONS"})
    ]
    for endpoint in endpoints:
        if not endpoint["description"] or "\n" in endpoint["description"]:
            raise ValueError(
                f"{endpoint['method']} {endpoint['path']}: route_doc needs a one-line summary"
            )
    return {"description": DESCRIPTION, "endpoints": endpoints}


def block(offered: dict[str, Any]) -> str:
    """The `provides:` YAML block, one endpoint per line (JSON strings are YAML scalars)."""
    lines = ["provides:", f"  description: {json.dumps(offered['description'])}", "  endpoints:"]
    lines += [
        f"    - {{method: {e['method']}, path: {json.dumps(e['path'])}, description: {json.dumps(e['description'])}}}"
        for e in offered["endpoints"]
    ]
    return "\n".join(lines) + "\n"


def updated(text: str, offered: dict[str, Any]) -> str:
    """dimos.yaml's text with its `provides:` block replaced."""
    if not BLOCK.search(text):
        raise ValueError("dimos.yaml has no top-level `provides:` to replace")
    return BLOCK.sub(lambda _: block(offered), text, count=1)


def write(path: Path = DIMOS_YAML) -> Path:
    path.write_text(updated(path.read_text(), provides(gateway_app())))
    return path


def stale_problems(path: Path = DIMOS_YAML) -> list[str]:
    text = path.read_text()
    if not BLOCK.search(text) or updated(text, provides(gateway_app())) != text:
        return [
            f"{path.name}'s provides: doesn't match the gateway's routes: run `{WRITE_COMMAND}`"
        ]
    return []
