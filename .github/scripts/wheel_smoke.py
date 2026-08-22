#!/usr/bin/env python3
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

"""Release-wheel smoke test, run by cibuildwheel against the installed wheel.

Imports the native extension, asserts the packaged relay + Cockpit files, then
starts the packaged relay and fetches /api/info and the Cockpit at /. Deno is
not preinstalled in the manylinux test containers: ensure_deno() downloads the
pinned DENO_VERSION there, exactly as it does on a customer machine.
"""

import json
from pathlib import Path
import urllib.request

from dimos.navigation.replanning_a_star.min_cost_astar_ext import min_cost_astar_cpp  # noqa: F401
from dimos.web.relay_bridge import locate
from dimos.web.relay_bridge.relay_process import RelayProcess

REQUIRED = (
    "deno.json",
    "deno.lock",
    "relay/main.ts",
    "shared/protocol.ts",
    "cockpit/dist/index.html",
    "sdk/deno.json",
    "sdk/dist/sdk.js",
)

# First start in a fresh container downloads Deno plus the relay's jsr deps.
RELAY_READY_TIMEOUT_S = 120.0


def main() -> None:
    dist = Path(locate.__file__).resolve().parent / "_relay_dist"
    for rel in REQUIRED:
        if not (dist / rel).is_file():
            raise SystemExit(f"missing from installed wheel: {dist / rel}")
    leaked = [p for p in dist.rglob("*") if p.name.endswith("_test.ts") or p.name == "testdata"]
    if leaked:
        raise SystemExit(f"test files leaked into the wheel: {leaked}")
    web_dir = locate.find_web_dir()
    if web_dir != dist:
        raise SystemExit(f"find_web_dir() resolved {web_dir}, not the packaged {dist}")

    with RelayProcess(timeout=RELAY_READY_TIMEOUT_S) as info:
        if not info.cockpit:
            raise SystemExit("relay started without the packaged Cockpit dist")
        base = f"http://127.0.0.1:{info.http_port}"
        with urllib.request.urlopen(f"{base}/api/info", timeout=30) as resp:
            api_info = json.load(resp)
        missing = [key for key in ("wtUrl", "certHash", "v") if key not in api_info]
        if missing:
            raise SystemExit(f"/api/info missing {missing}: {api_info}")
        with urllib.request.urlopen(f"{base}/", timeout=30) as resp:
            index = resp.read()
        if index != (dist / "cockpit" / "dist" / "index.html").read_bytes():
            raise SystemExit("/ did not serve the packaged Cockpit index.html")
        with urllib.request.urlopen(f"{base}/sdk.js", timeout=30) as resp:
            sdk_js = resp.read()
            acao = resp.headers.get("Access-Control-Allow-Origin")
        if sdk_js != (dist / "sdk" / "dist" / "sdk.js").read_bytes():
            raise SystemExit("/sdk.js did not serve the packaged SDK bundle")
        if acao != "*":
            raise SystemExit(f"/sdk.js missing the local CORS header (got {acao!r})")
    print("wheel smoke ok")


if __name__ == "__main__":
    main()
