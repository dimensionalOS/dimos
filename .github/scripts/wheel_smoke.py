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
pinned DENO_VERSION there, exactly as it does on a customer machine. It also
checks that the dependency catalog and the resolver policy ship in the wheel
and that the launcher can explain a blueprint from them.
"""

import json
from pathlib import Path
import subprocess
import sys
import urllib.request

from dimos.deps.catalog import CATALOG_PATH, default_catalog
from dimos.deps.policy import (
    SHIPPED_CONSTRAINTS_FILE,
    SHIPPED_PROJECT_FILE,
    load_constraints,
    load_uv_policy,
    render_wheel_project,
)
from dimos.deps.probe import ProbeRequest
from dimos.deps.profiles import PROFILES
from dimos.navigation.go2.replanning_a_star.min_cost_astar_ext import (
    min_cost_astar_cpp,  # noqa: F401
)
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


def check_dependency_catalog() -> None:
    """The static catalog, the resolver policy and the launcher work from the wheel."""
    if not CATALOG_PATH.is_file():
        raise SystemExit(f"missing from installed wheel: {CATALOG_PATH}")
    if "unitree-go2" not in default_catalog().names:
        raise SystemExit("blueprint catalog in the wheel does not list unitree-go2")
    if not SHIPPED_PROJECT_FILE.is_file():
        raise SystemExit(f"missing from installed wheel: {SHIPPED_PROJECT_FILE}")
    policy = load_uv_policy(SHIPPED_PROJECT_FILE)
    if not policy.override_dependencies:
        raise SystemExit("shipped pyproject.toml carries no uv override policy")
    if not SHIPPED_CONSTRAINTS_FILE.is_file():
        raise SystemExit(f"missing from installed wheel: {SHIPPED_CONSTRAINTS_FILE}")
    constraints = load_constraints(SHIPPED_CONSTRAINTS_FILE)
    if not any(line.startswith("numpy==") for line in constraints):
        raise SystemExit("constraints.txt in the wheel pins no numpy version")
    rendered = render_wheel_project(
        "0", (), "3.12", PROFILES["linux-x86_64-cpu"], policy, constraints
    )
    if constraints[0] not in rendered:
        raise SystemExit("the managed project does not apply the shipped constraints")
    explained = subprocess.run(
        [sys.executable, "-m", "dimos.cli.dimos", "deps", "unitree-go2"],
        capture_output=True,
        text=True,
        timeout=120,
    )
    if explained.returncode != 0 or "unitree" not in explained.stdout:
        raise SystemExit(f"dimos deps failed in the wheel:\n{explained.stdout}\n{explained.stderr}")
    request = ProbeRequest(extras=(), checks=("packages", "providers"))
    probed = subprocess.run(
        [sys.executable, "-m", "dimos.deps.probe"],
        input=json.dumps(request.to_json()),
        capture_output=True,
        text=True,
        timeout=120,
    )
    if probed.returncode != 0:
        raise SystemExit(f"environment probe failed in the wheel:\n{probed.stderr}")
    report = json.loads(probed.stdout)
    if not report.get("dimos_version") or report.get("missing"):
        raise SystemExit(f"the wheel's own core requirements are not satisfied: {report}")


def main() -> None:
    check_dependency_catalog()
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
