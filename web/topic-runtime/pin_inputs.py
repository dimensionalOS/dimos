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

"""Resolve release-only runtime inputs into a reviewed, fixed-hash Nix manifest.

Uses the runtime's frozen uv.lock, official Python standalone releases selected
by uv, and the exact Deno version used by build_runtime.ts. This command never
runs at Desktop startup. Regenerate only when intentionally updating a pin.
"""

import argparse
import base64
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import subprocess
import tomllib
from typing import Any

from packaging.markers import Marker
from packaging.tags import compatible_tags, cpython_tags, mac_platforms
from packaging.utils import parse_wheel_filename

PYTHON = "3.12.13"
DENO = "2.9.8"
TARGETS = {
    "x86_64-linux": ("linux", "x86_64", "x86_64-unknown-linux-gnu"),
    "aarch64-linux": ("linux", "aarch64", "aarch64-unknown-linux-gnu"),
    "x86_64-darwin": ("macos", "x86_64", "x86_64-apple-darwin"),
    "aarch64-darwin": ("macos", "aarch64", "aarch64-apple-darwin"),
}


def prefetch(url: str) -> dict[str, str]:
    result = json.loads(
        subprocess.check_output(["nix", "store", "prefetch-file", "--json", url], text=True)
    )
    return {"url": url, "hash": result["hash"]}


def wheels(system: str, packages: list[dict[str, Any]]) -> list[dict[str, str]]:
    os_name, arch, _ = TARGETS[system]
    if os_name == "linux":
        platforms = [f"manylinux_2_{version}_{arch}" for version in range(28, 16, -1)]
        platforms += [f"manylinux2014_{arch}", f"linux_{arch}"]
    else:
        platforms = list(
            mac_platforms(version=(11, 0), arch="arm64" if arch == "aarch64" else arch)
        )
    tags = [
        *cpython_tags((3, 12), ["cp312"], platforms),
        *compatible_tags((3, 12), "cp312", platforms),
    ]
    ranks = {tag: index for index, tag in reversed(list(enumerate(tags)))}
    selected = []
    for package in packages:
        if package["name"] == "dimos-topic-runtime":
            continue
        markers = package.get("resolution-markers", [])
        environment = {
            "sys_platform": "darwin" if os_name == "macos" else "linux",
            "platform_machine": "arm64" if os_name == "macos" and arch == "aarch64" else arch,
            "python_version": "3.12",
            "python_full_version": PYTHON,
        }
        if markers and not any(Marker(marker).evaluate(environment) for marker in markers):
            continue
        choices = []
        for wheel in package.get("wheels", []):
            filename = wheel["url"].rsplit("/", 1)[-1]
            matches = [ranks[tag] for tag in parse_wheel_filename(filename)[3] if tag in ranks]
            if matches:
                choices.append((min(matches), filename, wheel))
        if not choices:
            raise ValueError(f"no pinned {system} wheel for {package['name']}")
        _, filename, wheel = min(choices, key=lambda item: item[0])
        algorithm, digest = wheel["hash"].split(":")
        if algorithm != "sha256":
            raise ValueError("unexpected wheel checksum algorithm")
        selected.append(
            {
                "name": filename,
                "url": wheel["url"],
                "hash": "sha256-" + base64.b64encode(bytes.fromhex(digest)).decode(),
            }
        )
    return selected


def main(output: Path) -> None:
    downloads = json.loads(
        subprocess.check_output(
            [
                "uv",
                "python",
                "list",
                "--all-platforms",
                "--all-arches",
                "--only-downloads",
                "--all-versions",
                "--output-format",
                "json",
                PYTHON,
            ],
            text=True,
        )
    )
    packages = tomllib.loads(Path(__file__).with_name("uv.lock").read_text())["package"]
    result: dict[str, Any] = {
        "format": 1,
        "pythonVersion": PYTHON,
        "denoVersion": DENO,
        "platforms": {},
    }
    result["denoLicense"] = prefetch(
        f"https://raw.githubusercontent.com/denoland/deno/v{DENO}/LICENSE.md"
    )
    with ThreadPoolExecutor(max_workers=4) as executor:
        pending = []
        for system, (os_name, arch, target) in TARGETS.items():
            (python,) = [
                item
                for item in downloads
                if item["os"] == os_name
                and item["arch"] == arch
                and item["libc"] == ("gnu" if os_name == "linux" else "none")
            ]
            platform: dict[str, Any] = {
                "target": target,
                "os": "darwin" if os_name == "macos" else os_name,
                "arch": arch,
                "minimumGlibc": "2.28" if os_name == "linux" else None,
                "wheels": wheels(system, packages),
            }
            result["platforms"][system] = platform
            for name, url in {
                "python": python["url"],
                "deno": f"https://dl.deno.land/release/v{DENO}/deno-{target}.zip",
                "denort": f"https://dl.deno.land/release/v{DENO}/denort-{target}.zip",
            }.items():
                pending.append((platform, name, executor.submit(prefetch, url)))
        for platform, name, future in pending:
            platform[name] = future.result()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=4) + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    main(parser.parse_args().output)
