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

"""Export one ``pylock.toml`` per runtime bundle and backend from the checkout's ``uv.lock``.

    python -m dimos.deps.export_locks          # (re)write dimos/deps/locks/
    python -m dimos.deps.export_locks --check  # exit 1 when a file is stale, missing or orphaned

Each file carries the complete locked dependency set of one bundle and backend, with
markers, hashes and resolved artifact URLs, excluding the dimos project itself. They ship
in the wheel and sdist so ``dimos prepare`` works from an installed release.

The export always runs uv ``EXPORT_UV_VERSION`` (``uv tool run`` fetches it once): uv
releases change the exported markers, and ``--check`` compares the files byte for byte.
"""

from __future__ import annotations

import subprocess
import sys

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.deps.bundles import LOCKS_DIR, load_assignments, lock_path

BACKENDS = ("cpu", "cuda")
EXPORT_UV_VERSION = "0.12.24"


def bundle_names() -> list[str]:
    return sorted(set(load_assignments().values()))


def export_command(bundle: str, backend: str) -> list[str]:
    return [
        "uv",
        "tool",
        "run",
        f"uv@{EXPORT_UV_VERSION}",
        "export",
        "--locked",
        "--no-default-groups",
        "--no-emit-project",
        "--no-header",
        "--format",
        "pylock.toml",
        "--extra",
        bundle,
        "--extra",
        backend,
    ]


def export(bundle: str, backend: str) -> bytes:
    result = subprocess.run(
        export_command(bundle, backend), cwd=DIMOS_PROJECT_ROOT, capture_output=True, check=True
    )
    return result.stdout


def main(argv: list[str] | None = None) -> int:
    check = "--check" in (sys.argv[1:] if argv is None else argv)
    expected = {
        lock_path(bundle, backend): (bundle, backend)
        for bundle in bundle_names()
        for backend in BACKENDS
    }
    stale: list[str] = []
    for path, (bundle, backend) in sorted(expected.items()):
        content = export(bundle, backend)
        if check:
            if not path.is_file() or path.read_bytes() != content:
                stale.append(path.name)
        else:
            path.parent.mkdir(exist_ok=True)
            path.write_bytes(content)
            print(f"wrote {path}")
    orphans = sorted(set(LOCKS_DIR.glob("pylock.*.toml")) - set(expected))
    for orphan in orphans:
        if check:
            stale.append(f"{orphan.name} (no such bundle)")
        else:
            orphan.unlink()
            print(f"removed {orphan}")
    if stale:
        print(
            "lock exports out of date: " + ", ".join(stale),
            "regenerate with: python -m dimos.deps.export_locks",
            sep="\n",
            file=sys.stderr,
        )
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
