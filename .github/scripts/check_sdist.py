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

"""Assert a release sdist ships the pre-built Cockpit and no generated junk.

Usage: check_sdist.py dist/dimos-*.tar.gz
"""

import sys
import tarfile

# Real content is ~8 MB uncompressed; the web/node_modules regression alone
# added ~74 MB. Generous headroom below that failure regime.
MAX_UNCOMPRESSED_MB = 40

# Paths below the dimos-<version>/ root that every release sdist must carry.
REQUIRED = (
    "web/cockpit/dist/index.html",
    "web/deno.lock",
    "web/relay/main.ts",
    "web/sdk/deno.json",
)


def main(sdist: str) -> None:
    with tarfile.open(sdist) as tar:
        members = tar.getmembers()
    names = {m.name.split("/", 1)[1] for m in members if "/" in m.name}

    problems = [f"missing {path}" for path in REQUIRED if path not in names]
    generated = sorted(n for n in names if "node_modules" in n.split("/"))
    if generated:
        problems.append(f"{len(generated)} node_modules entries, e.g. {generated[0]}")
    total_mb = sum(m.size for m in members) / 1e6
    if total_mb > MAX_UNCOMPRESSED_MB:
        problems.append(f"{total_mb:.1f} MB uncompressed exceeds {MAX_UNCOMPRESSED_MB} MB")

    if problems:
        sys.exit(f"{sdist} failed checks:\n" + "\n".join(f"- {p}" for p in problems))
    print(f"{sdist} ok: {len(members)} entries, {total_mb:.1f} MB uncompressed")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit(f"usage: {sys.argv[0]} <sdist.tar.gz>")
    main(sys.argv[1])
