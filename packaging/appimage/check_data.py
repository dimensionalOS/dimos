# Copyright 2025-2026 Dimensional Inc.
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

"""Check the AppImage archive list, including in a pointer-only checkout."""

import argparse
import csv
from pathlib import Path
import re
import sys

MANIFEST = Path(__file__).with_name("data.tsv")
ARCHIVES = MANIFEST.resolve().parents[2] / "data" / ".lfs"


def archive_size(path: Path, *, require_payload: bool) -> int:
    with path.open("rb") as stream:
        header = stream.read(256)
    if header.startswith(b"\x1f\x8b"):
        return path.stat().st_size
    pointer = re.fullmatch(
        rb"version https://git-lfs.github.com/spec/v1\noid sha256:[0-9a-f]{64}\nsize (\d+)\n",
        header,
    )
    if pointer is None:
        raise ValueError(f"Expected a gzip archive or Git LFS pointer in {path}")
    if require_payload:
        raise ValueError(f"Fetch the Git LFS payload for {path}")
    return int(pointer[1])


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--build-inputs", action="store_true", help="print reviewed archive paths for packaging"
    )
    args = parser.parse_args()
    paths: dict[str, Path] = {}
    total = 0
    with MANIFEST.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream, delimiter="\t"):
            if not all(
                row.get(key)
                for key in ("archive", "owner_area", "licence", "reviewed_by", "use_and_evidence")
            ):
                raise ValueError(f"Missing manifest field at row {len(paths) + 2}")
            name = row["archive"]
            if not re.fullmatch(r"[\w.-]+", name) or name in paths:
                raise ValueError(f"Invalid or duplicate archive name: {name!r}")
            if args.build_inputs and (
                row["reviewed_by"] == "pending" or "NOASSERTION" in row["licence"]
            ):
                raise ValueError(f"Owner or licence review pending for {name}")
            path = ARCHIVES / f"{name}.tar.gz"
            total += archive_size(path, require_payload=args.build_inputs)
            paths[name] = path
    if not paths:
        raise ValueError(f"No archives in {MANIFEST}")
    summary = f"{len(paths)} archives, {total:,} bytes ({total / 1_000_000:.2f} MB) compressed"
    if args.build_inputs:
        print(*paths.values(), sep="\n")
    print(summary, file=sys.stderr if args.build_inputs else sys.stdout)


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError) as exc:
        raise SystemExit(str(exc)) from exc
