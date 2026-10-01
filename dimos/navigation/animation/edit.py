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

"""Edit the waypoints a running ``animation-waypoints`` picker shows.

python -m dimos.navigation.animation.edit place 3   # next click moves waypoint 3
python -m dimos.navigation.animation.edit insert 3  # next click becomes a new waypoint 3
python -m dimos.navigation.animation.edit delete 3
python -m dimos.navigation.animation.edit list
"""

import json
from pathlib import Path

import typer

from dimos.navigation.animation.waypoints import (
    WAYPOINTS_FILE,
    load_waypoints,
    pending_path,
    save_waypoints,
)

app = typer.Typer(no_args_is_help=True)
FILE = typer.Option(WAYPOINTS_FILE, "--file", "-f")


def _check(points: list[list[float]], index: int, allow_end: bool = False) -> None:
    top = len(points) if allow_end else len(points) - 1
    if not 0 <= index <= top:
        raise typer.BadParameter(f"index {index} outside 0..{top}")


def _pend(file: Path, op: str, index: int) -> None:
    pending_path(file).write_text(json.dumps({"op": op, "index": index}))
    print(f"click to {op} waypoint {index}")


@app.command()
def place(index: int, file: Path = FILE) -> None:
    """The next click moves waypoint INDEX."""
    _check(load_waypoints(file), index)
    _pend(file, "place", index)


@app.command()
def insert(index: int, file: Path = FILE) -> None:
    """The next click is inserted as waypoint INDEX; later ones shift up."""
    _check(load_waypoints(file), index, allow_end=True)
    _pend(file, "insert", index)


@app.command()
def delete(index: int, file: Path = FILE) -> None:
    """Remove waypoint INDEX now."""
    points = load_waypoints(file)
    _check(points, index)
    print(f"deleted {index}: {points.pop(index)}")
    save_waypoints(file, points)


@app.command("list")
def list_(file: Path = FILE) -> None:
    """Print the waypoints, and what the next click will do."""
    for i, p in enumerate(load_waypoints(file)):
        print(f"{i:3d}  {p[0]:9.3f} {p[1]:9.3f} {p[2]:7.3f}")
    pending = pending_path(file)
    print(f"next click: {json.loads(pending.read_text()) if pending.exists() else 'append'}")


@app.command()
def cancel(file: Path = FILE) -> None:
    """Drop a pending place/insert; the next click appends again."""
    pending_path(file).unlink(missing_ok=True)


if __name__ == "__main__":
    app()
