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

"""Stamp observations with poses pulled from another stream by nearest timestamp.

Some recorders store raw sensor streams (``lidar``, images) without baking the
trajectory into each frame — the pose lives in a separate odometry stream
(``odom``, ``fastlio_odometry``). PGO and the voxel rebuild skip pose-less
frames, so such a recording yields an empty map. :func:`pose_fill` re-attaches
poses by nearest-in-time match (via :meth:`Stream.align`) so those tools work
again. Frames stay in whatever coordinate frame they were stored in — only the
pose *metadata* is added.

:func:`pose_fill_db` runs the same fill while copying a whole SQLite dataset to
a new file, baking the pulled poses into the target stream.
"""

from __future__ import annotations

from collections.abc import Iterable
from pathlib import Path
from typing import Any, cast

from dimos.memory2.backend import Backend
from dimos.memory2.stream import Stream
from dimos.memory2.type.observation import Observation


def pose_fill(
    stream: Stream[Any], pose_stream: Stream[Any], *, tolerance: float = 0.1
) -> Stream[Any]:
    """Re-pose each observation in *stream* from the nearest entry in *pose_stream*.

    Pairs are formed by :meth:`Stream.align` (nearest ``|Δts| <= tolerance``);
    primaries with no match in tolerance are dropped. The pose is read from the
    matched pose observation's *payload* (``PoseStamped`` / ``Odometry`` / any
    object exposing ``.position`` + ``.orientation``) because pose-message
    streams carry the pose in the value, not the indexed pose columns. The
    target stream's payload stays lazy.
    """

    def _fill(pair_obs: Observation[Any]) -> Observation[Any]:
        primary, secondary = cast(
            "tuple[Observation[Any], Observation[Any]]", pair_obs.data
        )  # AlignedPair(primary, secondary)
        return primary.with_pose(secondary.data)

    return stream.align(pose_stream, tolerance=tolerance).map(_fill)


def pose_fill_db(
    src_path: str | Path,
    dest_path: str | Path,
    *,
    target: str = "lidar",
    pose_source: str = "odom",
    tolerance: float = 0.1,
    streams: list[str] | None = None,
) -> dict[str, int]:
    """Copy a SQLite dataset to *dest_path*, baking *pose_source* poses into *target*.

    Every stream in *streams* (default: all streams in the source) is copied
    with its original payload type and codec. Blobs are copied **verbatim** —
    the encoded bytes are moved as-is, never decoded and re-encoded — so lossy
    codecs (e.g. jpeg images) keep their original quality. Only the *target*
    stream's pose metadata is rewritten, to the nearest *pose_source* pose (the
    rtree spatial index is rebuilt from it on insert). Returns a per-stream
    count of observations written.
    """
    from dimos.memory2.store.sqlite import SqliteStore

    dest_p = Path(dest_path)
    if dest_p.exists():
        raise FileExistsError(
            f"{dest_p} already exists — refusing to append into it (stale rows would "
            "collide on ts). Delete it (and any -wal/-shm sidecars) first."
        )

    src = SqliteStore(path=str(src_path), must_exist=True)
    dest = SqliteStore(path=str(dest_path))
    names = streams if streams is not None else src.list_streams()
    if target not in names:
        raise ValueError(f"target stream {target!r} not in {names}")
    if pose_source not in src.list_streams():
        raise ValueError(f"pose_source stream {pose_source!r} not found in source dataset")

    written: dict[str, int] = {}
    for name in names:
        src_b = cast("Backend[Any]", src.stream(name)._source)
        dest_b = cast("Backend[Any]", dest.stream(name, src_b.data_type, codec=src_b.codec)._source)

        # Re-posed observations for the target (ids preserved from source, so
        # the source blob still resolves); plain metadata for everything else.
        if name == target:
            rows: Iterable[Observation[Any]] = pose_fill(
                src.stream(name), src.stream(pose_source), tolerance=tolerance
            )
        else:
            rows = src.stream(name).order_by("ts")

        scalar = src_b.data_type in (int, float)
        n = 0
        for obs in rows:
            blob = (
                None if scalar or src_b.blob_store is None else src_b.blob_store.get(name, obs.id)
            )
            row_id = dest_b.metadata_store.insert(obs)
            if blob is not None:
                assert dest_b.blob_store is not None
                dest_b.blob_store.put(name, row_id, blob)
            n += 1
        commit = getattr(dest_b.metadata_store, "commit", None)
        if commit is not None:
            commit()
        written[name] = n

    src.stop()
    dest.stop()
    return written


def main(
    dataset: str,
    out: str | None = None,
    target: str = "lidar",
    pose_source: str = "odom",
    tolerance: float = 0.1,
    streams: str | None = None,
) -> None:
    """Write a copy of *dataset* with *pose_source* poses baked into *target*.

    Run as ``python -m dimos.mapping.loop_closure.utils.pose_fill <dataset>``.
    The output (``<dataset>_posed.db`` by default) is then usable directly with
    ``dimos map reconstruct <out> --pgo``.

    *streams* is a comma-separated subset to copy (default: all) — restrict to
    e.g. ``lidar,odom`` for a smaller map-only db instead of copying every
    stream (images, fastlio, …). Blobs are copied verbatim either way.
    """
    from dimos.utils.data import resolve_named_path

    src = resolve_named_path(dataset, ".db")
    dest = Path(out) if out else src.with_name(f"{src.stem}_posed.db")
    names = [s.strip() for s in streams.split(",")] if streams else None
    print(f"pose-filling {src.name}: {target!r} <- nearest {pose_source!r} (±{tolerance}s)")
    written = pose_fill_db(
        src, dest, target=target, pose_source=pose_source, tolerance=tolerance, streams=names
    )
    for name, n in written.items():
        print(f"  {name}: {n}")
    print(f"wrote {dest}")
    print(f"now run: dimos map reconstruct {dest.stem} --pgo")


if __name__ == "__main__":
    import typer

    typer.run(main)
