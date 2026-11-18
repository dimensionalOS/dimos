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

"""LeRobot v3 directory tar, distinct from Desktop's lineage-bearing bundle."""

from __future__ import annotations

from collections import Counter
from collections.abc import Callable, Iterator
from contextlib import contextmanager
import ctypes
import hashlib
import io
import json
import math
import os
from pathlib import Path
import re
import shutil
import stat
import subprocess
import sys
import tarfile
from typing import Any, BinaryIO

FORMAT = "dimos.lerobot.directory"
DESKTOP = "dimos.lerobot.bundle"
MANIFEST = {"format": FORMAT, "version": 1}
MAX_MEMBERS = 20_000
MAX_BYTES = 100 * 1024**3
MAX_WIRE = MAX_BYTES + MAX_MEMBERS * 1024 + 10240
MAX_META = 16 * 1024**2
MAX_MEMBER = 8 * 1024**3 - 1  # USTAR size is an 11-digit octal field
MAX_EPISODES = 1_000_000
INFO, STATS, TASKS = "meta/info.json", "meta/stats.json", "meta/tasks.parquet"
DATA = "data/chunk-{chunk_index:03d}/file-{file_index:03d}.parquet"
VIDEO = "videos/{video_key}/chunk-{chunk_index:03d}/file-{file_index:03d}.mp4"
FEATURE = r"[A-Za-z][A-Za-z0-9_.-]{0,94}"
SHARD = r"chunk-[0-9]{3,9}/file-[0-9]{3,9}"
EPISODE = re.compile(rf"meta/episodes/{SHARD}\.parquet")
MEMBER = re.compile(
    rf"(?:data/{SHARD}\.parquet|videos/{FEATURE}/{SHARD}\.mp4|{EPISODE.pattern}|meta/(?:info\.json|stats\.json|tasks\.parquet))"
)
DIGEST = re.compile(r"[0-9a-f]{64}")
BASE = set("timestamp frame_index episode_index index task_index".split())
DTYPES = set(
    "bool int8 int16 int32 int64 uint8 uint16 uint32 uint64 float16 float32 float64".split()
)
SUMMARY = set("min max mean std count q01 q10 q50 q90 q99".split())
VIDEO_INFO = set(
    "video.fps video.height video.width video.channels video.codec video.pix_fmt video.is_depth_map has_audio".split()
)


class BundleError(RuntimeError):
    pass


def bundle_format(manifest: Any) -> str | None:
    if not isinstance(manifest, dict) or manifest.get("format") not in (FORMAT, DESKTOP):
        return None
    if type(manifest.get("version")) is not int or manifest["version"] != 1:
        raise BundleError("unsupported LeRobot bundle version")
    return str(manifest["format"])


@contextmanager
def source(path: Path, directory: bool = False) -> Iterator[int]:
    """Never follow a source symlink, including intermediate directories."""
    parts = path.absolute().parts
    fd = os.open(parts[0], os.O_RDONLY | os.O_DIRECTORY)
    try:
        for i, part in enumerate(parts[1:], 2):
            flags = os.O_DIRECTORY if i < len(parts) or directory else os.O_NONBLOCK
            child = os.open(part, os.O_RDONLY | os.O_NOFOLLOW | flags, dir_fd=fd)
            os.close(fd)
            fd = child
        if not directory and not stat.S_ISREG(os.fstat(fd).st_mode):
            raise BundleError("LeRobot members must be regular files")
        yield fd
    finally:
        os.close(fd)


@contextmanager
def read(path: Path) -> Iterator[BinaryIO]:
    with source(path) as fd, os.fdopen(os.dup(fd), "rb") as stream:
        yield stream


def document(path: Path) -> Any:
    with read(path) as stream:
        raw = stream.read(MAX_META + 1)
    if len(raw) > MAX_META:
        raise BundleError("oversized LeRobot JSON")
    return json.loads(raw)


def integer(value: Any, lower: int = 0, upper: int = 999_999_999) -> int:
    if type(value) is not int or not lower <= value <= upper:
        raise BundleError("invalid LeRobot index or count")
    return value


def numeric(value: Any) -> Any:
    if isinstance(value, list):
        return [numeric(item) for item in value]
    if type(value) not in (int, float) or not math.isfinite(value):
        raise BundleError("invalid LeRobot statistic")
    return value


def public_info(raw: Any) -> dict[str, Any]:
    if raw["codebase_version"] != "v3.0" or raw["data_path"] != DATA:
        raise BundleError("expected finalized LeRobot v3.0 with standard path templates")
    features = {}
    for key, feature in raw["features"].items():
        if not (
            key in BASE
            or key in {"action", "complementary_info.is_filled"}
            or key.startswith(("action.", "observation."))
        ):
            continue
        dtype = feature["dtype"]
        if not re.fullmatch(FEATURE, key) or dtype not in DTYPES | {"video"}:
            raise BundleError("unsupported LeRobot feature (expected numeric or video)")
        if dtype == "video" and not key.startswith("observation.images."):
            raise BundleError("unsupported LeRobot camera name")
        shape, names = feature["shape"], feature.get("names")
        if not isinstance(shape, list) or not 1 <= len(shape) <= 8:
            raise BundleError("invalid LeRobot feature shape")
        if names is not None and (
            not isinstance(names, list) or any(not isinstance(v, str) for v in names)
        ):
            raise BundleError("invalid LeRobot feature names")
        item = {"dtype": dtype, "shape": [integer(v, 1) for v in shape], "names": names}
        for field in ("info", "video_info") if dtype == "video" else ():
            if field in feature:
                meta = {
                    k: v
                    for k, v in feature[field].items()
                    if k in VIDEO_INFO and type(v) in (str, int, float, bool)
                }
                if meta.get("video.is_depth_map"):
                    raise BundleError("depth video is not supported")
                if "has_audio" in meta:
                    meta["has_audio"] = False
                item[field] = meta
        if feature.get("is_depth_map"):
            raise BundleError("depth video is not supported")
        features[key] = item
    if not BASE <= features.keys():
        raise BundleError("missing LeRobot frame index features")
    cameras = any(v["dtype"] == "video" for v in features.values())
    if cameras and raw.get("video_path") != VIDEO:
        raise BundleError("unsupported LeRobot video path template")
    episodes = integer(raw["total_episodes"], 1, MAX_EPISODES)
    splits = {}
    for k, span in raw.get("splits", {}).items():
        if k not in {"train", "test", "validation", "val"}:
            continue
        if not isinstance(span, str) or not re.fullmatch(r"[0-9]{1,7}:[0-9]{1,7}", span):
            raise BundleError("invalid LeRobot split range")
        start, stop = (int(v) for v in span.split(":"))
        if not 0 <= start <= stop <= episodes:
            raise BundleError("invalid LeRobot split range")
        splits[k] = span
    out = dict(
        codebase_version="v3.0",
        data_path=DATA,
        video_path=VIDEO if cameras else None,
        fps=integer(raw["fps"], 1, 1000),
        total_episodes=episodes,
        total_frames=integer(raw["total_frames"], episodes),
        total_tasks=integer(raw["total_tasks"], 1, MAX_EPISODES),
        features=features,
        splits=splits or {"train": f"0:{episodes}"},
        robot_type=raw.get("robot_type") if isinstance(raw.get("robot_type"), str) else None,
    )
    for key in ("chunks_size", "data_files_size_in_mb", "video_files_size_in_mb"):
        if key in raw:
            out[key] = integer(raw[key], 1)
    return out


def arrow() -> tuple[Any, Any]:
    try:
        import pyarrow as pa
        import pyarrow.parquet as pq
    except ImportError as exc:
        raise BundleError(
            "LeRobot directory support requires PyArrow (install dimos[learning])"
        ) from exc
    return pa, pq


def parquet(pq: Any, stream: BinaryIO, metadata: bool = False) -> Any:
    size = os.fstat(stream.fileno()).st_size
    if size < 12 or size > (MAX_META if metadata else MAX_BYTES):
        raise BundleError("invalid or oversized LeRobot Parquet file")
    stream.seek(-8, os.SEEK_END)
    footer = stream.read(8)
    if footer[4:] != b"PAR1" or not 0 < int.from_bytes(footer[:4], "little") <= min(
        MAX_META, size - 12
    ):
        raise BundleError("invalid or oversized LeRobot Parquet footer")
    stream.seek(0)
    return pq.ParquetFile(
        stream, thrift_string_size_limit=MAX_META, thrift_container_size_limit=MAX_EPISODES
    )


def inventory(root: Path, info: dict[str, Any], pq: Any) -> tuple[set[str], Counter[str]]:
    cameras = [k for k, v in info["features"].items() if v["dtype"] == "video"]
    refs = {"data": DATA, **{f"videos/{k}": VIDEO for k in cameras}}
    columns = ["episode_index", "length", "dataset_from_index", "dataset_to_index"]
    columns += [f"{k}/{field}_index" for k in refs for field in ("chunk", "file")]
    columns += [f"videos/{k}/{field}_timestamp" for k in cameras for field in ("from", "to")]
    names = {INFO, STATS, TASKS}
    counts: Counter[str] = Counter()
    intervals = {}
    visited = 0
    with source(root / "meta/episodes", True) as fd, os.scandir(fd) as chunks:
        for chunk in chunks:
            visited += 1
            if visited > MAX_MEMBERS:
                raise BundleError("too many LeRobot episode index entries")
            if not re.fullmatch(r"chunk-[0-9]{3,9}", chunk.name):
                continue
            with source(root / "meta/episodes" / chunk.name, True) as cfd, os.scandir(cfd) as files:
                for file in files:
                    visited += 1
                    if visited > MAX_MEMBERS:
                        raise BundleError("too many LeRobot episode index entries")
                    name = f"meta/episodes/{chunk.name}/{file.name}"
                    if not EPISODE.fullmatch(name):
                        continue
                    with read(root / name) as stream:
                        shard = parquet(pq, stream, True)
                        rows = shard.metadata.num_rows
                        if not rows:
                            continue
                        if rows + len(intervals) > info["total_episodes"]:
                            raise BundleError("too many LeRobot episode rows")
                        names.add(name)
                        for batch in shard.iter_batches(columns=columns, batch_size=1024):
                            for row in batch.to_pylist():
                                idx = integer(row["episode_index"], 0, info["total_episodes"] - 1)
                                start = integer(row["dataset_from_index"])
                                stop = integer(
                                    row["dataset_to_index"], start + 1, info["total_frames"]
                                )
                                if idx in intervals or stop - start != integer(row["length"], 1):
                                    raise BundleError("invalid LeRobot episode ranges")
                                intervals[idx] = (start, stop)
                                for key, template in refs.items():
                                    target = template.format(
                                        chunk_index=integer(row[f"{key}/chunk_index"]),
                                        file_index=integer(row[f"{key}/file_index"]),
                                        video_key=key.removeprefix("videos/"),
                                    )
                                    names.add(target)
                                    if key == "data":
                                        counts[target] += stop - start
                                    else:
                                        a, b = (row[f"{key}/{x}_timestamp"] for x in ("from", "to"))
                                        if (
                                            not math.isfinite(a)
                                            or not math.isfinite(b)
                                            or not 0 <= a <= b
                                        ):
                                            raise BundleError("invalid LeRobot video times")
                                if len(names) > MAX_MEMBERS:
                                    raise BundleError("too many LeRobot members")
    cursor = 0
    for idx in range(info["total_episodes"]):
        start, stop = intervals[idx]
        if start != cursor:
            raise BundleError("noncontiguous LeRobot episode ranges")
        cursor = stop
    if cursor != info["total_frames"]:
        raise BundleError("LeRobot frame count does not match episode metadata")
    return names, counts


def clean_field(pa: Any, field: Any, *, strings: bool = False) -> Any:
    t, dtype = pa.types, field.type
    if t.is_fixed_size_list(dtype):
        dtype = pa.list_(clean_field(pa, dtype.value_field, strings=strings), dtype.list_size)
    elif t.is_list(dtype) or t.is_large_list(dtype):
        dtype = (pa.large_list if t.is_large_list(dtype) else pa.list_)(
            clean_field(pa, dtype.value_field, strings=strings)
        )
    elif not (
        t.is_integer(dtype)
        or t.is_floating(dtype)
        or t.is_boolean(dtype)
        or (strings and (t.is_string(dtype) or t.is_large_string(dtype)))
    ):
        raise BundleError("unsupported LeRobot Parquet type")
    return pa.field(field.name, dtype, nullable=field.nullable)


def project_parquet(
    path: Path, out: Path, name: str, info: dict[str, Any], counts: Counter[str], pa: Any, pq: Any
) -> None:
    with read(path) as stream:
        p = parquet(pq, stream, not name.startswith("data/"))
        columns = p.schema_arrow.names
        if len(set(columns)) != len(columns):
            raise BundleError("duplicate LeRobot columns")
        if name == TASKS:
            import pandas as pd

            labels = "task" if "task" in columns else "__index_level_0__"
            if p.metadata.num_rows != info["total_tasks"]:
                raise BundleError("LeRobot task count mismatch")
            table = p.read(columns=["task_index", labels])
            indices, tasks = table["task_index"].to_pylist(), table[labels].to_pylist()
            if (
                any(type(i) is not int for i in indices)
                or sorted(indices) != list(range(len(indices)))
                or any(not isinstance(s, str) for s in tasks)
                or len(set(tasks)) != len(tasks)
            ):
                raise BundleError("invalid LeRobot task index")
            frame = pd.DataFrame(
                {"task_index": indices}, index=pd.Index(tasks, name="task")
            ).sort_values("task_index")
            pq.write_table(pa.Table.from_pandas(frame, preserve_index=True), out)
            return

        features = info["features"]
        if name.startswith("data/"):
            allowed = {key for key in features if features[key]["dtype"] != "video"}
            if p.metadata.num_rows != counts[name] or not allowed <= set(columns):
                raise BundleError(
                    "LeRobot data shard does not match its episode references or features"
                )
        else:
            allowed = set(
                "episode_index tasks length dataset_from_index dataset_to_index data/chunk_index data/file_index meta/episodes/chunk_index meta/episodes/file_index".split()
            )
            allowed |= {
                f"videos/{k}/{f}"
                for k, v in features.items()
                if v["dtype"] == "video"
                for f in ("chunk_index", "file_index", "from_timestamp", "to_timestamp")
            }
            allowed |= {f"stats/{k}/{s}" for k in features for s in SUMMARY}
        selected = [k for k in columns if k in allowed]
        schema = pa.schema(
            [clean_field(pa, p.schema_arrow.field(k), strings=k == "tasks") for k in selected]
        )
        # Copy clean shards byte-for-byte. Only project dirty shards.
        if p.schema_arrow.equals(schema, check_metadata=True) and set(
            p.metadata.metadata or {}
        ) <= {b"ARROW:schema"}:
            stream.seek(0)
            with out.open("wb") as dst:
                shutil.copyfileobj(stream, dst, 2**20)
        else:
            with pq.ParquetWriter(out, schema) as writer:
                for batch in p.iter_batches(batch_size=4096, columns=selected):
                    writer.write_batch(pa.RecordBatch.from_arrays(batch.columns, schema=schema))


def pack(root: Path, target: Path, require_space: Callable[[Path, int], None]) -> None:
    pa, pq = arrow()
    try:
        info = public_info(document(root / INFO))
        names, counts = inventory(root, info, pq)
        sizes = []
        for name in names:
            with source(root / name) as fd:
                size = os.fstat(fd).st_size
                if size > (MAX_META if name.startswith("meta/") else MAX_MEMBER):
                    raise BundleError("LeRobot member exceeds size limit")
                sizes.append(size)
        if sum(sizes) > MAX_BYTES:
            raise BundleError("LeRobot bundle exceeds 100 GiB")
        require_space(target.parent, sum(sizes) + max(sizes) + len(names) * 1024 + 10240)
        stats = {
            k: {s: numeric(v) for s, v in row.items() if s in SUMMARY}
            for k, row in document(root / STATS).items()
            if k in info["features"]
        }
        written = 0
        scratch = target.parent / "member"
        with tarfile.open(target, "w", format=tarfile.USTAR_FORMAT) as archive:
            for name in sorted(names):
                member = tarfile.TarInfo(name)
                member.mode = 0o644
                stream: BinaryIO
                if name in (INFO, STATS):
                    data = json.dumps(
                        info if name == INFO else stats, sort_keys=True, allow_nan=False
                    ).encode()
                    stream = io.BytesIO(data)
                    member.size = len(data)
                else:
                    if name.endswith(".mp4"):
                        # Copy video packets without transcoding; drop additional
                        # tracks, chapters and container/stream metadata.
                        with source(root / name) as fd:
                            subprocess.run(
                                [
                                    "ffmpeg",
                                    "-nostdin",
                                    "-v",
                                    "error",
                                    "-y",
                                    "-protocol_whitelist",
                                    "file",
                                    "-f",
                                    "mov",
                                    "-i",
                                    f"/dev/fd/{fd}",
                                    "-map",
                                    "0:v:0",
                                    "-map_metadata",
                                    "-1",
                                    "-map_metadata:s",
                                    "-1",
                                    "-map_chapters",
                                    "-1",
                                    "-c",
                                    "copy",
                                    "-fflags",
                                    "+bitexact",
                                    "-fs",
                                    str(min(MAX_MEMBER, MAX_BYTES - written) + 1),
                                    "-f",
                                    "mp4",
                                    str(scratch),
                                ],
                                pass_fds=(fd,),
                                stdout=subprocess.DEVNULL,
                                stderr=subprocess.DEVNULL,
                                check=True,
                                timeout=1800,
                            )
                    else:
                        project_parquet(root / name, scratch, name, info, counts, pa, pq)
                    member.size = scratch.stat().st_size
                    stream = scratch.open("rb")
                with stream:
                    written += member.size
                    if (
                        not 0 < member.size <= MAX_MEMBER
                        or written > MAX_BYTES
                        or (name.startswith("meta/") and member.size > MAX_META)
                    ):
                        raise BundleError("LeRobot bundle size limit exceeded")
                    archive.addfile(member, stream)
                scratch.unlink(missing_ok=True)
    except FileNotFoundError as exc:
        raise BundleError("missing LeRobot dataset member or ffmpeg executable") from exc
    except (
        ValueError,
        KeyError,
        TypeError,
        AttributeError,
        RecursionError,
        OverflowError,
        pa.ArrowException,
        tarfile.TarError,
        subprocess.SubprocessError,
    ) as exc:
        raise BundleError("invalid or unsupported LeRobot dataset") from exc


def extract(wire: Path, destination: Path, kind: str) -> None:
    """Parse regular USTAR headers only, never archive extension records."""
    seen: set[str] = set()
    zero = bytes(512)
    total = 0
    try:
        with wire.open("rb") as stream:
            while (header := stream.read(512)) != zero:
                if len(header) != 512 or header[257:265] != b"ustar\x0000":
                    raise BundleError("expected a plain USTAR LeRobot bundle")
                member = tarfile.TarInfo.frombuf(header, "ascii", "strict")
                name = member.name
                if (
                    member.type not in (tarfile.REGTYPE, tarfile.AREGTYPE)
                    or member.linkname
                    or not (
                        MEMBER.fullmatch(name) or (kind == DESKTOP and name == "provenance.json")
                    )
                    or name in seen
                    or len(seen) >= MAX_MEMBERS
                    or not 0 < member.size <= MAX_MEMBER
                ):
                    raise BundleError("unsafe, duplicate or excessive LeRobot archive member")
                seen.add(name)
                total += member.size
                if total > MAX_BYTES or (
                    (name.startswith("meta/") or name == "provenance.json")
                    and member.size > MAX_META
                ):
                    raise BundleError("LeRobot archive size limit exceeded")
                target = destination / name
                target.parent.mkdir(parents=True, exist_ok=True)
                with target.open("xb") as dst:
                    remaining = member.size
                    while remaining:
                        chunk = stream.read(min(remaining, 2**20))
                        if not chunk:
                            raise BundleError("truncated LeRobot archive member")
                        dst.write(chunk)
                        remaining -= len(chunk)
                padding = -member.size % 512
                if stream.read(padding) != bytes(padding):
                    raise BundleError("invalid LeRobot archive padding")
            trailer = stream.read(10240)
            if len(trailer) < 512 or len(trailer) % 512 or any(trailer) or stream.read(1):
                raise BundleError("invalid LeRobot archive trailer")
        if kind == DESKTOP:
            provenance = document(destination / "provenance.json")
            digests, sources = provenance["inventory"], provenance["sources"]
            if not sources or set(digests) != seen - {"provenance.json"}:
                raise BundleError("Desktop bundle lacks source provenance or complete inventory")
            for entry in sources:
                if not isinstance(entry["recordingId"], str) or not all(
                    DIGEST.fullmatch(entry["identity"][k])
                    for k in ("recordingSha256", "schemaSha256")
                ):
                    raise BundleError("invalid Desktop source identity")
            for name, expected in digests.items():
                with (destination / name).open("rb") as src:
                    digest = hashlib.sha256()
                    while chunk := src.read(2**20):
                        digest.update(chunk)
                if digest.hexdigest() != expected:
                    raise BundleError("Desktop bundle inventory hash mismatch")
        pa, pq = arrow()
        try:
            info = public_info(document(destination / INFO))
            names, counts = inventory(destination, info, pq)
            if (kind == FORMAT and names != seen) or (kind == DESKTOP and not names <= seen):
                raise BundleError("LeRobot archive does not match its dataset references")
            for name, rows in {TASKS: info["total_tasks"], **counts}.items():
                with read(destination / name) as stream:
                    if parquet(pq, stream, name == TASKS).metadata.num_rows != rows:
                        raise BundleError("LeRobot archive shard row count mismatch")
        except pa.ArrowException as exc:
            raise BundleError("invalid LeRobot archive metadata") from exc
    except (
        ValueError,
        KeyError,
        TypeError,
        AttributeError,
        RecursionError,
        OverflowError,
        tarfile.TarError,
    ) as exc:
        raise BundleError("invalid LeRobot archive") from exc


def publish_directory(source: Path, target: Path) -> None:
    """Atomic same-filesystem rename that never replaces an empty directory."""
    libc = ctypes.CDLL(None, use_errno=True)
    src, dst = os.fsencode(source), os.fsencode(target)
    if sys.platform == "linux" and hasattr(libc, "renameat2"):
        result = libc.renameat2(-100, src, -100, dst, 1)  # AT_FDCWD, RENAME_NOREPLACE
    elif sys.platform == "darwin":
        result = libc.renamex_np(src, dst, 4)  # RENAME_EXCL
    else:
        raise BundleError("atomic LeRobot restore requires Linux or macOS")
    if result:
        err = ctypes.get_errno()
        raise OSError(err, os.strerror(err), target)
