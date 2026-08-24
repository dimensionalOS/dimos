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

"""Compare storage codecs across complete recorded depth streams.

Run with local SQLite recordings or extracted depth-frame directories::

    uv run python -m dimos.memory.codecs.tool_depth_benchmark recording.db g1_zed \
        --output /tmp/depth-codecs

Every frame in every detected depth stream is measured. Use repeatable
``--stream`` flags to restrict SQLite inputs. ``--synthetic`` is a quick harness
check that requires no LFS data; it is not benchmark evidence.
"""

from __future__ import annotations

import argparse
from collections.abc import Callable, Iterable, Iterator, Sequence
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from functools import partial
import importlib.metadata
from itertools import chain
import json
from pathlib import Path
import pickle
import platform
import sys
import time
from typing import Any

import imagecodecs
import lz4.frame  # type: ignore[import-untyped]
import numpy as np
from rich.progress import (
    BarColumn,
    MofNCompleteColumn,
    Progress,
    TextColumn,
    TimeElapsedColumn,
    TimeRemainingColumn,
)

from dimos.memory.codecs.lcm import LcmCodec
from dimos.memory.codecs.lerc import MAX_ERROR_METERS
from dimos.memory.codecs.zstd import ZstdCodec
from dimos.memory.store.sqlite import SqliteStore
from dimos.memory.stream import Stream
from dimos.msgs.sensor_msgs.CompressedImage import CompressedImage
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.utils.data import get_data

_ZSTD_LCM = ZstdCodec(LcmCodec(Image))
_SYNTHETIC_FRAME_COUNT = 5
_FIDELITY_TOLERANCE_MM = 0.001


@dataclass(frozen=True)
class Candidate:
    name: str
    supports: Callable[[Image], bool]
    encode: Callable[[Image], bytes]
    decode: Callable[[bytes], Image]
    max_error_mm: float
    exact_array: bool = True


@dataclass(frozen=True)
class Fidelity:
    max_error_mm: float
    mean_error_mm: float
    rmse_mm: float
    invalid_mismatch_pct: float


@dataclass(frozen=True)
class BenchmarkRow:
    codec: str
    frames: int
    raw_bytes: int
    encoded_bytes: int
    mean_bytes_per_frame: float
    p50_bytes_per_frame: float
    p95_bytes_per_frame: float
    compression_ratio: float
    encode_wall_p50_ms: float
    encode_wall_p95_ms: float
    encode_wall_total_s: float
    encode_cpu_p50_ms: float
    encode_cpu_p95_ms: float
    encode_cpu_total_s: float
    decode_wall_p50_ms: float
    decode_wall_p95_ms: float
    decode_wall_total_s: float
    decode_cpu_p50_ms: float
    decode_cpu_p95_ms: float
    decode_cpu_total_s: float
    encode_fps: float
    decode_fps: float
    max_error_mm: float
    mean_error_mm: float
    rmse_mm: float
    invalid_mismatch_pct: float


@dataclass(frozen=True)
class SkippedRow:
    codec: str
    status: str
    reason: str


@dataclass(frozen=True)
class DepthStreamSource:
    source: Path
    stream: str
    frame_count: int
    frames: Callable[[], Iterator[Image]]


@dataclass(frozen=True)
class StreamBenchmark:
    source: str
    stream: str
    frames: int
    shape: tuple[int, ...]
    dtype: str
    image_format: str
    codecs: list[BenchmarkRow | SkippedRow]


def _always(_: Image) -> bool:
    return True


def _uint16(image: Image) -> bool:
    return image.format is ImageFormat.DEPTH16 and image.dtype == np.uint16


def _float32(image: Image) -> bool:
    return image.format is ImageFormat.DEPTH and image.dtype == np.float32


def _wrap(image: Image, wire_format: str, blob: bytes) -> bytes:
    return CompressedImage(
        data=blob,
        format=wire_format,
        frame_id=image.frame_id,
        ts=image.ts,
    ).lcm_encode()


def _unwrap(data: bytes, wire_format: str) -> CompressedImage:
    wrapped = CompressedImage.lcm_decode(data)
    if wrapped.format != wire_format:
        raise ValueError(f"expected {wire_format!r}, got {wrapped.format!r}")
    return wrapped


def _image_from_array(array: np.ndarray[Any, Any], wrapped: CompressedImage) -> Image:
    if array.dtype == np.float32:
        image_format = ImageFormat.DEPTH
    elif array.dtype == np.uint16:
        image_format = ImageFormat.DEPTH16
    else:
        raise ValueError(f"unsupported decoded depth dtype {array.dtype}")
    return Image(data=array, format=image_format, frame_id=wrapped.frame_id, ts=wrapped.ts)


def _encode_lcm(image: Image) -> bytes:
    return image.lcm_encode()


def _decode_lcm(data: bytes) -> Image:
    return Image.lcm_decode(data)


def _encode_lz4(image: Image) -> bytes:
    return bytes(lz4.frame.compress(image.lcm_encode()))


def _decode_lz4(data: bytes) -> Image:
    return Image.lcm_decode(lz4.frame.decompress(data))


def _encode_zstd(image: Image) -> bytes:
    return _ZSTD_LCM.encode(image)


def _decode_zstd(data: bytes) -> Image:
    decoded = _ZSTD_LCM.decode(data)
    if not isinstance(decoded, Image):
        raise TypeError(f"expected Image, got {type(decoded).__name__}")
    return decoded


def _encode_png(image: Image) -> bytes:
    import cv2

    ok, blob = cv2.imencode(".png", image.data, [cv2.IMWRITE_PNG_COMPRESSION, 3])
    if not ok:
        raise ValueError("PNG encoding failed")
    return _wrap(image, "depth-png", blob.tobytes())


def _decode_png(data: bytes) -> Image:
    import cv2

    wrapped = _unwrap(data, "depth-png")
    pixels = cv2.imdecode(np.frombuffer(wrapped.data, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
    if pixels is None:
        raise ValueError("PNG decoding failed")
    return _image_from_array(pixels, wrapped)


def _encode_jpegxl(image: Image) -> bytes:
    blob = bytes(imagecodecs.jpegxl_encode(image.data, lossless=True, effort=1))
    return _wrap(image, "depth-jxl", blob)


def _decode_jpegxl(data: bytes) -> Image:
    wrapped = _unwrap(data, "depth-jxl")
    return _image_from_array(imagecodecs.jpegxl_decode(wrapped.data), wrapped)


def _encode_zfp(image: Image) -> bytes:
    blob = bytes(imagecodecs.zfp_encode(image.data, mode="reversible"))
    return _wrap(image, "depth-zfp", blob)


def _decode_zfp(data: bytes) -> Image:
    wrapped = _unwrap(data, "depth-zfp")
    return _image_from_array(imagecodecs.zfp_decode(wrapped.data), wrapped)


def _lerc_level(image: Image, max_error_m: float) -> float:
    return max_error_m if image.format is ImageFormat.DEPTH else max_error_m * 1000.0


def _lerc_candidate(max_error_m: float) -> Candidate:
    label = "lerc-lossless" if max_error_m == 0 else f"lerc-{max_error_m * 1000:g}mm"
    wire_format = f"bench-{label}"

    def encode(image: Image) -> bytes:
        valid = np.isfinite(image.data) & (image.data > 0)
        pixels = np.ascontiguousarray(np.where(valid, image.data, 0))
        blob = bytes(
            imagecodecs.lerc_encode(
                pixels,
                level=_lerc_level(image, max_error_m),
                masks=valid,
            )
        )
        return _wrap(image, wire_format, blob)

    def decode(data: bytes) -> Image:
        wrapped = _unwrap(data, wire_format)
        pixels, valid = imagecodecs.lerc_decode(wrapped.data, masks=True)
        pixels = np.asarray(pixels)
        if valid is None:
            valid = np.ones(pixels.shape, dtype=np.bool_)
        if pixels.dtype == np.float32:
            pixels[~valid] = np.nan
            pixels[valid & (pixels <= 0)] = np.nextafter(np.float32(0), np.float32(1))
        else:
            pixels[~valid] = 0
            pixels[valid & (pixels == 0)] = 1
        return _image_from_array(pixels, wrapped)

    return Candidate(
        label,
        _always,
        encode,
        decode,
        max_error_m * 1000.0,
        exact_array=False,
    )


def candidates() -> list[Candidate]:
    return [
        Candidate("raw-lcm", _always, _encode_lcm, _decode_lcm, 0.0),
        Candidate("lz4+lcm", _always, _encode_lz4, _decode_lz4, 0.0),
        Candidate("zstd3+lcm", _always, _encode_zstd, _decode_zstd, 0.0),
        Candidate("png3", _uint16, _encode_png, _decode_png, 0.0),
        Candidate("jpegxl-lossless", _always, _encode_jpegxl, _decode_jpegxl, 0.0),
        Candidate("zfp-reversible", _float32, _encode_zfp, _decode_zfp, 0.0),
        _lerc_candidate(0.0),
        _lerc_candidate(0.001),
        _lerc_candidate(MAX_ERROR_METERS),
        _lerc_candidate(0.010),
    ]


def fidelity(source: Image, decoded: Image) -> Fidelity:
    source_valid = np.isfinite(source.data) & (source.data > 0)
    decoded_valid = np.isfinite(decoded.data) & (decoded.data > 0)
    mismatch_pct = float(np.mean(source_valid != decoded_valid) * 100)
    jointly_valid = source_valid & decoded_valid
    if not np.any(jointly_valid):
        return Fidelity(0.0, 0.0, 0.0, mismatch_pct)

    scale_to_mm = 1000.0 if source.format is ImageFormat.DEPTH else 1.0
    errors = (
        np.abs(
            source.data[jointly_valid].astype(np.float64)
            - decoded.data[jointly_valid].astype(np.float64)
        )
        * scale_to_mm
    )
    return Fidelity(
        max_error_mm=float(np.max(errors)),
        mean_error_mm=float(np.mean(errors)),
        rmse_mm=float(np.sqrt(np.mean(np.square(errors)))),
        invalid_mismatch_pct=mismatch_pct,
    )


def _measure(call: Callable[[], Any]) -> tuple[Any, float, float]:
    wall_start = time.perf_counter_ns()
    cpu_start = time.process_time_ns()
    result = call()
    cpu_ns = time.process_time_ns() - cpu_start
    wall_ns = time.perf_counter_ns() - wall_start
    return result, wall_ns / 1e6, cpu_ns / 1e6


def _percentile(values: Sequence[float] | Sequence[int], percentile: float) -> float:
    return float(np.percentile(values, percentile))


class _FidelityAccumulator:
    def __init__(self) -> None:
        self.max_error_mm = 0.0
        self.absolute_error_sum_mm = 0.0
        self.squared_error_sum_mm = 0.0
        self.valid_pixels = 0
        self.invalid_mismatches = 0
        self.total_pixels = 0

    def add(self, source: Image, decoded: Image) -> Fidelity:
        source_valid = np.isfinite(source.data) & (source.data > 0)
        decoded_valid = np.isfinite(decoded.data) & (decoded.data > 0)
        mismatches = source_valid != decoded_valid
        jointly_valid = source_valid & decoded_valid
        self.invalid_mismatches += int(np.count_nonzero(mismatches))
        self.total_pixels += int(source.data.size)

        if not np.any(jointly_valid):
            return Fidelity(0.0, 0.0, 0.0, float(np.mean(mismatches) * 100))

        scale_to_mm = 1000.0 if source.format is ImageFormat.DEPTH else 1.0
        errors = (
            np.abs(
                source.data[jointly_valid].astype(np.float64)
                - decoded.data[jointly_valid].astype(np.float64)
            )
            * scale_to_mm
        )
        frame_max = float(np.max(errors))
        self.max_error_mm = max(self.max_error_mm, frame_max)
        self.absolute_error_sum_mm += float(np.sum(errors))
        self.squared_error_sum_mm += float(np.sum(np.square(errors)))
        self.valid_pixels += int(errors.size)
        return Fidelity(
            max_error_mm=frame_max,
            mean_error_mm=float(np.mean(errors)),
            rmse_mm=float(np.sqrt(np.mean(np.square(errors)))),
            invalid_mismatch_pct=float(np.mean(mismatches) * 100),
        )

    def result(self) -> Fidelity:
        if self.valid_pixels:
            mean_error = self.absolute_error_sum_mm / self.valid_pixels
            rmse = (self.squared_error_sum_mm / self.valid_pixels) ** 0.5
        else:
            mean_error = 0.0
            rmse = 0.0
        mismatch_pct = (
            self.invalid_mismatches / self.total_pixels * 100 if self.total_pixels else 0.0
        )
        return Fidelity(self.max_error_mm, mean_error, rmse, mismatch_pct)


class _MetricAccumulator:
    def __init__(self, candidate: Candidate) -> None:
        self.candidate = candidate
        self.raw_bytes = 0
        self.encoded_bytes = 0
        self.sizes: list[int] = []
        self.encode_wall: list[float] = []
        self.encode_cpu: list[float] = []
        self.decode_wall: list[float] = []
        self.decode_cpu: list[float] = []
        self.fidelity = _FidelityAccumulator()

    def add(
        self,
        source: Image,
        decoded: Image,
        raw_size: int,
        payload_size: int,
        encode_wall_ms: float,
        encode_cpu_ms: float,
        decode_wall_ms: float,
        decode_cpu_ms: float,
    ) -> None:
        self.raw_bytes += raw_size
        self.encoded_bytes += payload_size
        self.sizes.append(payload_size)
        self.encode_wall.append(encode_wall_ms)
        self.encode_cpu.append(encode_cpu_ms)
        self.decode_wall.append(decode_wall_ms)
        self.decode_cpu.append(decode_cpu_ms)
        frame_fidelity = self.fidelity.add(source, decoded)
        if frame_fidelity.invalid_mismatch_pct != 0.0:
            raise ValueError(
                f"{self.candidate.name} changed the invalid-pixel mask "
                f"({frame_fidelity.invalid_mismatch_pct:.6f}% mismatch)"
            )
        if frame_fidelity.max_error_mm > (self.candidate.max_error_mm + _FIDELITY_TOLERANCE_MM):
            raise ValueError(
                f"{self.candidate.name} exceeded its {self.candidate.max_error_mm:g} mm "
                f"error bound ({frame_fidelity.max_error_mm:.6f} mm)"
            )

    def row(self) -> BenchmarkRow:
        frames = len(self.sizes)
        fidelity_result = self.fidelity.result()
        encode_wall_total_s = sum(self.encode_wall) / 1000.0
        decode_wall_total_s = sum(self.decode_wall) / 1000.0
        return BenchmarkRow(
            codec=self.candidate.name,
            frames=frames,
            raw_bytes=self.raw_bytes,
            encoded_bytes=self.encoded_bytes,
            mean_bytes_per_frame=self.encoded_bytes / frames,
            p50_bytes_per_frame=_percentile(self.sizes, 50),
            p95_bytes_per_frame=_percentile(self.sizes, 95),
            compression_ratio=self.raw_bytes / self.encoded_bytes,
            encode_wall_p50_ms=_percentile(self.encode_wall, 50),
            encode_wall_p95_ms=_percentile(self.encode_wall, 95),
            encode_wall_total_s=encode_wall_total_s,
            encode_cpu_p50_ms=_percentile(self.encode_cpu, 50),
            encode_cpu_p95_ms=_percentile(self.encode_cpu, 95),
            encode_cpu_total_s=sum(self.encode_cpu) / 1000.0,
            decode_wall_p50_ms=_percentile(self.decode_wall, 50),
            decode_wall_p95_ms=_percentile(self.decode_wall, 95),
            decode_wall_total_s=decode_wall_total_s,
            decode_cpu_p50_ms=_percentile(self.decode_cpu, 50),
            decode_cpu_p95_ms=_percentile(self.decode_cpu, 95),
            decode_cpu_total_s=sum(self.decode_cpu) / 1000.0,
            encode_fps=frames / encode_wall_total_s,
            decode_fps=frames / decode_wall_total_s,
            max_error_mm=fidelity_result.max_error_mm,
            mean_error_mm=fidelity_result.mean_error_mm,
            rmse_mm=fidelity_result.rmse_mm,
            invalid_mismatch_pct=fidelity_result.invalid_mismatch_pct,
        )


def _validate_frame(frame: Image, reference: Image, index: int) -> None:
    signature = (frame.format, frame.dtype, frame.shape)
    expected = (reference.format, reference.dtype, reference.shape)
    if signature != expected:
        raise ValueError(
            f"depth frame {index} changed format, dtype, or shape: "
            f"expected {expected}, got {signature}"
        )


def _validate_decoded(candidate: Candidate, source: Image, decoded: Image) -> None:
    if (
        decoded.format is not source.format
        or decoded.dtype != source.dtype
        or decoded.shape != source.shape
        or decoded.frame_id != source.frame_id
        or not np.isclose(decoded.ts, source.ts, rtol=0.0, atol=1e-9)
    ):
        raise ValueError(f"{candidate.name} did not preserve image metadata")
    if (
        candidate.exact_array
        and candidate.max_error_mm == 0.0
        and not np.array_equal(source.data, decoded.data, equal_nan=True)
    ):
        raise ValueError(f"{candidate.name} did not reconstruct depth values exactly")


def benchmark(
    frames: Iterable[Image],
    *,
    frame_count: int | None = None,
    progress_label: str | None = None,
) -> list[BenchmarkRow | SkippedRow]:
    iterator = iter(frames)
    try:
        first = next(iterator)
    except StopIteration:
        raise ValueError("no depth frames to benchmark") from None
    if first.format not in (ImageFormat.DEPTH, ImageFormat.DEPTH16):
        raise ValueError(f"expected a depth image, got {first.format}")

    all_candidates = candidates()
    supported = [candidate for candidate in all_candidates if candidate.supports(first)]
    accumulators = {candidate.name: _MetricAccumulator(candidate) for candidate in supported}
    for candidate in supported:
        payload = candidate.encode(first)
        decoded = candidate.decode(payload)
        _validate_decoded(candidate, first, decoded)

    total = frame_count
    if total is None and isinstance(frames, Sequence):
        total = len(frames)
    progress: Progress | None = None
    task_id: Any = None
    if progress_label is not None and total is not None:
        progress = Progress(
            TextColumn("[progress.description]{task.description}"),
            BarColumn(),
            MofNCompleteColumn(),
            TimeElapsedColumn(),
            TimeRemainingColumn(),
        )
        progress.start()
        task_id = progress.add_task(progress_label, total=total)

    seen = 0
    try:
        for index, frame in enumerate(chain((first,), iterator)):
            _validate_frame(frame, first, index)
            raw_size = len(frame.lcm_encode())
            for candidate in supported:
                try:
                    payload, encode_wall_ms, encode_cpu_ms = _measure(
                        partial(candidate.encode, frame)
                    )
                    decoded, decode_wall_ms, decode_cpu_ms = _measure(
                        partial(candidate.decode, payload)
                    )
                    _validate_decoded(candidate, frame, decoded)
                    accumulators[candidate.name].add(
                        frame,
                        decoded,
                        raw_size,
                        len(payload),
                        encode_wall_ms,
                        encode_cpu_ms,
                        decode_wall_ms,
                        decode_cpu_ms,
                    )
                except Exception as error:
                    raise RuntimeError(
                        f"{candidate.name} failed on depth frame {index}: {error}"
                    ) from error
            seen += 1
            if progress is not None:
                progress.advance(task_id)
    finally:
        if progress is not None:
            progress.stop()

    if frame_count is not None and seen != frame_count:
        raise ValueError(f"expected {frame_count} frames, read {seen}")

    rows: list[BenchmarkRow | SkippedRow] = []
    for candidate in all_candidates:
        accumulator = accumulators.get(candidate.name)
        if accumulator is None:
            rows.append(
                SkippedRow(
                    codec=candidate.name,
                    status="unsupported",
                    reason=f"unsupported for {first.format}/{first.dtype}",
                )
            )
        else:
            rows.append(accumulator.row())
    return rows


def synthetic_frames(dtype: str, count: int) -> list[Image]:
    height, width = 240, 320
    yy, xx = np.mgrid[:height, :width]
    depth_m = 0.3 + xx * 0.008 + yy * 0.002
    depth_m[:, width // 2 :] += 1.5
    depth_m[(xx - 90) ** 2 + (yy - 120) ** 2 < 35**2] = np.nan
    rng = np.random.default_rng(7)
    result: list[Image] = []
    for index in range(count):
        noisy = depth_m + rng.normal(0, 0.0015, depth_m.shape) + index * 0.0002
        if dtype == "float32":
            pixels = noisy.astype(np.float32)
            image_format = ImageFormat.DEPTH
        else:
            pixels = np.nan_to_num(noisy * 1000, nan=0).astype(np.uint16)
            image_format = ImageFormat.DEPTH16
        result.append(
            Image(
                data=pixels,
                format=image_format,
                frame_id="synthetic_depth",
                ts=float(index + 1),
            )
        )
    return result


def _depth_format(pixels: np.ndarray[Any, Any]) -> ImageFormat:
    if pixels.dtype == np.float32:
        return ImageFormat.DEPTH
    if pixels.dtype == np.uint16:
        return ImageFormat.DEPTH16
    raise ValueError(f"unsupported depth dtype {pixels.dtype}")


def _pickle_frame(path: Path, source: Path) -> Image:
    payload = pickle.loads(path.read_bytes())
    if isinstance(payload, Image):
        return payload
    if (
        not isinstance(payload, tuple)
        or len(payload) != 2
        or not isinstance(payload[0], (int, float))
    ):
        raise ValueError(f"unsupported depth pickle payload in {path}")
    timestamp, pixels = payload
    array = np.asarray(pixels)
    return Image(
        data=array,
        format=_depth_format(array),
        frame_id=source.name,
        ts=float(timestamp),
    )


def _png_frame(path: Path, source: Path, index: int) -> Image:
    import cv2

    pixels = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if pixels is None:
        raise ValueError(f"failed to read depth image {path}")
    return Image(
        data=pixels,
        format=_depth_format(pixels),
        frame_id=source.name,
        ts=float(index + 1),
    )


def _directory_source(source: Path) -> DepthStreamSource:
    depth_dir = source / "depth" if (source / "depth").is_dir() else source
    pickle_paths = tuple(sorted(depth_dir.glob("*.pickle")))
    png_paths = tuple(sorted(depth_dir.glob("*.png")))
    frame_count = len(pickle_paths) + len(png_paths)
    if frame_count == 0:
        raise ValueError(f"no depth .pickle or .png frames found in {source}")

    def frames() -> Iterator[Image]:
        for path in pickle_paths:
            yield _pickle_frame(path, source)
        for index, path in enumerate(png_paths):
            yield _png_frame(path, source, index)

    return DepthStreamSource(
        source=source.resolve(),
        stream=depth_dir.name,
        frame_count=frame_count,
        frames=frames,
    )


def _sqlite_frames(source: Path, stream_name: str) -> Iterator[Image]:
    with SqliteStore(path=str(source), must_exist=True) as store:
        stream: Stream[Any] = store.stream(stream_name)
        for index, observation in enumerate(stream):
            value = observation.data
            if not isinstance(value, Image) or value.format not in (
                ImageFormat.DEPTH,
                ImageFormat.DEPTH16,
            ):
                raise ValueError(
                    f"{source}:{stream_name} contains non-depth value at frame {index}"
                )
            yield value


def discover_depth_streams(
    source: Path, stream_names: Sequence[str] = ()
) -> list[DepthStreamSource]:
    if source.is_dir():
        if stream_names:
            raise ValueError("--stream applies only to SQLite recordings")
        return [_directory_source(source)]

    with SqliteStore(path=str(source), must_exist=True) as store:
        available = store.list_streams()
        missing = [name for name in stream_names if name not in available]
        if missing:
            raise ValueError(
                f"streams not found in {source}: {', '.join(missing)}; "
                f"available: {', '.join(available)}"
            )
        names = list(dict.fromkeys(stream_names)) if stream_names else available
        result: list[DepthStreamSource] = []
        for name in names:
            stream: Stream[Any] = store.stream(name)
            try:
                sample = stream.first().data
            except LookupError:
                if stream_names:
                    raise ValueError(f"stream {name!r} in {source} is empty") from None
                continue
            if not isinstance(sample, Image) or sample.format not in (
                ImageFormat.DEPTH,
                ImageFormat.DEPTH16,
            ):
                if stream_names:
                    raise ValueError(f"stream {name!r} in {source} is not a depth-image stream")
                continue
            result.append(
                DepthStreamSource(
                    source=source.resolve(),
                    stream=name,
                    frame_count=stream.count(),
                    frames=partial(_sqlite_frames, source, name),
                )
            )
    if not result:
        raise ValueError(
            f"no depth-image streams found in {source}; available: {', '.join(available)}"
        )
    return result


def resolve_source(value: str) -> Path:
    path = Path(value)
    return path if path.exists() else get_data(value)


def print_table(rows: Sequence[BenchmarkRow | SkippedRow]) -> None:
    header = (
        f"{'codec':<18} {'avg bytes':>10} {'ratio':>7} {'enc wall p50/p95':>20} "
        f"{'enc cpu p50/p95':>19} {'dec wall p50/p95':>20} {'dec cpu p50/p95':>19} "
        f"{'max err':>9} {'rmse':>8} {'mask':>7}"
    )
    print(header)
    print("-" * len(header))
    for row in rows:
        if isinstance(row, SkippedRow):
            print(f"{row.codec:<18} {'N/A':>10}  {row.reason}")
            continue
        print(
            f"{row.codec:<18} {row.mean_bytes_per_frame:>10.0f} "
            f"{row.compression_ratio:>6.2f}x "
            f"{row.encode_wall_p50_ms:>8.2f}/{row.encode_wall_p95_ms:<7.2f}ms "
            f"{row.encode_cpu_p50_ms:>7.2f}/{row.encode_cpu_p95_ms:<7.2f}ms "
            f"{row.decode_wall_p50_ms:>8.2f}/{row.decode_wall_p95_ms:<7.2f}ms "
            f"{row.decode_cpu_p50_ms:>7.2f}/{row.decode_cpu_p95_ms:<7.2f}ms "
            f"{row.max_error_mm:>7.3f}mm {row.rmse_mm:>6.3f}mm "
            f"{row.invalid_mismatch_pct:>6.3f}%"
        )


def _benchmark_source(source: DepthStreamSource) -> StreamBenchmark:
    frame_iterator = source.frames()
    try:
        first = next(frame_iterator)
    except StopIteration:
        raise ValueError(f"{source.source}:{source.stream} is empty") from None
    rows = benchmark(
        chain((first,), frame_iterator),
        frame_count=source.frame_count,
        progress_label=f"{source.source.name}:{source.stream}",
    )
    return StreamBenchmark(
        source=str(source.source),
        stream=source.stream,
        frames=source.frame_count,
        shape=first.shape,
        dtype=str(first.dtype),
        image_format=first.format.value,
        codecs=rows,
    )


def _package_version(name: str) -> str:
    try:
        return importlib.metadata.version(name)
    except importlib.metadata.PackageNotFoundError:
        return "unknown"


def _run_document(
    results: Sequence[StreamBenchmark],
    inputs: Sequence[str],
    started_at: datetime,
) -> dict[str, Any]:
    return {
        "schema_version": 1,
        "run": {
            "started_at": started_at.isoformat(),
            "completed_at": datetime.now(timezone.utc).isoformat(),
            "platform": platform.platform(),
            "python": sys.version.split()[0],
            "packages": {
                "imagecodecs": _package_version("imagecodecs"),
                "lz4": _package_version("lz4"),
                "opencv-contrib-python": _package_version("opencv-contrib-python"),
                "zstandard": _package_version("zstandard"),
            },
            "inputs": list(inputs),
            "timing": "one warmup per codec and stream; one timed call per frame",
        },
        "results": [asdict(result) for result in results],
    }


def _markdown(document: dict[str, Any]) -> str:
    run = document["run"]
    lines = [
        "# Depth codec benchmark",
        "",
        f"Started: `{run['started_at']}`  ",
        f"Completed: `{run['completed_at']}`  ",
        f"Platform: `{run['platform']}`  ",
        f"Python: `{run['python']}`",
        "",
        "Times are milliseconds per frame. Each cell is p50 / p95. Ratios use total raw",
        "bytes divided by total encoded bytes across the complete stream.",
        "",
    ]
    for result in document["results"]:
        shape = "x".join(str(value) for value in result["shape"])
        lines.extend(
            [
                f"## `{result['source']}` — `{result['stream']}`",
                "",
                f"{result['frames']} frames; {shape}; `{result['dtype']}`; "
                f"`{result['image_format']}`.",
                "",
                "| Codec | Avg bytes | Ratio | Encode wall | Encode CPU | "
                "Decode wall | Decode CPU | Max error | RMSE | Mask mismatch |",
                "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
            ]
        )
        for row in result["codecs"]:
            if row.get("status") == "unsupported":
                lines.append(
                    f"| {row['codec']} | N/A | N/A | N/A | N/A | N/A | N/A | "
                    f"N/A | N/A | {row['reason']} |"
                )
                continue
            lines.append(
                f"| {row['codec']} | {row['mean_bytes_per_frame']:.0f} | "
                f"{row['compression_ratio']:.2f}x | "
                f"{row['encode_wall_p50_ms']:.2f} / {row['encode_wall_p95_ms']:.2f} | "
                f"{row['encode_cpu_p50_ms']:.2f} / {row['encode_cpu_p95_ms']:.2f} | "
                f"{row['decode_wall_p50_ms']:.2f} / {row['decode_wall_p95_ms']:.2f} | "
                f"{row['decode_cpu_p50_ms']:.2f} / {row['decode_cpu_p95_ms']:.2f} | "
                f"{row['max_error_mm']:.3f} mm | {row['rmse_mm']:.3f} mm | "
                f"{row['invalid_mismatch_pct']:.6f}% |"
            )
        lines.append("")
    return "\n".join(lines)


def write_results(
    output: Path,
    results: Sequence[StreamBenchmark],
    inputs: Sequence[str],
    started_at: datetime,
) -> None:
    if output.exists() and any(output.iterdir()):
        raise ValueError(f"output directory is not empty: {output}")
    output.mkdir(parents=True, exist_ok=True)
    document = _run_document(results, inputs, started_at)
    (output / "results.json").write_text(json.dumps(document, indent=2) + "\n")
    (output / "results.md").write_text(_markdown(document) + "\n")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "recordings",
        nargs="*",
        help="SQLite paths, depth-frame directories, or named LFS datasets",
    )
    parser.add_argument(
        "--stream",
        action="append",
        default=[],
        help="SQLite depth stream to include; repeat to select several (default: all)",
    )
    parser.add_argument(
        "--synthetic",
        choices=("uint16", "float32"),
        help="use generated depth instead of a recording",
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="new or empty output directory for results.json and results.md",
    )
    args = parser.parse_args()
    if args.synthetic and args.recordings:
        parser.error("recordings and --synthetic cannot be used together")
    if not args.synthetic and not args.recordings:
        parser.error("provide at least one recording or use --synthetic")
    if not args.synthetic and args.output is None:
        parser.error("real-recording benchmarks require --output")
    if args.output is not None and args.output.exists() and any(args.output.iterdir()):
        parser.error(f"output directory is not empty: {args.output}")

    started_at = datetime.now(timezone.utc)
    if args.synthetic:
        frames = synthetic_frames(args.synthetic, _SYNTHETIC_FRAME_COUNT)
        rows = benchmark(frames)
        print(f"\nsynthetic-{args.synthetic}")
        print_table(rows)
        results = [
            StreamBenchmark(
                source=f"synthetic-{args.synthetic}",
                stream="depth",
                frames=len(frames),
                shape=frames[0].shape,
                dtype=str(frames[0].dtype),
                image_format=frames[0].format.value,
                codecs=rows,
            )
        ]
        inputs = [f"synthetic-{args.synthetic}"]
    else:
        inputs = list(args.recordings)
        resolved = [resolve_source(value) for value in inputs]
        sources = [
            source
            for recording in resolved
            for source in discover_depth_streams(recording, args.stream)
        ]
        results = []
        for source in sources:
            print(f"\n{source.source}:{source.stream} ({source.frame_count} frames)")
            result = _benchmark_source(source)
            print_table(result.codecs)
            results.append(result)

    if args.output is not None:
        write_results(args.output, results, inputs, started_at)
        print(f"\nWrote {args.output / 'results.json'} and {args.output / 'results.md'}")


if __name__ == "__main__":
    main()
