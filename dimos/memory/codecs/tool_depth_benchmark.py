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

"""Compare storage codecs on recorded depth frames.

Run with local SQLite recordings or extracted depth-frame directories::

    uv run python -m dimos.memory.codecs.tool_depth_benchmark recording.db g1_zed

Omit recordings to pull the default RealSense recording through ``get_data``.
Use ``--synthetic`` for a quick check that requires no LFS data.
"""

from __future__ import annotations

import argparse
from collections.abc import Callable, Sequence
from dataclasses import asdict, dataclass
from functools import partial
import json
from pathlib import Path
import pickle
import statistics
import time
from typing import Any

import cv2
import imagecodecs
import lz4.frame  # type: ignore[import-untyped]
import numpy as np

from dimos.memory.codecs.lcm import LcmCodec
from dimos.memory.codecs.lerc import MAX_ERROR_METERS
from dimos.memory.codecs.zstd import ZstdCodec
from dimos.memory.store.sqlite import SqliteStore
from dimos.memory.stream import Stream
from dimos.msgs.sensor_msgs.CompressedImage import CompressedImage
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.utils.data import get_data

_DEFAULT_RECORDING = (
    "xarm6_worldbelief_realsense_d435i_stationery_calibrated/"
    "xarm6_worldbelief_20260729_203624_161992.db"
)
_ZSTD_LCM = ZstdCodec(LcmCodec(Image))


@dataclass(frozen=True)
class Candidate:
    name: str
    supports: Callable[[Image], bool]
    encode: Callable[[Image], bytes]
    decode: Callable[[bytes], Image]


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
    bytes_per_frame: float
    compression_ratio: float
    encode_wall_ms: float
    encode_cpu_ms: float
    decode_wall_ms: float
    decode_cpu_ms: float
    max_error_mm: float
    mean_error_mm: float
    rmse_mm: float
    invalid_mismatch_pct: float


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
    ok, blob = cv2.imencode(".png", image.data, [cv2.IMWRITE_PNG_COMPRESSION, 3])
    if not ok:
        raise ValueError("PNG encoding failed")
    return _wrap(image, "depth-png", blob.tobytes())


def _decode_png(data: bytes) -> Image:
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

    return Candidate(label, _always, encode, decode)


def candidates() -> list[Candidate]:
    return [
        Candidate("raw-lcm", _always, _encode_lcm, _decode_lcm),
        Candidate("lz4+lcm", _always, _encode_lz4, _decode_lz4),
        Candidate("zstd3+lcm", _always, _encode_zstd, _decode_zstd),
        Candidate("png3", _uint16, _encode_png, _decode_png),
        Candidate("jpegxl-lossless", _uint16, _encode_jpegxl, _decode_jpegxl),
        Candidate("zfp-reversible", _float32, _encode_zfp, _decode_zfp),
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
        return Fidelity(float("inf"), float("inf"), float("inf"), mismatch_pct)

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


def _measure(call: Callable[[], Any], repeats: int) -> tuple[Any, float, float]:
    result = call()
    wall_start = time.perf_counter_ns()
    cpu_start = time.process_time_ns()
    for _ in range(repeats):
        result = call()
    cpu_ns = time.process_time_ns() - cpu_start
    wall_ns = time.perf_counter_ns() - wall_start
    return result, wall_ns / repeats / 1e6, cpu_ns / repeats / 1e6


def benchmark(frames: Sequence[Image], repeats: int = 3) -> list[BenchmarkRow]:
    if not frames:
        raise ValueError("no depth frames to benchmark")
    raw_sizes = [len(frame.lcm_encode()) for frame in frames]
    rows: list[BenchmarkRow] = []

    for candidate in candidates():
        supported = [frame for frame in frames if candidate.supports(frame)]
        if not supported:
            continue
        encode = candidate.encode
        decode = candidate.decode

        sizes: list[int] = []
        encode_wall: list[float] = []
        encode_cpu: list[float] = []
        decode_wall: list[float] = []
        decode_cpu: list[float] = []
        fidelities: list[Fidelity] = []
        for frame in supported:
            payload, encode_wall_ms, encode_cpu_ms = _measure(partial(encode, frame), repeats)
            decoded, decode_wall_ms, decode_cpu_ms = _measure(partial(decode, payload), repeats)
            sizes.append(len(payload))
            encode_wall.append(encode_wall_ms)
            encode_cpu.append(encode_cpu_ms)
            decode_wall.append(decode_wall_ms)
            decode_cpu.append(decode_cpu_ms)
            fidelities.append(fidelity(frame, decoded))

        median_size = statistics.median(sizes)
        comparable_raw = [
            raw_size
            for frame, raw_size in zip(frames, raw_sizes, strict=True)
            if candidate.supports(frame)
        ]
        rows.append(
            BenchmarkRow(
                codec=candidate.name,
                frames=len(supported),
                bytes_per_frame=median_size,
                compression_ratio=statistics.median(comparable_raw) / median_size,
                encode_wall_ms=statistics.median(encode_wall),
                encode_cpu_ms=statistics.median(encode_cpu),
                decode_wall_ms=statistics.median(decode_wall),
                decode_cpu_ms=statistics.median(decode_cpu),
                max_error_mm=max(item.max_error_mm for item in fidelities),
                mean_error_mm=statistics.mean(item.mean_error_mm for item in fidelities),
                rmse_mm=statistics.mean(item.rmse_mm for item in fidelities),
                invalid_mismatch_pct=max(item.invalid_mismatch_pct for item in fidelities),
            )
        )
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
            Image(data=pixels, format=image_format, frame_id="synthetic_depth", ts=float(index))
        )
    return result


def _depth_format(pixels: np.ndarray[Any, Any]) -> ImageFormat:
    if pixels.dtype == np.float32:
        return ImageFormat.DEPTH
    if pixels.dtype == np.uint16:
        return ImageFormat.DEPTH16
    raise ValueError(f"unsupported depth dtype {pixels.dtype}")


def _directory_frames(source: Path, count: int) -> list[Image]:
    depth_dir = source / "depth" if (source / "depth").is_dir() else source
    pickle_paths = sorted(depth_dir.glob("*.pickle"))
    png_paths = sorted(depth_dir.glob("*.png"))
    frames: list[Image] = []

    for path in pickle_paths[:count]:
        payload = pickle.loads(path.read_bytes())
        if isinstance(payload, Image):
            frames.append(payload)
            continue
        if (
            not isinstance(payload, tuple)
            or len(payload) != 2
            or not isinstance(payload[0], (int, float))
        ):
            raise ValueError(f"unsupported depth pickle payload in {path}")
        timestamp, pixels = payload
        array = np.asarray(pixels)
        frames.append(
            Image(
                data=array,
                format=_depth_format(array),
                frame_id=source.name,
                ts=float(timestamp),
            )
        )

    for index, path in enumerate(png_paths[: count - len(frames)]):
        pixels = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
        if pixels is None:
            raise ValueError(f"failed to read depth image {path}")
        frames.append(
            Image(
                data=pixels,
                format=_depth_format(pixels),
                frame_id=source.name,
                ts=float(index),
            )
        )

    if not frames:
        raise ValueError(f"no depth .pickle or .png frames found in {source}")
    return frames


def load_frames(source: Path, stream_name: str | None, count: int) -> list[Image]:
    if source.is_dir():
        if stream_name is not None:
            raise ValueError("--stream applies only to SQLite recordings")
        return _directory_frames(source, count)

    frames: list[Image] = []
    with SqliteStore(path=str(source), must_exist=True) as store:
        names = [stream_name] if stream_name else store.list_streams()
        for name in names:
            if name is None:
                continue
            stream: Stream[Any] = store.stream(name)
            for observation in stream.limit(count - len(frames)):
                value = observation.data
                if isinstance(value, Image) and value.format in (
                    ImageFormat.DEPTH,
                    ImageFormat.DEPTH16,
                ):
                    frames.append(value)
                if len(frames) == count:
                    return frames
    if not frames:
        available = ", ".join(names)
        raise ValueError(f"no depth images found in {source}; streams: {available}")
    return frames


def resolve_source(value: str) -> Path:
    path = Path(value)
    return path if path.exists() else get_data(value)


def print_table(rows: Sequence[BenchmarkRow]) -> None:
    header = (
        f"{'codec':<18} {'bytes':>10} {'ratio':>7} {'enc wall':>10} {'enc cpu':>9} "
        f"{'dec wall':>10} {'dec cpu':>9} {'max err':>9} {'rmse':>8} {'mask':>7}"
    )
    print(header)
    print("-" * len(header))
    for row in rows:
        print(
            f"{row.codec:<18} {row.bytes_per_frame:>10.0f} {row.compression_ratio:>6.2f}x "
            f"{row.encode_wall_ms:>8.2f}ms {row.encode_cpu_ms:>7.2f}ms "
            f"{row.decode_wall_ms:>8.2f}ms {row.decode_cpu_ms:>7.2f}ms "
            f"{row.max_error_mm:>7.3f}mm {row.rmse_mm:>6.3f}mm "
            f"{row.invalid_mismatch_pct:>6.3f}%"
        )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "recordings",
        nargs="*",
        help="SQLite paths, depth-frame directories, or named LFS datasets",
    )
    parser.add_argument("--stream", help="depth stream name; auto-detect by default")
    parser.add_argument("--frames", type=int, default=20, help="frames to sample (default: 20)")
    parser.add_argument("--repeats", type=int, default=3, help="timing repeats (default: 3)")
    parser.add_argument(
        "--synthetic",
        choices=("uint16", "float32"),
        help="use generated depth instead of a recording",
    )
    parser.add_argument("--json", type=Path, dest="json_path", help="also write JSON results")
    args = parser.parse_args()
    if args.frames <= 0 or args.repeats <= 0:
        parser.error("--frames and --repeats must be positive")
    if args.synthetic and args.recordings:
        parser.error("recordings and --synthetic cannot be used together")

    if args.synthetic:
        sources = {f"synthetic-{args.synthetic}": synthetic_frames(args.synthetic, args.frames)}
    else:
        requested = args.recordings or [_DEFAULT_RECORDING]
        resolved = [resolve_source(value) for value in requested]
        sources = {
            str(source): load_frames(source, args.stream, args.frames) for source in resolved
        }

    results: dict[str, list[BenchmarkRow]] = {}
    for source, frames in sources.items():
        print(f"\n{source}")
        rows = benchmark(frames, repeats=args.repeats)
        print_table(rows)
        results[source] = rows
    if args.json_path:
        serialized = {source: [asdict(row) for row in rows] for source, rows in results.items()}
        args.json_path.write_text(json.dumps(serialized, indent=2) + "\n")


if __name__ == "__main__":
    main()
