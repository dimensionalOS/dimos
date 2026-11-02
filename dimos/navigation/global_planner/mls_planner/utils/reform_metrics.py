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

"""How well the unitree-go2-mls-reform stack follows GSC's loop closures, over a go2 recording.

run:   replays the recording through the whole stack at 1x (`dimos run ... --replay`), with or
       without closures (reform_min_shift_m=1e9 turns them off in both reform modules), sampling the
       native modules' RSS every 5 s. The modules dump their maps and GSC its keyframes as they go.
score: rebuilds the ray tracer's map from scratch with every scan placed by GSC's final corrected
       keyframes, and the planner's surface from that map, and compares them with the reformed ones.
       Writes <out>/<name>.metrics.json.

events: GSC's closure dump as the event file tests/reform_against_fast_reform.rs reads
       (MLS_REFORM_EVENTS=<out>/<name>.events.bin).

usage: reform_metrics.py run --db <recording.db> --name <name> --out <dir> [--no-closures]
       reform_metrics.py score --db <recording.db> --name <name> --out <dir>
       reform_metrics.py events --db <recording.db> --name <name> --out <dir>
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import signal
import subprocess
import time

from dimos_mls_planner import MLSPlanner
import numpy as np
import psutil
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from dimos.mapping.ray_tracing.voxel_map import VoxelRayMapper
from dimos.memory.store.sqlite import SqliteStore
from dimos.robot.unitree.go2.nav_3d_config import mls_planner_config, ray_tracing_config

BLUEPRINT = "unitree-go2-mls-reform"
NATIVE = ("mls_reform_planner", "reform_ray_tracing", "gsc-pgo")
RSS_PERIOD_S = 5.0
# after the recording ends: GSC's last closure, then a dump period or two
SETTLE_S = 60.0
NO_CLOSURES = "1e9"
# ts, raw xyz + xyzw, corrected xyz + xyzw (gsc_pgo dump_keyframes)
KEYFRAME = np.dtype([("ts", "<f8"), ("raw", "<f8", 7), ("corrected", "<f8", 7)])
# one loop_closure_records record: pre pose, id, ts, then the delta with post = delta * pre
CLOSURE_RECORD = np.dtype(
    [
        ("xyz", "<f4", 3),
        ("id", "<u4"),
        ("ts", "<f8"),
        ("q", "<f4", 4),
        ("dt", "<f4", 3),
        ("dq", "<f4", 4),
    ]
)
# `pgo-eval --event-dump`'s record, what tests/reform_against_fast_reform.rs reads
EVENT_RECORD = np.dtype(
    [("id", "<u4"), ("ts", "<f8"), ("pre", "<f4", 7), ("post", "<f4", 7), ("local", "<f4", 7)]
)


def paths(out: Path, name: str, mode: str) -> dict[str, Path]:
    stem = f"{name}.{mode}"
    suffixes = {
        "log": "log",
        "jsonl": "jsonl",
        "rss": "rss.jsonl",
        "tracer": "tracer.f32",
        "planner": "planner",
        "keyframes": "keyframes.f64",
    }
    return {key: out / f"{stem}.{suffix}" for key, suffix in suffixes.items()}


def recording_span_s(db: Path) -> float:
    with SqliteStore(path=str(db)) as store:
        stamps = [obs.ts for obs in store.streams.lidar.order_by("ts")]
    return stamps[-1] - stamps[0]


def newest_run_log(since: float) -> Path | None:
    """The main.jsonl of the newest dimos run started after `since`."""
    logs = [p for p in Path("logs").glob("*/main.jsonl") if p.stat().st_mtime >= since]
    return max(logs, key=lambda p: p.parent.stat().st_ctime, default=None)


def native_processes() -> dict[str, psutil.Process]:
    found = {}
    for proc in psutil.process_iter(["cmdline"]):
        cmdline = " ".join(proc.info["cmdline"] or [])
        for name in NATIVE:
            if f"result/bin/{name}" in cmdline:
                found[name] = proc
    return found


def run(db: Path, name: str, out: Path, closures: bool) -> None:
    out.mkdir(parents=True, exist_ok=True)
    mode = "closures" if closures else "no_closures"
    files = paths(out, name, mode)
    for key in ("tracer", "keyframes"):
        files[key].unlink(missing_ok=True)
    Path(f"{files['keyframes']}.closures").unlink(missing_ok=True)
    span = recording_span_s(db)
    command = [
        "dimos",
        "run",
        BLUEPRINT,
        "--replay",
        "--replay-db",
        str(db),
        "--reformraytracingvoxelmap.dump-path",
        str(files["tracer"]),
        "--mlsreformplannernative.dump-path",
        str(files["planner"]),
        "--gscpgo.dump-path",
        str(files["keyframes"]),
    ]
    if not closures:
        command += [
            "--reformraytracingvoxelmap.reform-min-shift-m",
            NO_CLOSURES,
            "--mlsreformplannernative.reform-min-shift-m",
            NO_CLOSURES,
        ]
    print(f"{name} {mode}: {span / 60:.1f} min of recording", flush=True)
    launched = time.time()
    with files["log"].open("w") as log, files["rss"].open("w") as rss:
        proc = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
        started = None
        try:
            while proc.poll() is None:
                found = native_processes()
                if started is None and "mls_reform_planner" in found:
                    started = time.monotonic()
                sample = {"t": time.time()}
                for module, native in found.items():
                    try:
                        sample[module] = native.memory_info().rss / 1e6
                    except psutil.Error:
                        pass
                rss.write(json.dumps(sample) + "\n")
                rss.flush()
                if started is not None and time.monotonic() - started > span + SETTLE_S:
                    proc.send_signal(signal.SIGINT)
                    proc.wait(timeout=120)
                    break
                time.sleep(RSS_PERIOD_S)
        finally:
            if proc.poll() is None:
                proc.kill()
    run_log = newest_run_log(launched)
    if run_log is not None:
        files["jsonl"].write_bytes(run_log.read_bytes())
    print(f"{name} {mode}: exit {proc.returncode}, log {files['jsonl']}", flush=True)


def events(closures: Path, out: Path) -> int:
    """Turn GSC's closure dump into the event dump reform_against_fast_reform reads."""
    raw = closures.read_bytes()
    chunks, at = [], 0
    while at < len(raw):
        ts = np.frombuffer(raw, "<f8", 1, at)
        n = int(np.frombuffer(raw, "<u4", 1, at + 8)[0])
        records = np.frombuffer(raw, CLOSURE_RECORD, n, at + 12)
        at += 12 + n * CLOSURE_RECORD.itemsize
        delta = Rotation.from_quat(records["dq"].astype(float))
        out_records = np.zeros(n, EVENT_RECORD)
        out_records["id"], out_records["ts"] = records["id"], records["ts"]
        out_records["pre"][:, :3], out_records["pre"][:, 3:] = records["xyz"], records["q"]
        out_records["post"][:, :3] = delta.apply(records["xyz"].astype(float)) + records["dt"]
        out_records["post"][:, 3:] = (
            delta * Rotation.from_quat(records["q"].astype(float))
        ).as_quat()
        out_records["local"] = out_records["pre"]
        chunks += [ts.tobytes(), np.uint32(n).tobytes(), out_records.tobytes()]
    out.write_bytes(b"".join(chunks))
    return len(chunks) // 3


def load_points(path: Path) -> np.ndarray:
    return np.fromfile(path, "<f4").reshape(-1, 3)


def rebuild(db: Path, keyframes: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """The ray tracer's map and the planner's surface, from scratch, every scan placed by the
    final correction of the latest keyframe at or before it."""
    rt = ray_tracing_config
    mapper = VoxelRayMapper(
        voxel_size=rt.voxel_size,
        max_range=rt.max_range,
        fine_divisor=rt.fine_divisor,
        ray_subsample=rt.ray_subsample,
        shadow_depth=rt.shadow_depth,
        grace_depth=rt.grace_depth,
        min_health=rt.min_health,
        max_health=rt.max_health,
        graze_cos=rt.graze_cos,
        support_min=rt.support_min,
        region_percentile=rt.region_percentile,
        worker_threads=rt.worker_threads,
    )
    raw_rotation = Rotation.from_quat(keyframes["raw"][:, 3:])
    corrected_rotation = Rotation.from_quat(keyframes["corrected"][:, 3:])
    correction_rotation = corrected_rotation * raw_rotation.inv()
    correction_translation = keyframes["corrected"][:, :3] - correction_rotation.apply(
        keyframes["raw"][:, :3]
    )
    with SqliteStore(path=str(db)) as store:
        for obs in store.streams.lidar.order_by("ts"):
            k = max(int(np.searchsorted(keyframes["ts"], obs.ts, side="right")) - 1, 0)
            rotation, translation = correction_rotation[k], correction_translation[k]
            points = np.asarray(obs.data.points_f32(), np.float64)
            if len(points) == 0 or obs.pose is None:
                continue
            p = obs.pose.position
            origin = rotation.apply([p.x, p.y, p.z]) + translation
            corrected = (rotation.apply(points) + translation).astype(np.float32)
            mapper.add_frame_world(corrected, tuple(map(float, origin)))
    tracer = np.asarray(mapper.global_map(), np.float32)
    cfg = mls_planner_config
    planner = MLSPlanner(
        voxel_size=cfg.voxel_size,
        robot_height=cfg.robot_height,
        max_overhead_m=cfg.max_overhead_m,
        surface_closing_radius=cfg.surface_closing_radius,
        node_spacing_m=cfg.node_spacing_m,
        wall_clearance_m=cfg.wall_clearance_m,
        wall_buffer_m=cfg.wall_buffer_m,
        wall_buffer_weight=cfg.wall_buffer_weight,
        step_threshold_m=cfg.step_threshold_m,
        step_penalty_weight=cfg.step_penalty_weight,
        worker_threads=cfg.worker_threads,
    )
    planner.update_global_map(tracer)
    return tracer, np.asarray(planner.surface_map(), np.float32)


def agreement(rebuilt: np.ndarray, reformed: np.ndarray, voxel: float) -> dict[str, float]:
    """For each rebuilt voxel, the distance to the nearest reformed one."""
    distances, _ = cKDTree(reformed).query(rebuilt, workers=2)
    return {
        "rebuilt": len(rebuilt),
        "reformed": len(reformed),
        "median_offset_voxels": float(np.median(distances) / voxel),
        "within_1_voxel": float(np.mean(distances <= voxel * 1.001)),
    }


def closure_timings(log: Path) -> dict[str, dict[str, float]]:
    """Per-closure planner and ray tracer times from the run's debug lines, with the rebuild count."""
    times: dict[str, list[float]] = {"planner": [], "tracer": []}
    rebuilds = {"planner": 0, "tracer": 0}
    for line in log.read_text(errors="replace").splitlines():
        try:
            entry = json.loads(line)
        except json.JSONDecodeError:
            continue
        event = entry.get("event", "")
        if event.startswith("Loop closure applied in place"):
            module, ms = "planner", float(entry["total_ms"])
        elif event.startswith("Warped the map through a loop closure"):
            module, ms = "tracer", float(entry["warp_ms"]) + float(entry["voxel_ms"])
        else:
            continue
        times[module].append(ms)
        rebuilds[module] += entry.get("decision") == "rebuild"
    out = {}
    for module, values in times.items():
        v = np.array([x for x in values if np.isfinite(x)])
        out[module] = {
            "closures": len(v),
            "p50_ms": float(np.percentile(v, 50)) if len(v) else 0.0,
            "p90_ms": float(np.percentile(v, 90)) if len(v) else 0.0,
            "max_ms": float(v.max()) if len(v) else 0.0,
            "rebuilds": rebuilds[module],
        }
    return out


def peak_rss(rss: Path) -> dict[str, float]:
    peaks: dict[str, float] = {}
    for line in rss.read_text().splitlines():
        sample = json.loads(line)
        for module in NATIVE:
            if module in sample:
                peaks[module] = max(peaks.get(module, 0.0), sample[module])
    return peaks


def score(db: Path, name: str, out: Path) -> dict:
    voxel = ray_tracing_config.voxel_size
    on, off = paths(out, name, "closures"), paths(out, name, "no_closures")
    keyframes = np.fromfile(on["keyframes"], KEYFRAME)
    tracer, surface = rebuild(db, keyframes)
    planner_voxels = {
        mode: len(load_points(Path(f"{p['planner']}.voxels")))
        for mode, p in (("on", on), ("off", off))
    }
    metrics = {
        "recording": name,
        "keyframes": len(keyframes),
        "peak_rss_mb": {"closures": peak_rss(on["rss"]), "no_closures": peak_rss(off["rss"])},
        "planner_voxels": {
            "closures": planner_voxels["on"],
            "no_closures": planner_voxels["off"],
            "ratio": planner_voxels["on"] / max(planner_voxels["off"], 1),
        },
        "tracer_vs_rebuild": agreement(tracer, load_points(on["tracer"]), voxel),
        "surface_vs_rebuild": agreement(
            surface, load_points(Path(f"{on['planner']}.surface")), voxel
        ),
        "closure_ms": closure_timings(on["jsonl"]),
    }
    (out / f"{name}.metrics.json").write_text(json.dumps(metrics, indent=2))
    print(json.dumps(metrics, indent=2), flush=True)
    return metrics


def main() -> None:
    parser = argparse.ArgumentParser(description=(__doc__ or "").split("\n\n")[0])
    parser.add_argument("command", choices=("run", "score", "events"))
    parser.add_argument("--db", type=Path, required=True)
    parser.add_argument("--name", required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--no-closures", action="store_true")
    args = parser.parse_args()
    if args.command == "run":
        run(args.db, args.name, args.out, closures=not args.no_closures)
    elif args.command == "events":
        closures = Path(f"{paths(args.out, args.name, 'closures')['keyframes']}.closures")
        out = args.out / f"{args.name}.events.bin"
        print(f"{events(closures, out)} closures -> {out}")
    else:
        score(args.db, args.name, args.out)


if __name__ == "__main__":
    main()
