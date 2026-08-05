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

"""Run ORB-SLAM3 on a memory2 recording and store the trajectory beside it.

    python -m dimos.mapping.orbslam3_runner <recording> [--modes stereo,stereo_inertial]

ORB-SLAM3 reads the EuRoC layout (mav0/cam{0,1}/data/<ts_ns>.png + mav0/imu0/data.csv)
and nothing else, so the recording is flattened to that on the way in. The PNGs are
bulky -- roughly 2 GB per recording -- so the staging directory is removed once the
trajectory has been captured unless --keep is passed.

The IR pair is delivered already rectified by the device, which is why the settings use
Camera.type "Rectified" with a plain baseline and no distortion coefficients.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

import cv2
import numpy as np

from dimos.memory2.replay import Replay
from dimos.memory2.store.sqlite import SqliteStore
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

DATASETS = Path.home() / "datasets" / "d455"
ORB_ROOT = Path.home() / "repos" / "ORB_SLAM3"
VOCABULARY = ORB_ROOT / "Vocabulary" / "ORBvoc.txt"
NS_PER_S = 1_000_000_000
# Kalibr T_cam_imu for this rig, and the camera/IMU time offset it solved for.
KALIBR_T_CAM_IMU = np.array(
    [
        [0.9999569910181326, 0.006734352892450546, -0.006376880513342993, 0.030829037333556044],
        [-0.006749594780694915, 0.999974408718808, -0.002371682475734932, -0.004349223866105144],
        [0.006360745574060212, 0.0024146218315163133, 0.9999768549907303, -0.015419659057578911],
        [0.0, 0.0, 0.0, 1.0],
    ]
)
KALIBR_TIMESHIFT_CAM_IMU_S = -0.03291138284568801
# Kalibr's sheet for this unit; the D455 defaults are far looser than measured.
IMU_NOISE_GYRO = 2.44e-4
IMU_NOISE_ACC = 1.862e-3
IMU_GYRO_WALK = 1.9e-5
IMU_ACC_WALK = 3.0e-3
IMU_RATE_HZ = 400.0
# 40 (the D435i default) leaves nearly every outdoor point classified as far, which
# drives the pose to NaN and aborts inside Sophus on the outdoor sequence. 60 with a
# larger feature budget tracks all six recordings under one setting.
STEREO_TH_DEPTH = 60.0
ORB_FEATURES = 2000

MODES = {
    "stereo": ("Examples/Stereo/stereo_euroc", False),
    "stereo_inertial": ("Examples/Stereo-Inertial/stereo_inertial_euroc", True),
}


def stage_euroc(db_path: Path, out_dir: Path) -> dict[str, object]:
    """Flatten a recording into the EuRoC layout ORB-SLAM3 expects."""
    replay = Replay(store=SqliteStore(path=str(db_path)))
    cam0 = out_dir / "mav0" / "cam0" / "data"
    cam1 = out_dir / "mav0" / "cam1" / "data"
    for directory in (cam0, cam1):
        directory.mkdir(parents=True, exist_ok=True)
    (out_dir / "mav0" / "imu0").mkdir(parents=True, exist_ok=True)

    info = next(iter(replay.stream("realsense_infra_left_camera_info").iterate_ts()))[1]
    right_info = next(iter(replay.stream("realsense_infra_right_camera_info").iterate_ts()))[1]
    baseline_m = -right_info.P[3] / right_info.P[0] if right_info.P[0] else 0.0

    right_by_ts = {
        round(message.ts, 4): message
        for _ts, message in replay.stream("realsense_infra_right").iterate_ts()
    }
    right_stamps = np.array(sorted(right_by_ts))

    stamps: list[int] = []
    unpaired = 0
    for _ts, left in replay.stream("realsense_infra_left").iterate_ts():
        nearest = right_stamps[np.argmin(np.abs(right_stamps - left.ts))]
        right = right_by_ts[nearest]
        if abs(right.ts - left.ts) > 0.001:
            unpaired += 1
            continue
        stamp_ns = int(left.ts * NS_PER_S)
        cv2.imwrite(str(cam0 / f"{stamp_ns}.png"), left.data)
        cv2.imwrite(str(cam1 / f"{stamp_ns}.png"), right.data)
        stamps.append(stamp_ns)

    (out_dir / "times.txt").write_text("".join(f"{stamp}\n" for stamp in stamps))

    imu_rows = 0
    with (out_dir / "mav0" / "imu0" / "data.csv").open("w") as handle:
        handle.write("#timestamp [ns],w_x,w_y,w_z,a_x,a_y,a_z\n")
        for _ts, sample in replay.stream("realsense_imu").iterate_ts():
            angular, linear = sample.angular_velocity, sample.linear_acceleration
            stamp_ns = int((sample.ts + KALIBR_TIMESHIFT_CAM_IMU_S) * NS_PER_S)
            handle.write(
                f"{stamp_ns},{angular.x},{angular.y},{angular.z},{linear.x},{linear.y},{linear.z}\n"
            )
            imu_rows += 1

    return {
        "frames": len(stamps),
        "unpaired": unpaired,
        "imu_samples": imu_rows,
        "width": int(info.width),
        "height": int(info.height),
        "fx": float(info.K[0]),
        "fy": float(info.K[4]),
        "cx": float(info.K[2]),
        "cy": float(info.K[5]),
        "baseline_m": float(baseline_m),
    }


def write_settings(path: Path, manifest: dict[str, object]) -> None:
    # ORB-SLAM3's IMU.T_b_c1 is the camera pose in the IMU (body) frame, the inverse of
    # what Kalibr reports.
    t_body_cam = np.linalg.inv(KALIBR_T_CAM_IMU)
    rows = ",\n          ".join(", ".join(f"{value:.9f}" for value in row) for row in t_body_cam)
    path.write_text(
        f"""%YAML:1.0
File.version: "1.0"

Camera.type: "Rectified"
Camera1.fx: {manifest["fx"]}
Camera1.fy: {manifest["fy"]}
Camera1.cx: {manifest["cx"]}
Camera1.cy: {manifest["cy"]}
Stereo.b: {manifest["baseline_m"]}
Camera.width: {manifest["width"]}
Camera.height: {manifest["height"]}
Camera.fps: 30
Camera.RGB: 1
Stereo.ThDepth: {STEREO_TH_DEPTH}

IMU.T_b_c1: !!opencv-matrix
   rows: 4
   cols: 4
   dt: f
   data: [{rows}]

IMU.InsertKFsWhenLost: 0
IMU.NoiseGyro: {IMU_NOISE_GYRO}
IMU.NoiseAcc: {IMU_NOISE_ACC}
IMU.GyroWalk: {IMU_GYRO_WALK}
IMU.AccWalk: {IMU_ACC_WALK}
IMU.Frequency: {IMU_RATE_HZ}

ORBextractor.nFeatures: {ORB_FEATURES}
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 8
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7

Viewer.KeyFrameSize: 0.05
Viewer.KeyFrameLineWidth: 1.0
Viewer.GraphLineWidth: 0.9
Viewer.PointSize: 2.0
Viewer.CameraSize: 0.08
Viewer.CameraLineWidth: 3.0
Viewer.ViewpointX: 0.0
Viewer.ViewpointY: -0.7
Viewer.ViewpointZ: -3.5
Viewer.ViewpointF: 500.0
"""
    )


def run_mode(
    mode: str, stage: Path, settings: Path, work: Path
) -> tuple[dict[str, object], Path | None]:
    binary, _ = MODES[mode]
    label = f"{mode}_run"
    command = [
        str(ORB_ROOT / binary),
        str(VOCABULARY),
        str(settings),
        str(stage),
        str(stage / "times.txt"),
        label,
    ]
    environment = dict(os.environ)
    environment["LD_LIBRARY_PATH"] = os.pathsep.join(
        [
            str(Path.home() / ".local/lib"),
            str(ORB_ROOT / "lib"),
            str(ORB_ROOT / "Thirdparty/DBoW2/lib"),
            str(ORB_ROOT / "Thirdparty/g2o/lib"),
            environment.get("LD_LIBRARY_PATH", ""),
        ]
    )
    started = time.monotonic()
    completed = subprocess.run(
        command,
        cwd=work,
        capture_output=True,
        text=True,
        timeout=7200,
        check=False,
        env=environment,
    )
    elapsed = time.monotonic() - started
    produced = work / f"f_{label}.txt"
    outcome: dict[str, object] = {
        "mode": mode,
        "returncode": completed.returncode,
        "wall_s": round(elapsed, 1),
        "stderr_tail": completed.stderr[-1500:],
        "stdout_tail": completed.stdout[-800:],
    }
    return outcome, produced if produced.exists() else None


def to_npy(tum_path: Path, destination: Path) -> int:
    """Convert ORB-SLAM3's EuRoC/TUM output to the Nx4 (t, x, y, z) the plots expect."""
    raw = np.loadtxt(tum_path)
    if raw.ndim == 1:
        raw = raw.reshape(1, -1)
    seconds = raw[:, 0]
    # SaveTrajectoryEuRoC writes nanoseconds for EuRoC-style input.
    if seconds.max() > 1e12:
        seconds = seconds / NS_PER_S
    trajectory = np.column_stack([seconds - seconds[0], raw[:, 1:4]])
    np.save(destination, trajectory)
    return len(trajectory)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording")
    parser.add_argument("--modes", default="stereo,stereo_inertial")
    parser.add_argument("--stage-root", type=Path, default=Path("/tmp/orb_stage"))
    parser.add_argument("--keep", action="store_true")
    args = parser.parse_args()

    recording = args.recording
    recording_dir = DATASETS / recording
    db_path = recording_dir / f"{recording}.db"
    if not db_path.exists():
        logger.error("%s does not exist", db_path)
        return 1

    stage = args.stage_root / recording
    work = stage.parent / f"{recording}_work"
    work.mkdir(parents=True, exist_ok=True)

    if not (stage / "times.txt").exists():
        logger.info("staging %s to EuRoC layout", recording)
        manifest = stage_euroc(db_path, stage)
        (stage / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    else:
        manifest = json.loads((stage / "manifest.json").read_text())
    logger.info(
        "%s: %s frames, %s imu samples", recording, manifest["frames"], manifest["imu_samples"]
    )

    settings = stage / "orbslam3.yaml"
    write_settings(settings, manifest)

    results = []
    for mode in args.modes.split(","):
        logger.info("running ORB-SLAM3 %s on %s", mode, recording)
        outcome, trajectory = run_mode(mode, stage, settings, work)
        if trajectory is None:
            logger.error(
                "%s %s produced no trajectory (rc=%s)", recording, mode, outcome["returncode"]
            )
        else:
            destination = recording_dir / f"orbslam3_{mode}_traj.npy"
            outcome["poses"] = to_npy(trajectory, destination)
            outcome["saved"] = str(destination)
            logger.info("%s %s -> %s poses", recording, mode, outcome["poses"])
        results.append(outcome)

    (recording_dir / "orbslam3_stats.json").write_text(json.dumps(results, indent=2) + "\n")
    print(json.dumps(results, indent=2))

    if not args.keep:
        shutil.rmtree(stage, ignore_errors=True)
        shutil.rmtree(work, ignore_errors=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
