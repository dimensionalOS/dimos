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

"""FRANK's rolling eyes: recognize faces continuously and keep the server's world state fresh.

    uv run python dimos/experimental/frank/tools/watch.py                  # camera from the running DimOS instance
    uv run python dimos/experimental/frank/tools/watch.py --source webcam  # laptop webcam, no robot needed
    uv run python dimos/experimental/frank/tools/watch.py --fps 4 --show   # preview window with boxes and names

Long-running. Reads a few frames a second, matches every face against the enrolled gallery, and
posts one sighting per person per second to `POST /agent/sightings` with where that person is:

    on the robot   the face box, widened to a body box, is projected through the Go2 camera model
                   onto the last two seconds of lidar (DimOS `Detection3DPC.from_2d`); the median of
                   the points inside is the person's world x, y. Bearing and range follow from that
                   and the `odom` pose. Falls back to the estimate below when no lidar points land.
    fallback       bearing from the face's pixel column and the intrinsics (fx, cx); range from face
                   width against a 16 cm head, so ±30% at best; rotated into the world by `odom`.

It only ever reads. Nothing here moves the robot.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys
import threading
import time
from typing import Any

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import identify  # sibling scripts, after the sys.path insert above
import inbox

HEAD_WIDTH_M = 0.16  # a face's outer width; the source of the ±30% fallback range error
LIDAR_KEEP_S = 2.0  # how much recent lidar to project a body box onto
LIDAR_MIN_POINTS = 8  # fewer than this inside the body box and we fall back to face width
ENROLL_EVERY_S = 5.0  # how often to embed selfies of newly enrolled people
ROBOT_HFOV_DEG = 120.0  # fallback when the robot publishes no intrinsics (Go2 wide angle)
WEBCAM_HFOV_DEG = 70.0  # typical laptop webcam; override with --hfov
POST_EVERY_S = 1.0  # at most one sighting per person per second
LEFT_VIEW_AFTER_S = 3.0  # no match for this long -> post the explicit "left view"


class Source:
    """Frames, plus whatever the source knows about the camera and the robot's pose."""

    hfov_deg = ROBOT_HFOV_DEG  # used only when no intrinsics are available

    def read(self) -> np.ndarray | None:
        raise NotImplementedError

    def intrinsics(self, shape: tuple[int, ...]) -> tuple[float, float]:
        """(fx, cx) in pixels."""
        w = shape[1]
        return w / (2 * math.tan(math.radians(self.hfov_deg) / 2)), w / 2

    def pose(self) -> tuple[float, float, float]:
        """Robot x, y (m) and yaw (rad) in the world frame."""
        return 0.0, 0.0, 0.0

    def locate3d(
        self, bbox: list[int], shape: tuple[int, ...], ts: float
    ) -> dict[str, float] | None:
        """Where a face is from real depth, if the source has any. None means use `locate()`."""
        return None

    def close(self) -> None:
        pass


class Webcam(Source):
    hfov_deg = WEBCAM_HFOV_DEG

    def __init__(self, index: int = 0) -> None:
        self.cap = cv2.VideoCapture(index)
        if not self.cap.isOpened():
            raise SystemExit(f"cannot open webcam {index}")

    def read(self) -> np.ndarray | None:
        ok, frame = self.cap.read()
        return frame if ok else None

    def close(self) -> None:
        self.cap.release()


def _connect_with_patience(total_s: float = 120.0) -> Any:
    """`Dimos.connect()` right after `dimos run` fails until the coordinator is on the bus;
    up.py starts us in that window, so keep trying instead of dying."""
    from dimos.porcelain.dimos import Dimos  # heavy import, kept where the robot source needs it

    t0 = time.time()
    while True:
        try:
            return Dimos.connect(timeout=10.0)
        except RuntimeError:
            if time.time() - t0 > total_s:
                raise
            print("no coordinator yet; retrying", flush=True)
            time.sleep(5)


class RobotCamera(Source):
    """`color_image` off the running DimOS instance, with `camera_info` and `odom` alongside.

    `Dimos.connect()` has no subscribe path, only `peek_stream`, which blocks until the stream's
    next emission — so peeking in a loop *is* the subscription, at the stream's own rate.
    """

    def __init__(self) -> None:
        # local import: pulling in dimos is slow, and a webcam-only run never needs it

        self.app = _connect_with_patience()
        self._k: tuple[float, float] | None = None
        self._pose = (0.0, 0.0, 0.0)
        self._odom: Any = None
        self._lidar: list[
            tuple[float, Any]
        ] = []  # (arrival time, PointCloud2 chunk), last LIDAR_KEEP_S
        self._lidar_lock = threading.Lock()
        self._lidar_frame_warned = False
        threading.Thread(target=self._pump_lidar, daemon=True).start()

    def _pump_lidar(self) -> None:
        """The Go2 sends lidar as small chunks; keep the recent ones so a body box has points in it."""
        while True:
            try:
                pc = self.app.peek_stream("lidar", timeout=2.0)
            except Exception:
                time.sleep(0.2)
                continue
            if pc is None:
                continue
            now = time.monotonic()
            with self._lidar_lock:
                self._lidar.append((now, pc))
                self._lidar = [(t, c) for t, c in self._lidar if now - t < LIDAR_KEEP_S]

    def _recent_lidar(self) -> Any:
        with self._lidar_lock:
            chunks = [c for _, c in self._lidar]
        if not chunks:
            return None
        pc = chunks[0]
        for c in chunks[1:]:
            pc = pc + c
        return pc

    def locate3d(
        self, bbox: list[int], shape: tuple[int, ...], ts: float
    ) -> dict[str, float] | None:
        from dimos.msgs.geometry_msgs.Transform import Transform
        from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
        from dimos.perception.detection.type.detection3d.pointcloud import Detection3DPC
        from dimos.robot.unitree.go2.connection import BASE_TO_OPTICAL, _camera_info_static

        pc = self._recent_lidar()
        if pc is None or self._odom is None:
            return None
        if pc.frame_id != "world":
            if not self._lidar_frame_warned:
                self._lidar_frame_warned = True
                print(
                    f"lidar is in frame {pc.frame_id!r}, not 'world'; using face-width ranges",
                    flush=True,
                )
            return None
        # a face is a few lidar points at best; the body under it is many. Widen to a body box.
        x1, y1, x2, y2 = bbox
        w, h = x2 - x1, y2 - y1
        body = (
            max(0.0, x1 - 1.0 * w),
            max(0.0, y1 - 0.5 * h),
            min(float(shape[1]), x2 + 1.0 * w),
            min(float(shape[0]), y2 + 4.0 * h),
        )
        det = Detection2DBBox(
            bbox=body, track_id=0, class_id=0, confidence=1.0, name="person", ts=ts, image=None
        )  # type: ignore[arg-type]
        world_to_optical = -(Transform.from_pose("base_link", self._odom) + BASE_TO_OPTICAL)
        d3 = Detection3DPC.from_2d(det, pc, _camera_info_static(), world_to_optical, filters=[])
        if d3 is None:
            return None
        pts, _ = d3.pointcloud.as_numpy()
        if len(pts) < LIDAR_MIN_POINTS:
            return None
        rx, ry, yaw = self._pose
        # the box also catches whatever is behind the person, so keep the nearest cluster only
        rel = pts[:, :2] - np.array([rx, ry])
        ranges = np.hypot(rel[:, 0], rel[:, 1])
        near = ranges < np.percentile(ranges, 25) + 0.6
        rel, ranges = rel[near], ranges[near]
        bearings = np.arctan2(rel[:, 1], rel[:, 0]) - yaw
        rng = float(np.median(ranges))
        bearing = float(math.atan2(np.median(np.sin(bearings)), np.median(np.cos(bearings))))
        return {
            "bearing_deg": round(math.degrees(bearing), 1),
            "range_m": round(rng, 2),
            "x": round(rx + rng * math.cos(yaw + bearing), 2),
            "y": round(ry + rng * math.sin(yaw + bearing), 2),
            "lidar_points": int(near.sum()),
        }

    def read(self) -> np.ndarray | None:
        img = self.app.peek_stream("color_image", timeout=2.0)
        if img is None:
            return None
        data = getattr(img, "data", img)
        frame = np.asarray(data)
        if frame.ndim == 3 and frame.shape[2] == 3 and getattr(img, "encoding", "bgr8") == "rgb8":
            frame = frame[:, :, ::-1]
        return np.ascontiguousarray(frame)

    def intrinsics(self, shape: tuple[int, ...]) -> tuple[float, float]:
        if self._k is None:
            try:
                info = self.app.peek_stream("camera_info", timeout=2.0)
                k = list(getattr(info, "k", None) or info.K)
                self._k = (float(k[0]), float(k[2]))
            except Exception:
                self._k = super().intrinsics(shape)  # no intrinsics published: assume wide angle
        return self._k

    def pose(self) -> tuple[float, float, float]:
        try:
            od = self.app.peek_stream("odom", timeout=2.0)
            yaw = float(od.yaw)
            self._odom = od
            self._pose = (float(od.position.x), float(od.position.y), yaw)
        except Exception:
            pass  # keep the last pose rather than teleporting the world to the origin
        return self._pose

    def close(self) -> None:
        self.app.stop()


def locate(
    bbox: list[int], fx: float, cx: float, pose: tuple[float, float, float]
) -> dict[str, float]:
    """Where a face is: bearing (deg, positive = FRANK's left), range (m), and world x, y."""
    x1, _, x2, _ = bbox
    face_px = max(1.0, float(x2 - x1))
    bearing = math.atan2(cx - (x1 + x2) / 2.0, fx)  # left of centre = positive
    rng = fx * HEAD_WIDTH_M / face_px
    rx, ry, yaw = pose
    return {
        "bearing_deg": round(math.degrees(bearing), 1),
        "range_m": round(rng, 2),
        "x": round(rx + rng * math.cos(yaw + bearing), 2),
        "y": round(ry + rng * math.sin(yaw + bearing), 2),
    }


class Poster:
    """One sighting per person per second, one "left view" when they go, one log line per change."""

    def __init__(self, quiet: bool = False) -> None:
        self.last_post: dict[str, float] = {}
        self.last_seen: dict[str, float] = {}
        self.present: set[str] = set()
        self.names: dict[str, str] = {}
        self.quiet = quiet

    def log(self, msg: str) -> None:
        if not self.quiet:
            print(f"{time.strftime('%H:%M:%S')} {msg}", flush=True)

    def _post(self, person_id: str, confidence: float, **fields: Any) -> dict[str, Any] | None:
        try:
            return inbox.sighting(person_id, confidence, **fields)
        except inbox.FrankError as exc:
            self.log(f"server: {exc}")
            return None

    def saw(
        self, person_id: str, name: str, confidence: float, where: dict[str, float], face_px: int
    ) -> None:
        now = time.time()
        self.names[person_id] = name
        self.last_seen[person_id] = now
        fresh = person_id not in self.present
        if not fresh and now - self.last_post.get(person_id, 0.0) < POST_EVERY_S:
            return
        self.present.add(person_id)
        got = self._post(
            person_id,
            round(confidence, 3),
            # `pose` is where the *person* is, so a later `greet_known` wake points at them
            pose=(where["x"], where["y"], 0.0),
            face_px=face_px,
            in_view=True,
            **where,
        )
        self.last_post[person_id] = now
        if fresh:
            self.log(
                f"{name} entered view {where['range_m']:.1f} m "
                f"{inbox.bearing_word(where['bearing_deg'])}"
                + (f" ({got['event']})" if got and got.get("event") else "")
            )

    def sweep(self) -> None:
        now = time.time()
        for person_id in sorted(self.present):
            if now - self.last_seen.get(person_id, 0.0) < LEFT_VIEW_AFTER_S:
                continue
            self.present.discard(person_id)
            self._post(person_id, 0.0, in_view=False)
            self.log(f"{self.names.get(person_id, person_id)} left view")


def draw(frame: np.ndarray, faces: list[identify.Face], where: dict[int, dict[str, float]]) -> None:
    for i, f in enumerate(faces):
        x1, y1, x2, y2 = f.bbox
        known = f.person_id is not None
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 200, 0) if known else (0, 140, 255), 2)
        label = f.name or "?"
        if i in where:
            label += f"  {where[i]['range_m']:.1f}m {where[i]['bearing_deg']:+.0f}deg"
        cv2.putText(
            frame, label, (x1, max(16, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1
        )


def run(args: argparse.Namespace) -> int:
    source: Source = Webcam(args.device) if args.source == "webcam" else RobotCamera()
    if args.hfov:
        source.hfov_deg = args.hfov
    rec = identify.Recognizer()
    db = identify.load_db()
    if not db["people"]:
        print(
            "nobody enrolled yet — run `identify.py enroll` after someone signs up", file=sys.stderr
        )
    poster = Poster(quiet=args.quiet)
    period = 1.0 / max(0.5, args.fps)
    db_mtime = identify.EMBED_PATH.stat().st_mtime if identify.EMBED_PATH.exists() else 0.0
    last_enroll = 0.0
    print(
        f"watching ({args.source}) at {args.fps} fps, {len(db['people'])} enrolled. ctrl-c to stop.",
        flush=True,
    )

    try:
        while True:
            t0 = time.monotonic()
            frame = source.read()
            if frame is None:
                time.sleep(period)
                continue

            # someone can enroll on their phone while we run: embed any new selfie every few
            # seconds (cheap when nothing changed), then pick the new gallery up without a restart
            if time.monotonic() - last_enroll > ENROLL_EVERY_S:
                last_enroll = time.monotonic()
                try:
                    rep = identify.enroll()
                    for pid in rep.get("added", []):
                        print(f"{time.strftime('%H:%M:%S')} enrolled face for {pid}", flush=True)
                    for pid, msg in (rep.get("failed") or {}).items():
                        print(
                            f"{time.strftime('%H:%M:%S')} could not enroll {pid}: {msg}", flush=True
                        )
                except Exception as e:
                    print(f"enroll pass failed: {e}", flush=True)
            if identify.EMBED_PATH.exists() and identify.EMBED_PATH.stat().st_mtime != db_mtime:
                db_mtime = identify.EMBED_PATH.stat().st_mtime
                db = identify.load_db()

            faces = identify.identify_image(frame, rec=rec, db=db, threshold=args.threshold)
            fx, cx = source.intrinsics(frame.shape)
            pose = source.pose()
            shown: dict[int, dict[str, float]] = {}
            for i, f in enumerate(faces):
                if not f.person_id:
                    continue
                where = source.locate3d(f.bbox, frame.shape, time.time()) or locate(
                    f.bbox, fx, cx, pose
                )
                shown[i] = where
                poster.saw(f.person_id, f.name or f.person_id, f.confidence, where, f.face_px)
            poster.sweep()

            if args.show:
                draw(frame, faces, shown)
                cv2.imshow("FRANK watcher", frame)
                if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                    break

            time.sleep(max(0.0, period - (time.monotonic() - t0)))
    except KeyboardInterrupt:
        pass
    finally:
        source.close()
        if args.show:
            cv2.destroyAllWindows()
    return 0


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("--source", choices=["robot", "webcam"], default="robot")
    p.add_argument("--device", type=int, default=0, help="webcam index")
    p.add_argument(
        "--hfov",
        type=float,
        help="horizontal field of view in degrees when no intrinsics (webcam 70, robot 120)",
    )
    p.add_argument("--fps", type=float, default=3.0, help="frames processed per second")
    p.add_argument("--threshold", type=float, default=identify.THRESHOLD)
    p.add_argument("--show", action="store_true", help="preview window with boxes and names")
    p.add_argument("--quiet", action="store_true", help="no state-change lines on stdout")
    return run(p.parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
