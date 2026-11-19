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

"""Find the stack bins on the kitting bench with a fixed camera calibrated to the arm.

Green pixels with depth are lifted into the arm's base frame through the camera's hand-eye
result (``STATE_DIR/calibration/<calibration>.json``, written by the xarm7 scene calibration),
grouped on a 1 cm grid, and each group is fitted with a bin: footprint centre, long axis, and
which end is the tall back wall. ``load_from_scan`` and ``cycle_from_scan`` tell the kitting
module which slots hold a bin and hand it the empty bin in the middle zone, and
``unload_from_scan`` carries the bin in the loading slot to a done slot no bin is in, so the cycle
runs on the camera's numbers instead of a tape measure's.
"""

from __future__ import annotations

import json
import threading
import time
from typing import Any, Protocol

import cv2
import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

from dimos.constants import STATE_DIR
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.robot.manipulators.xarm.kitting import (
    BIN_HEIGHT_M,
    BIN_LENGTH_M,
    BIN_WIDTH_M,
    XArm7KittingConfig,
)
from dimos.spec.utils import Spec
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

FloatArray = NDArray[np.float64]

GRID_M = 0.01
# A bin seen from the side shows its near wall and the rims; fewer cells is a stray green thing.
MIN_CELLS = 40
# The rim: points above this fraction of the bin's height. The end of the rim a quarter of the
# length long with more points is the back wall; the scoop front is low.
RIM_FRACTION = 0.75
MIN_RIM_POINTS = 50
MAX_LIP_M = 0.02
# Depth noise at a metre sprays a few rim points centimetres past the side walls, and each fused
# frame adds more: the rectangle is fitted to the rim less this share on each side, across it.
WIDTH_TRIM_PCT = 2.0
# A done slot is taken when a seen bin comes within this of the slot's footprint. The done slots
# leave 26 mm between bins, so a neighbour standing on its own slot does not take this one.
SLOT_CLEAR_M = 0.015
# The unload grips the bin where the loading slot says it stands; further off than this, it would
# miss the wall.
LOADING_TOL_M = 0.025
LOADING_TOL_DEG = 8.0
END_FRACTION = 0.5
# Measured footprint further than this from the bin's own size is reported as suspect.
SIZE_TOL_M = 0.04


class KittingSpec(Spec, Protocol):
    def load(self, x: float, y: float, yaw_deg: float) -> str: ...
    def run_cycle(self, x: float, y: float, yaw_deg: float) -> str: ...
    def unload(self) -> str: ...
    def set_done_taken(self, slots: list[int]) -> None: ...
    def set_loading_full(self, full: bool) -> None: ...


class XArm7BinFinderConfig(ModuleConfig):
    # The hand-eye result naming the camera's optical frame in the arm's base.
    calibration: str = "xarm7_scene_camera"
    table_top_z: float = XArm7KittingConfig().table_top_z
    # Only points this far above the table and inside the bench are kept.
    min_height_m: float = 0.012
    bench_x_m: tuple[float, float] = (-0.05, 0.80)
    bench_y_m: tuple[float, float] = (-0.75, 0.95)
    # Bins of the kitting bench: OpenCV HSV (hue 0-180), and the depth the camera trusts.
    hsv_lo: tuple[int, int, int] = (40, 70, 35)
    hsv_hi: tuple[int, int, int] = (95, 255, 255)
    depth_range_m: tuple[float, float] = (0.2, 2.5)
    # Zones by footprint centre: the loading slot past loading_min_y, the done zone below
    # done_max_y, empty bins in between.
    loading_min_y: float = 0.30
    done_max_y: float = -0.30
    # Frames fused per scan, so one noisy depth frame does not move a bin.
    scan_frames: int = 5
    scan_timeout_s: float = 5.0


class XArm7BinFinder(Module):
    """Scans the bench for bins with a calibrated fixed camera. ``find_bins`` reports every bin
    with its zone and pose; ``load_from_scan`` and ``cycle_from_scan`` start the kitting module
    on the empty bin it found."""

    config: XArm7BinFinderConfig

    color_image: In[Image]
    depth_image: In[Image]
    camera_info: In[CameraInfo]

    _kitting: KittingSpec

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = threading.Lock()
        self._color: Image | None = None
        self._depth: Image | None = None
        self._K: FloatArray | None = None
        self._frames = 0

    @rpc
    def start(self) -> None:
        super().start()
        self.color_image.subscribe(self._on_color)
        self.depth_image.subscribe(self._on_depth)
        self.camera_info.subscribe(self._on_info)

    def _on_color(self, msg: Image) -> None:
        with self._lock:
            self._color = msg

    def _on_depth(self, msg: Image) -> None:
        with self._lock:
            self._depth = msg
            self._frames += 1

    def _on_info(self, msg: CameraInfo) -> None:
        with self._lock:
            self._K = np.asarray(msg.K, dtype=float).reshape(3, 3)

    @rpc
    def find_bins(self) -> dict[str, Any]:
        """Every bin the camera sees: footprint centre (m), ``yaw_deg`` from back wall to scoop,
        zone, and how far the measured size is from a stack bin's. Saves an annotated image."""
        T = self._camera_pose()
        clouds: list[FloatArray] = []
        last: tuple[NDArray[Any], NDArray[Any], FloatArray] | None = None
        seen = self._frames
        deadline = time.monotonic() + self.config.scan_timeout_s
        while len(clouds) < self.config.scan_frames and time.monotonic() < deadline:
            with self._lock:
                color, depth_msg, K = self._color, self._depth, self._K
                fresh = self._frames > seen
                seen = self._frames
            if not fresh or color is None or depth_msg is None or K is None:
                time.sleep(0.02)
                continue
            rgb, depth = _rgb(color), _metres(depth_msg)
            clouds.append(self._lift(rgb, depth, K, T))
            last = (rgb, depth, K)
        if last is None:
            raise RuntimeError("no colour, depth and camera_info from the camera")
        bins = self._fit_bins(np.concatenate(clouds) if clouds else np.zeros((0, 3)))
        image = self._annotate(last[0], last[2], T, bins)
        logger.info("Bin scan", bins=[(b["zone"], b["x"], b["y"], b["yaw_deg"]) for b in bins])
        return {"bins": bins, "frames": len(clouds), "image": image}

    @rpc
    def load_from_scan(self) -> dict[str, Any]:
        """Scan, then load the empty bin in the middle zone nearest the loading slot."""
        return self._hand_over(lambda b: self._kitting.load(b["x"], b["y"], b["yaw_deg"]))

    @rpc
    def cycle_from_scan(self) -> dict[str, Any]:
        """Scan, then run the full cycle with the empty bin in the middle zone."""
        return self._hand_over(lambda b: self._kitting.run_cycle(b["x"], b["y"], b["yaw_deg"]))

    @rpc
    def unload_from_scan(self) -> dict[str, Any]:
        """Scan, then carry the bin in the loading slot to the first done slot no bin is in."""
        scan, state = self._tell_kitting()
        bench = XArm7KittingConfig()
        loading = [b for b in scan["bins"] if b["zone"] == "loading"]
        if len(loading) != 1:
            reason = f"{len(loading)} bins in the loading zone, need one"
            return {"started": reason, "bench": state, "scan": scan}
        b, slot = loading[0], bench.loading_slot
        off_m = float(np.hypot(b["x"] - slot.x, b["y"] - slot.y))
        off_deg = abs((b["yaw_deg"] - slot.yaw_deg + 180) % 360 - 180)
        if off_m > LOADING_TOL_M or off_deg > LOADING_TOL_DEG or not b["size_ok"]:
            reason = (
                f"the loading bin is {off_m * 1000:.0f} mm and {off_deg:.0f} deg from the slot"
                f" (size_ok {b['size_ok']}); set it straight"
            )
            return {"started": reason, "bin": b, "bench": state, "scan": scan}
        free = [i for i in range(len(bench.done_slots)) if i not in state["done_taken"]]
        if not free:
            return {"started": "every done slot is taken", "bench": state, "scan": scan}
        return {
            "started": self._kitting.unload(),
            "bin": b,
            "done_slot": free[0],
            "bench": state,
            "scan": scan,
        }

    def _hand_over(self, start: Any) -> dict[str, Any]:
        scan, state = self._tell_kitting()
        # a blob that is not a stack bin's size (two bins touching, a stray object) is never handed on
        empties = [b for b in scan["bins"] if b["zone"] == "middle" and b["size_ok"]]
        if not empties:
            return {"started": "no empty bin in the middle zone", "bench": state, "scan": scan}
        slot = XArm7KittingConfig().loading_slot
        pick = min(empties, key=lambda b: np.hypot(b["x"] - slot.x, b["y"] - slot.y))
        return {"started": start(pick), "bin": pick, "bench": state, "scan": scan}

    def _tell_kitting(self) -> tuple[dict[str, Any], dict[str, Any]]:
        """Scan, and replace what the kitting module was told about the slots with what is seen."""
        scan = self.find_bins()
        bench = XArm7KittingConfig()
        loading_full = any(b["zone"] == "loading" for b in scan["bins"])
        taken = [
            i
            for i, slot in enumerate(bench.done_slots)
            if any(blocks_slot(b, slot.x, slot.y, slot.yaw_deg) for b in scan["bins"])
        ]
        self._kitting.set_loading_full(loading_full)
        self._kitting.set_done_taken(taken)
        return scan, {"loading_full": loading_full, "done_taken": taken}

    def _camera_pose(self) -> FloatArray:
        path = STATE_DIR / "calibration" / f"{self.config.calibration}.json"
        if not path.exists():
            raise RuntimeError(f"no camera calibration at {path}; run the scene calibration")
        optical = json.loads(path.read_text())["optical"]
        T = np.eye(4)
        T[:3, :3] = Rotation.from_quat(optical["rotation_xyzw"]).as_matrix()
        T[:3, 3] = optical["translation_m"]
        return T

    def _lift(
        self, rgb: NDArray[Any], depth: FloatArray, K: FloatArray, T: FloatArray
    ) -> FloatArray:
        """Green pixels with good depth, as points in the arm's base, above the table."""
        cfg = self.config
        if depth.shape[:2] != rgb.shape[:2]:
            depth = np.asarray(
                cv2.resize(depth, rgb.shape[1::-1], interpolation=cv2.INTER_NEAREST), dtype=float
            )
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        green = cv2.inRange(hsv, np.array(cfg.hsv_lo), np.array(cfg.hsv_hi)) > 0
        lo, hi = cfg.depth_range_m
        v, u = np.nonzero(green & (depth > lo) & (depth < hi))
        z = depth[v, u]
        cam = np.stack(
            [(u - K[0, 2]) * z / K[0, 0], (v - K[1, 2]) * z / K[1, 1], z, np.ones_like(z)]
        )
        p = (T @ cam)[:3].T
        keep = (
            (p[:, 2] > cfg.table_top_z + cfg.min_height_m)
            & (p[:, 2] < cfg.table_top_z + BIN_HEIGHT_M + 0.05)
            & (p[:, 0] > cfg.bench_x_m[0])
            & (p[:, 0] < cfg.bench_x_m[1])
            & (p[:, 1] > cfg.bench_y_m[0])
            & (p[:, 1] < cfg.bench_y_m[1])
        )
        out: FloatArray = p[keep]
        return out

    def _fit_bins(self, p: FloatArray) -> list[dict[str, Any]]:
        if len(p) == 0:
            return []
        cfg = self.config
        x0, y0 = cfg.bench_x_m[0], cfg.bench_y_m[0]
        ij = np.floor((p[:, :2] - (x0, y0)) / GRID_M).astype(int)
        shape = (
            int((cfg.bench_x_m[1] - x0) / GRID_M) + 1,
            int((cfg.bench_y_m[1] - y0) / GRID_M) + 1,
        )
        grid = np.zeros(shape, np.uint8)
        grid[ij[:, 0], ij[:, 1]] = 1
        # no dilation: bins a couple of centimetres apart stay apart
        n, labels = cv2.connectedComponents(grid, connectivity=8)
        cell_label = labels[ij[:, 0], ij[:, 1]]
        bins = []
        for k in range(1, n):
            if np.count_nonzero(labels == k) < MIN_CELLS:
                continue
            fit = self._fit_one(p[cell_label == k])
            if fit is not None:
                bins.append(fit)
        return bins

    def _fit_one(self, p: FloatArray) -> dict[str, Any] | None:
        """The bin's rectangle from its rim, the points near full height: the outline of the back
        wall and the sides, which a camera above the bench sees whichever way the bin faces."""
        cfg = self.config
        rim = p[p[:, 2] > cfg.table_top_z + RIM_FRACTION * BIN_HEIGHT_M]
        if len(rim) < MIN_RIM_POINTS:
            return None
        mm = (rim[:, :2] * 1000).astype(np.float32)
        (cx, cy), (w, h), angle = cv2.minAreaRect(mm)
        across = np.radians(angle if w < h else angle + 90.0)
        b = (mm - (cx, cy)) @ np.array([np.cos(across), np.sin(across)])
        lo, hi = np.percentile(b, [WIDTH_TRIM_PCT, 100.0 - WIDTH_TRIM_PCT])
        inside = (b >= lo) & (b <= hi)
        rim = rim[inside]
        (cx, cy), (w, h), angle = cv2.minAreaRect(mm[inside])
        along_deg = angle if w >= h else angle + 90.0
        along = np.array([np.cos(np.radians(along_deg)), np.sin(np.radians(along_deg))])
        centre: FloatArray = np.array([cx, cy], dtype=float) / 1000
        length, width = max(w, h) / 1000, min(w, h) / 1000
        # The back wall's rim runs across the whole end; the scoop front is low, so its end of
        # the rim holds only the two side walls' tops.
        a = (rim[:, :2] - centre) @ along
        band = END_FRACTION * length / 2
        n_plus = int(np.count_nonzero(a > length / 2 - band))
        n_minus = int(np.count_nonzero(a < -length / 2 + band))
        to_scoop = -along if n_plus > n_minus else along
        # the lip on the back rim lengthens the rim's outline at the back only
        centre = centre + to_scoop * float(np.clip(length - BIN_LENGTH_M, 0.0, MAX_LIP_M)) / 2
        yaw = float(np.degrees(np.arctan2(to_scoop[1], to_scoop[0])))
        x, y = float(centre[0]), float(centre[1])
        zone = "loading" if y > cfg.loading_min_y else "done" if y < cfg.done_max_y else "middle"
        return {
            "x": round(x, 4),
            "y": round(y, 4),
            "yaw_deg": round(yaw, 1),
            "zone": zone,
            "length_m": round(length, 3),
            "width_m": round(width, 3),
            "size_ok": bool(
                abs(length - BIN_LENGTH_M) < SIZE_TOL_M and abs(width - BIN_WIDTH_M) < SIZE_TOL_M
            ),
            "ends": [n_minus, n_plus],
            "points": len(p),
        }

    def _annotate(
        self, rgb: NDArray[Any], K: FloatArray, T: FloatArray, bins: list[dict[str, Any]]
    ) -> str:
        """The scan's last frame with each fitted footprint drawn on it, for checking by eye."""
        img = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        T_cam = np.linalg.inv(T)
        z = self.config.table_top_z
        for b in bins:
            yaw = np.radians(b["yaw_deg"])
            f, s = np.array([np.cos(yaw), np.sin(yaw)]), np.array([-np.sin(yaw), np.cos(yaw)])
            c = np.array([b["x"], b["y"]])
            hl, hw = BIN_LENGTH_M / 2, BIN_WIDTH_M / 2
            ring = [
                c + f * hl + s * hw,
                c + f * hl - s * hw,
                c - f * hl - s * hw,
                c - f * hl + s * hw,
            ]
            pts = []
            for q in (*ring, c, c + f * hl):
                pc = T_cam @ np.array([q[0], q[1], z, 1.0])
                pts.append(
                    (int(K[0, 0] * pc[0] / pc[2] + K[0, 2]), int(K[1, 1] * pc[1] / pc[2] + K[1, 2]))
                )
            cv2.polylines(img, [np.array(pts[:4])], True, (0, 0, 255), 2)
            cv2.arrowedLine(img, pts[4], pts[5], (255, 0, 0), 2)
            cv2.putText(
                img,
                f"{b['zone']} {b['x']:.3f},{b['y']:.3f} {b['yaw_deg']:.0f}",
                pts[4],
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 255),
                1,
            )
        out = STATE_DIR / "kitting" / f"scan_{time.strftime('%Y%m%d_%H%M%S')}.png"
        out.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(out), img)
        return str(out)


def blocks_slot(b: dict[str, Any], x: float, y: float, yaw_deg: float) -> bool:
    """Whether a seen bin stands in, or within SLOT_CLEAR_M of, the bin footprint at a slot."""
    grow = 2 * SLOT_CLEAR_M
    slot = ((x, y), (BIN_LENGTH_M + grow, BIN_WIDTH_M + grow), yaw_deg)
    seen = (
        (b["x"], b["y"]),
        (max(b["length_m"], BIN_LENGTH_M), max(b["width_m"], BIN_WIDTH_M)),
        b["yaw_deg"],
    )
    kind, _ = cv2.rotatedRectangleIntersection(slot, seen)
    return bool(kind != cv2.INTERSECT_NONE)


def _rgb(msg: Image) -> NDArray[Any]:
    data = np.asarray(msg.data)
    fmt = str(getattr(msg.format, "value", msg.format))
    return cv2.cvtColor(data, cv2.COLOR_BGR2RGB) if fmt == "BGR" else data[..., :3]


def _metres(msg: Image) -> FloatArray:
    data = np.asarray(msg.data)
    if data.dtype == np.uint16:
        return data.astype(float) * 0.001
    return data.astype(float)
