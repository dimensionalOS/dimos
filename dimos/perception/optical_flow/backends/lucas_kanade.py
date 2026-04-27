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


import cv2
import numpy as np


class GridDetector:
    """
    Uniform pixel lattice
    """

    def __init__(self, spacing: int = 10):
        self.spacing = spacing

    def detect(self, gray: np.ndarray) -> np.ndarray | None:
        h, w = gray.shape
        xs = np.arange(0, w, self.spacing)
        ys = np.arange(0, h, self.spacing)
        gx, gy = np.meshgrid(xs, ys)
        pts = np.stack([gx.ravel(), gy.ravel()], axis=1).astype(np.float32)
        return pts.reshape(-1, 1, 2) if len(pts) else None


class LucasKanadeBackend:
    def __init__(
        self,
        detector: GridDetector | None = None,
        tau_threshold: float = 3.0,
        refresh_interval: int = 10,
        window_size: int = 21,
        err_threshold: float = 30.0,
        min_blob_area: int = 15,
    ):
        """
        Args:
            detector: Generates seed points for tracking (default: GridDetector).
            tau_threshold: Time-to-Contact (frames) limit. Triggers danger if τ < this.
            refresh_interval: Frames between redetecting points to prevent drift.
            window_size: Edge length of the Lucas-Kanade local tracking window.
            err_threshold: Cutoff for LK photometric reconstruction error; drops bad tracks.
            min_blob_area: Minimum connected-component area (grid cells) required to
                trigger the alarm, filtering out isolated tracking noise.
        """
        self.detector = detector or GridDetector()
        self.tau_threshold = tau_threshold
        self.refresh_interval = refresh_interval
        self.window_size = int(window_size)
        self.err_threshold = float(err_threshold)
        self.min_blob_area = int(min_blob_area)
        self.lk_params = dict(
            winSize=(self.window_size, self.window_size),
            maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03),
        )
        self.prev_gray: np.ndarray | None = None
        self.prev_pts: np.ndarray | None = None
        self.frame_count: int = 0

    def compute(self, frame_bgr: np.ndarray) -> dict | None:  # type: ignore[type-arg]
        """
        Args:
            frame_bgr: HxWx3 uint8 BGR numpy array
        Returns:
            dict: {flow_data, danger}
                flow_data: (N, 5) float32 [x, y, tau, u, v] per tracked point.
                           tau is NaN for non-expanding points.
                danger:    bool — true iff a low-τ blob of area ≥ min_blob_area exists.
        """
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)

        if self.prev_gray is None or self.prev_pts is None or len(self.prev_pts) == 0:
            self.prev_gray = gray
            self._refresh(gray)
            return None

        curr_pts, status, err = cv2.calcOpticalFlowPyrLK(
            self.prev_gray, gray, self.prev_pts, None, **self.lk_params
        )

        good_mask = (status.ravel() == 1) & (err.ravel() < self.err_threshold)
        good_prev = self.prev_pts[good_mask].reshape(-1, 2)
        good_curr = curr_pts[good_mask].reshape(-1, 2)
        flow_vecs = good_curr - good_prev

        self.frame_count += 1
        if self.frame_count % self.refresh_interval == 0 or len(good_prev) < 50:
            self._refresh(gray)
        else:
            self.prev_pts = good_curr.reshape(-1, 1, 2)
        self.prev_gray = gray

        if len(good_prev) < 10:
            return None

        h, w = gray.shape
        spacing = self.detector.spacing
        flow_data, div_smooth = self._divergence_tau(good_prev, flow_vecs, h, w, spacing)

        # Alarm via connected components on the thresholded divergence map.
        # Equivalent threshold: τ < tau_threshold ⇔ div > 2/tau_threshold.
        div_threshold = 2.0 / self.tau_threshold
        mask = (div_smooth > div_threshold).astype(np.uint8)
        n_labels, _, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)

        danger = False
        if n_labels > 1:
            largest_area = int(stats[1:, cv2.CC_STAT_AREA].max())
            danger = largest_area >= self.min_blob_area

        return {
            "flow_data": flow_data,
            "danger": danger,
        }

    @staticmethod
    def _divergence_tau(
        pts: np.ndarray,
        flows: np.ndarray,
        h: int,
        w: int,
        spacing: int,
    ) -> tuple:
        """
        Returns:
            flow_data: (N, 5) float32 [x, y, tau, u, v] per tracked point —
                       tau mapped from the per-cell divergence map, for
                       per-arrow visualization coloring.
            div_smooth: 2D divergence map at grid resolution, for the
                        downstream connected-components alarm.
        """
        H_grid = int(np.ceil(h / spacing))
        W_grid = int(np.ceil(w / spacing))

        u_grid = np.zeros((H_grid, W_grid), dtype=np.float32)
        v_grid = np.zeros((H_grid, W_grid), dtype=np.float32)
        j_idx = np.round(pts[:, 0] / spacing).astype(np.int32)
        i_idx = np.round(pts[:, 1] / spacing).astype(np.int32)
        in_bounds = (i_idx >= 0) & (i_idx < H_grid) & (j_idx >= 0) & (j_idx < W_grid)
        u_grid[i_idx[in_bounds], j_idx[in_bounds]] = flows[in_bounds, 0]
        v_grid[i_idx[in_bounds], j_idx[in_bounds]] = flows[in_bounds, 1]

        # Pre-smooth the flow field and suppresses LK noise
        u_smooth = cv2.GaussianBlur(u_grid, (3, 3), sigmaX=0)
        v_smooth = cv2.GaussianBlur(v_grid, (3, 3), sigmaX=0)

        # Spatial derivatives
        du_dy, du_dx = np.gradient(u_smooth, spacing)
        dv_dy, dv_dx = np.gradient(v_smooth, spacing)
        div = (du_dx + dv_dy).astype(np.float32)

        # Post-smooth divergence to absorb residual outliers.
        div_smooth = cv2.medianBlur(div, 3)

        # τ = 2 / div, only where div is positive (point is expanding =
        # surface is approaching). Negative or near-zero div → NaN.
        tau_grid = np.full_like(div_smooth, np.nan)
        eps = 1e-3
        expanding = div_smooth > eps
        tau_grid[expanding] = 2.0 / div_smooth[expanding]

        # Per-point τ
        taus = np.full(len(pts), np.nan, dtype=np.float32)
        taus[in_bounds] = tau_grid[i_idx[in_bounds], j_idx[in_bounds]]

        flow_data = np.stack(
            [
                pts[:, 0].astype(np.float32),
                pts[:, 1].astype(np.float32),
                taus,
                flows[:, 0].astype(np.float32),
                flows[:, 1].astype(np.float32),
            ],
            axis=1,
        )
        return flow_data, div_smooth

    def _refresh(self, gray: np.ndarray) -> None:
        self.prev_pts = self.detector.detect(gray)
