import cv2
import numpy as np
from typing import Optional

from dimos.perception.optical_flow.tac_estimator import divergence_to_tac


class LucasKanadeBackend:
    """
    FAST keypoints + Lucas-Kanade sparse optical flow.
    Computes a Time-to-Contact proxy (tau) via flow divergence in an NxN grid.
    Based on Lee (1976): a closer/faster-approaching obstacle produces larger
    image-expansion and thus smaller tau. Scale-free; no depth calibration.

    Note: the tau value is monotonically related to physical TTC but is not
    literally seconds — it depends on frame rate and cell geometry. Use
    tau_threshold as an empirically tuned urgency parameter. The default 3.0
    was chosen from the PR-curve sweep on unitree_office_walk (P=0.939 at the
    production operating point).
    """

    def __init__(self, max_keypoints=300, grid_size=5,
                 tau_threshold=3.0, refresh_interval=10,
                 fast_threshold=20):
        self.max_keypoints    = max_keypoints
        self.grid_size        = grid_size
        self.tau_threshold    = tau_threshold
        self.refresh_interval = refresh_interval
        self.lk_params = dict(
            winSize=(21, 21),
            maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03),
        )
        self.fast = cv2.FastFeatureDetector_create(
            threshold=fast_threshold, nonmaxSuppression=True
        )
        self.prev_gray:   Optional[np.ndarray] = None
        self.prev_pts:    Optional[np.ndarray] = None
        self.frame_count: int = 0

    def compute(self, frame_bgr: np.ndarray) -> Optional[dict]:  # type: ignore[type-arg]
        """
        Args:
            frame_bgr: HxWx3 uint8 BGR numpy array
        Returns:
            dict: {tac_grid, danger, flow_pts, min_tau}
            None on first frame.
        """
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)

        if self.prev_gray is None or self.prev_pts is None or len(self.prev_pts) == 0:
            self.prev_gray = gray
            self._refresh(gray)
            return None

        curr_pts, status, _ = cv2.calcOpticalFlowPyrLK(
            self.prev_gray, gray, self.prev_pts, None, **self.lk_params
        )

        good_mask = status.ravel() == 1
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

        h, w     = gray.shape
        tac_grid = self._tac_grid(good_prev, flow_vecs, h, w)
        min_tau  = float(np.nanmin(tac_grid)) if not np.all(np.isnan(tac_grid)) else np.inf

        return {
            "tac_grid": tac_grid,
            "danger":   min_tau < self.tau_threshold,
            "flow_pts": (good_prev, good_curr),
            "min_tau":  min_tau,
        }

    def _refresh(self, gray: np.ndarray) -> None:
        kps = sorted(self.fast.detect(gray, None),
                     key=lambda k: k.response, reverse=True)[:self.max_keypoints]
        self.prev_pts = (
            np.array([[k.pt] for k in kps], dtype=np.float32) if kps else None
        )

    def _tac_grid(self, pts: np.ndarray, flows: np.ndarray, h: int, w: int) -> np.ndarray:
        div_grid = np.full((self.grid_size, self.grid_size), np.nan, dtype=np.float32)
        cell_h   = h / self.grid_size
        cell_w   = w / self.grid_size
        for i in range(self.grid_size):
            for j in range(self.grid_size):
                mask = (
                    (pts[:, 0] >= j * cell_w) & (pts[:, 0] < (j+1) * cell_w) &
                    (pts[:, 1] >= i * cell_h) & (pts[:, 1] < (i+1) * cell_h)
                )
                cf = flows[mask]
                if len(cf) < 3:
                    continue
                div_grid[i, j] = np.mean(cf[:, 0]) / cell_w + np.mean(cf[:, 1]) / cell_h
        return divergence_to_tac(div_grid)
