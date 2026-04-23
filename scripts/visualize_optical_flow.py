"""
Live visualization of the Optical Flow obstacle-avoidance module on
unitree_office_walk. Streams annotated frames, flow vectors, tau grid,
and min_tau time series into a Rerun viewer (spawned automatically).

Run from repo root:
    PYTHONPATH=.venv/lib/python3.12/site-packages/rerun_sdk \\
        uv run python3 scripts/visualize_optical_flow.py
"""

import sys
from pathlib import Path

import cv2
import numpy as np
import rerun as rr

sys.path.insert(0, ".")
from dimos.memory.timeseries.legacy import LegacyPickleStore
from dimos.perception.optical_flow.backends.lucas_kanade import LucasKanadeBackend

REPO_ROOT = Path(__file__).resolve().parent.parent
DATA      = REPO_ROOT / "data" / "unitree_office_walk"


def draw_overlay(frame_bgr, result, tau_threshold, grid_size):
    """Draw flow arrows + tau grid + DANGER/CLEAR label onto a copy."""
    viz = frame_bgr.copy()
    h, w = viz.shape[:2]

    gp, gc = result["flow_pts"]
    for p, c in zip(gp, gc):
        cv2.arrowedLine(viz, (int(p[0]), int(p[1])), (int(c[0]), int(c[1])),
                        (0, 255, 0), 1, tipLength=0.4)

    cell_h = h / grid_size
    cell_w = w / grid_size
    for i in range(grid_size):
        for j in range(grid_size):
            tau = result["tac_grid"][i, j]
            if np.isnan(tau):
                continue
            danger = tau < tau_threshold
            color  = (0, 0, 255) if danger else (0, 200, 200)
            x0, y0 = int(j * cell_w), int(i * cell_h)
            x1, y1 = int((j + 1) * cell_w), int((i + 1) * cell_h)
            cv2.rectangle(viz, (x0, y0), (x1, y1), color, 2)
            cv2.putText(viz, f"{tau:.1f}", (x0 + 4, y0 + 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)

    label       = "DANGER" if result["danger"] else "CLEAR"
    label_color = (0, 0, 255) if result["danger"] else (0, 255, 0)
    cv2.putText(viz, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, label_color, 2)
    cv2.putText(viz, f"min_tau={result['min_tau']:.2f}",
                (10, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
    return viz


def main():
    rr.init("optical_flow_1775", spawn=True)
    rr.log("threshold", rr.SeriesLines(colors=[255, 128, 0], names="tau_threshold"),
           static=True)

    backend = LucasKanadeBackend()
    video   = LegacyPickleStore(DATA / "video")

    print(f"Streaming {DATA.name} to Rerun…  tau_threshold={backend.tau_threshold}")

    n = 0
    for ts, frame in video._iter_items():
        rr.set_time("video", timestamp=ts)

        if not isinstance(frame, np.ndarray):
            frame = np.asarray(frame)

        result = backend.compute(frame)
        if result is None:
            rr.log("camera", rr.Image(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)))
            n += 1
            continue

        viz = draw_overlay(frame, result, backend.tau_threshold, backend.grid_size)
        rr.log("camera",    rr.Image(cv2.cvtColor(viz, cv2.COLOR_BGR2RGB)))
        rr.log("min_tau",   rr.Scalars(float(result["min_tau"]) if np.isfinite(result["min_tau"]) else 0.0))
        rr.log("threshold", rr.Scalars(backend.tau_threshold))
        rr.log("danger",    rr.TextLog("DANGER" if result["danger"] else "CLEAR"))
        n += 1

    print(f"Done. {n} frames streamed. Viewer stays open; Ctrl+C to exit.")


if __name__ == "__main__":
    main()
