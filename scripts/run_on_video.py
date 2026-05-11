import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, ".")
from dimos.perception.optical_flow.backends.lucas_kanade import LucasKanadeBackend

REPO_ROOT = Path(__file__).resolve().parent.parent
OUT_DIR   = REPO_ROOT / "results"

TAU_THRESHOLD = 3.0   # alarm threshold (frames); matches OpticalFlowConfig default
TARGET_FPS    = 15    # downsample iPhone 30/60fps to match Go2 dataset frame rate;
                      # τ is measured in frames so the absolute rate matters for the threshold


def draw_overlay(frame_bgr: np.ndarray, result: dict, tau_threshold: float) -> np.ndarray:
    """Per-point flow arrows colored by tau band + DANGER/CLEAR banner.
    Mirrors what OpticalFlowModule._draw_visualization. do"""
    viz    = frame_bgr.copy()
    red    = (0,   0, 255)
    yellow = (0, 220, 220)
    green  = (0, 200,   0)

    for x, y, tau, u, v in result["flow_data"]:
        if tau < tau_threshold:
            color = red
        elif tau < 2.0 * tau_threshold:
            color = yellow
        else:
            color = green
        cv2.arrowedLine(viz, (int(x), int(y)), (int(x + u), int(y + v)),
                        color, 1, tipLength=0.4)

    label = "DANGER" if result["danger"] else "CLEAR"
    color = red if result["danger"] else (0, 255, 0)
    cv2.putText(viz, label, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.4, color, 3)
    return viz


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("video", help="path to the input video file (mp4/mov/etc.)")
    p.add_argument("--target-fps", type=int, default=TARGET_FPS,
                   help=f"downsample to this fps (default: {TARGET_FPS}, matches Go2 dataset)")
    args = p.parse_args()

    video_path = Path(args.video)
    if not video_path.exists():
        sys.exit(f"video file {video_path} does not exist")
    OUT_DIR.mkdir(exist_ok=True)
    out_path = OUT_DIR / f"annotated_{video_path.stem}.mp4"

    cap = cv2.VideoCapture(str(video_path))
    src_fps = cap.get(cv2.CAP_PROP_FPS)
    n_src   = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    w       = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h       = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    # Downsample by integer stride so τ-in-frames is calibrated for our threshold.
    stride = max(1, round(src_fps / args.target_fps))
    out_fps = src_fps / stride
    print(f"input:  {video_path.name}  {w}x{h} @ {src_fps:.2f}fps  ({n_src} frames, {n_src/src_fps:.1f}s)")
    print(f"stride: every {stride}-th frame → output ~{out_fps:.1f}fps")

    backend = LucasKanadeBackend(tau_threshold=TAU_THRESHOLD)
    fourcc  = cv2.VideoWriter_fourcc(*"mp4v")
    writer  = cv2.VideoWriter(str(out_path), fourcc, out_fps, (w, h))

    n_processed = 0
    n_danger    = 0
    n_warmup    = 0
    src_idx     = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if src_idx % stride != 0:
            src_idx += 1
            continue
        src_idx += 1

        result = backend.compute(frame)
        if result is None:
            n_warmup += 1
            continue

        n_processed += 1
        if result["danger"]:
            n_danger += 1

        viz = draw_overlay(frame, result, TAU_THRESHOLD)
        writer.write(viz)

    writer.release()
    cap.release()

    print()
    print("Done.")
    print(f"  processed:     {n_processed} frames")
    print(f"  backend warmup/sparse: {n_warmup} frames")
    print(f"  DANGER fires:  {n_danger} / {n_processed} ({100*n_danger/max(1,n_processed):.1f}%)")
    print(f"  output:        {out_path}")


if __name__ == "__main__":
    main()
