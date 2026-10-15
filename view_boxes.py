#!/usr/bin/env python3
"""View the DimSim apartment ground-truth boxes as an interactive 3D plot.

Usage:
    python view_boxes.py                 # all boxes
    python view_boxes.py --objects       # objects only
    python view_boxes.py --walls         # walls only
    python view_boxes.py --save out.png  # write a PNG instead of a window
"""

import argparse
import itertools
import json
from pathlib import Path
import re

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Line3DCollection

SRC = Path(__file__).parent / "misc/DimSim/scenes/apartment/object_detections.json"
WALL_RE = re.compile(r"(^|[-_ ])wall([-_ ]|$)", re.I)
OBJ, WALL_C = "#1f9e8f", "#d97a2b"


def is_wall(det):
    _id, lbl = det.get("id", ""), det.get("label", "")
    return bool(_id and _id == lbl and WALL_RE.search(_id))


def box_edges(c, s):
    hx, hy, hz = s[0] / 2, s[1] / 2, s[2] / 2
    corners = [
        (c[0] + sx * hx, c[1] + sy * hy, c[2] + sz * hz)
        for sx, sy, sz in itertools.product((-1, 1), (-1, 1), (-1, 1))
    ]
    return [
        (corners[i], corners[j])
        for i in range(8)
        for j in range(i + 1, 8)
        if bin(i ^ j).count("1") == 1
    ]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--objects", action="store_true", help="objects only")
    ap.add_argument("--walls", action="store_true", help="walls only")
    ap.add_argument("--save", metavar="PNG", help="save to file instead of opening a window")
    args = ap.parse_args()

    dets = json.load(open(SRC))["detections"]
    obj_segs, wall_segs = [], []
    for det in dets:
        segs = box_edges(det["center_xyz"], det["size_xyz"])
        (wall_segs if is_wall(det) else obj_segs).extend(segs)

    fig = plt.figure(figsize=(11, 8), facecolor="#0b0e13")
    ax = fig.add_subplot(111, projection="3d")
    if not args.objects:
        ax.add_collection3d(Line3DCollection(wall_segs, colors=WALL_C, linewidths=1.1, alpha=0.85))
    if not args.walls:
        ax.add_collection3d(Line3DCollection(obj_segs, colors=OBJ, linewidths=0.9, alpha=0.95))
    ax.set_xlim(-6, 10)
    ax.set_ylim(-7, 7)
    ax.set_zlim(0, 5.5)
    ax.set_box_aspect((16, 14, 5.5))
    ax.view_init(elev=28, azim=-60)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_facecolor("#0e1116")
    n_obj = sum(1 for d in dets if not is_wall(d))
    n_wall = len(dets) - n_obj
    ax.set_title(
        f"DimSim apartment  |  teal=objects ({n_obj})  orange=walls ({n_wall})",
        color="#e6ecf5",
        family="monospace",
        fontsize=11,
    )
    for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
        axis.label.set_color("#8a97ab")
        for t in axis.get_ticklabels():
            t.set_color("#8a97ab")

    if args.save:
        fig.savefig(args.save, dpi=120, facecolor="#0b0e13")
        print("saved", args.save)
    else:
        plt.show()


if __name__ == "__main__":
    main()
