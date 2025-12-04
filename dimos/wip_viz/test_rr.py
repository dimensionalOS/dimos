from __future__ import annotations

import argparse
import json
import os
from typing import TYPE_CHECKING, Any
    
import cv2
import numpy as np
import rerun as rr  # pip install rerun-sdk
import rerun.blueprint as rrb
import trimesh
from scipy.spatial.transform import Rotation as R

import numpy as np
import rerun as rr

from dimos.wip_viz.dimos_msg_to_rr import dimos_msg_to_rr
from dimos.wip_viz.create_example_msgs import create_example_msgs
from dimos.wip_viz.dimos_default_blueprint import dimos_default_blueprint

def rr_viz_test(misc_datapoint):
    import rerun as rr

    from math import tau
    import numpy as np
    from rerun.utilities import build_color_spiral
    from rerun.utilities import bounce_lerp
    
    
    three_d_points = None
    if isinstance(misc_datapoint, rr.Points3D):
        three_d_points = misc_datapoint

    rr.init("rerun_example_dna_abacus")
    rr.spawn()

    NUM_POINTS = 100

    # Points and colors are both np.array((NUM_POINTS, 3))
    points1, colors1 = build_color_spiral(NUM_POINTS)
    points2, colors2 = build_color_spiral(NUM_POINTS, angular_offset=tau*0.5)
    
    if not three_d_points:
        three_d_points = rr.Points3D(points1, colors=colors1, radii=0.08)
        
    rr.log("dna/structure/left", three_d_points)
    rr.log("dna/structure/right", rr.Points3D(points2, colors=colors2, radii=0.08))

    rr.log(
        "dna/structure/scaffolding",
        rr.LineStrips3D(np.stack((points1, points2), axis=1), colors=[128, 128, 128])
    )
    offsets = np.random.rand(NUM_POINTS)
    beads = [bounce_lerp(points1[n], points2[n], offsets[n]) for n in range(NUM_POINTS)]
    colors = [[int(bounce_lerp(80, 230, offsets[n] * 2))] for n in range(NUM_POINTS)]
    rr.log(
        "dna/structure/scaffolding/beads",
        rr.Points3D(beads, radii=0.06, colors=np.repeat(colors, 3, axis=-1)),
    )


for unknown_msg in create_example_msgs():
    rr_message = dimos_msg_to_rr(unknown_msg)
    # FIXME: send it to some kind of rerun viewer
