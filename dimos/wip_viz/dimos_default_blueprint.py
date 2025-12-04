#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

import rerun as rr  # pip install rerun-sdk
import rerun.blueprint as rrb

# idea:
    # 3D view
    # top down lidar view
    
# blueprint = rrb.Horizontal(
#     rrb.Spatial3DView(name="3D"),
#     rrb.Vertical(
#         rrb.Tabs(
#             # Note that we re-project the annotations into the 2D views:
#             # For this to work, the origin of the 2D views has to be a pinhole camera,
#             # this way the viewer knows how to project the 3D annotations into the 2D views.
#             rrb.Spatial2DView(
#                 name="BGR",
#                 origin="world/camera_highres",
#                 contents=["$origin/bgr", "/world/annotations/**"],
#             ),
#             rrb.Spatial2DView(
#                 name="Depth",
#                 origin="world/camera_highres",
#                 contents=["$origin/depth", "/world/annotations/**"],
#             ),
#             name="2D",
#         ),
#         rrb.TextDocumentView(name="Readme"),
#         row_shares=[2, 1],
#     ),
# )
dimos_default_blueprint = rrb.Blueprint(
    rrb.Tabs(
        rrb.Spatial3DView(
            name="Spatial3D",
            origin="/spatial3d",
            line_grid=rrb.LineGrid3D(spacing=1.0, stroke_width=1.0),
        ),
        rrb.Spatial2DView(name="Spatial2D", origin="/spatial2d"),
        rrb.BarChartView(name="Bar Chart", origin="/bar_chart"),
        rrb.DataframeView(name="Dataframe", origin="/dataframe"),
        rrb.GraphView(name="Graph", origin="/graph"),
        rrb.MapView(name="Map", origin="/map"),
        rrb.TensorView(name="Tensor", origin="/tensor"),
        rrb.TextDocumentView(name="Text Doc", origin="/text_doc"),
        rrb.TimePanel(),
    ),
    collapse_panels=False,
)

if __name__ == "__main__":
    rr.init("rerun_mega_blueprint", spawn=True)
    rr.send_blueprint(build_mega_blueprint())
