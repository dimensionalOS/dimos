#!/usr/bin/env python3
from collections import namedtuple
import rerun as rr  # pip install rerun-sdk
import rerun.blueprint as rrb

from dimos.dashboard.rerun.layouts_base import Layout

# example of rerun blueprint types: 
# NOTES:
#     only one rerun blueprint can be active at a time
#     we can very easily allow multiple types of blueprints, with this just being one kind of layout
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

from dataclasses import dataclass
from typing import Literal

@dataclass(frozen=True)
class LayoutAllTabsEntities:
    spatial3d: Literal["spatial3d"] = "/spatial3d"
    spatial2d: Literal["spatial2d"] = "/spatial2d"
    bar_chart: Literal["bar_chart"] = "/bar_chart"
    dataframe: Literal["dataframe"] = "/dataframe"
    graph    : Literal["graph"]     = "/graph"
    map      : Literal["map"]       = "/map"
    tensor   : Literal["tensor"]    = "/tensor"
    text_doc : Literal["text_doc"]  = "/text_doc"
    image    : Literal["image"]     = "/image"

class LayoutAllTabs(Layout):
    entities = LayoutAllTabsEntities()
    def __init__(self, collapse_panels=False) -> None:  # type: ignore[no-untyped-def]
        self.rerun_blueprint = rrb.Blueprint(
            rrb.Tabs(
                rrb.Spatial3DView(
                    name="Spatial3D",
                    origin=self.entities.spatial3d,
                    line_grid=rrb.LineGrid3D(spacing=1.0, stroke_width=1.0),
                ),
                rrb.Spatial2DView(name="Spatial2D", origin=self.entities.spatial2d),
                rrb.BarChartView(name="Bar Chart", origin=self.entities.bar_chart),
                rrb.DataframeView(name="Dataframe", origin=self.entities.dataframe),
                rrb.GraphView(name="Graph", origin=self.entities.graph),
                rrb.MapView(name="Map", origin=self.entities.map),
                rrb.TensorView(name="Tensor", origin=self.entities.tensor),
                rrb.TextDocumentView(name="Text Doc", origin=self.entities.text_doc),
                rrb.TimePanel(),
                rrb.Spatial2DView(origin=self.entities.image, name="Image"),
            ),
            collapse_panels=collapse_panels,
        )

@dataclass(frozen=True)
class layouts:
    AllTabs = LayoutAllTabs