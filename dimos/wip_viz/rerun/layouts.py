#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

import rerun as rr  # pip install rerun-sdk
import rerun.blueprint as rrb
from dimos.wip_viz.rerun.types import RerunRender, BlueprintRecord
from dimos.core import In, Out, Module, rpc
from reactivex.disposable import Disposable

import secrets
import string

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

class RerunAllTabsLayout(Module):
    rerun_blueprint : Out[BlueprintRecord] = None
    
    # TODO: not sure that autoconnect is going to like the way the types are done here, especially the None vs "/entity/address" differences
    # Takes (basically) every possible rerun message type
    render_arrows2d           : In[RerunRender[rr.Arrows2D         , None]]  = None
    render_asset3d            : In[RerunRender[rr.Asset3D          , None]]  = None
    render_bar_chart          : In[RerunRender[rr.BarChart         , None]]  = None
    render_boxes2d            : In[RerunRender[rr.Boxes2D          , None]]  = None
    render_boxes3d            : In[RerunRender[rr.Boxes3D          , None]]  = None
    render_capsules3d         : In[RerunRender[rr.Capsules3D       , None]]  = None
    render_cylinders3d        : In[RerunRender[rr.Cylinders3D      , None]]  = None
    render_depth_image        : In[RerunRender[rr.DepthImage       , None]]  = None
    render_ellipsoids3d       : In[RerunRender[rr.Ellipsoids3D     , None]]  = None
    render_encoded_image      : In[RerunRender[rr.EncodedImage     , None]]  = None
    render_geo_line_strings   : In[RerunRender[rr.GeoLineStrings   , None]]  = None
    render_geo_points         : In[RerunRender[rr.GeoPoints        , None]]  = None
    render_graph_edge         : In[RerunRender[rr.GraphEdge        , None]]  = None
    render_graph_edges        : In[RerunRender[rr.GraphEdges       , None]]  = None
    render_graph_nodes        : In[RerunRender[rr.GraphNodes       , None]]  = None
    render_graph_type         : In[RerunRender[rr.GraphType        , None]]  = None
    # render_image              : In[RerunRender[rr.Image            , None]]  = None
    render_instance_poses3d   : In[RerunRender[rr.InstancePoses3D  , None]]  = None
    render_line_strips2d      : In[RerunRender[rr.LineStrips2D     , None]]  = None
    render_line_strips3d      : In[RerunRender[rr.LineStrips3D     , None]]  = None
    render_mesh3d             : In[RerunRender[rr.Mesh3D           , None]]  = None
    render_pinhole            : In[RerunRender[rr.Pinhole          , None]]  = None
    render_points2d           : In[RerunRender[rr.Points2D         , None]]  = None
    render_points3d           : In[RerunRender[rr.Points3D         , None]]  = None
    render_quaternion         : In[RerunRender[rr.Quaternion       , None]]  = None
    render_scalars            : In[RerunRender[rr.Scalars          , None]]  = None
    render_segmentation_image : In[RerunRender[rr.SegmentationImage, None]]  = None
    render_series_lines       : In[RerunRender[rr.SeriesLines      , None]]  = None
    render_series_points      : In[RerunRender[rr.SeriesPoints     , None]]  = None
    render_tensor             : In[RerunRender[rr.Tensor           , None]]  = None
    render_text_document      : In[RerunRender[rr.TextDocument     , None]]  = None
    render_text_log           : In[RerunRender[rr.TextLog          , None]]  = None
    render_transform3d        : In[RerunRender[rr.Transform3D      , None]]  = None
    render_video_stream       : In[RerunRender[rr.VideoStream      , None]]  = None
    render_view_coordinates   : In[RerunRender[rr.ViewCoordinates  , None]]  = None
    
    types_to_entities : dict[type, str] = {
        rr.Arrows2D:          "/arrows2d",
        rr.Asset3D:           "/spatial3d/asset3d",
        rr.BarChart:          "/bar_chart",
        rr.Boxes2D:           "/boxes2d",
        rr.Boxes3D:           "/spatial3d/boxes3d",
        rr.Capsules3D:        "/spatial3d/capsules3d",
        rr.Cylinders3D:       "/spatial3d/cylinders3d",
        rr.DepthImage:        "/depth_image",
        rr.Ellipsoids3D:      "/spatial3d/ellipsoids3d",
        rr.EncodedImage:      "/encoded_image",
        rr.GeoLineStrings:    "/geo_line_strings",
        rr.GeoPoints:         "/geo_points",
        rr.GraphEdge:         "/graph_edge",
        rr.GraphEdges:        "/graph_edges",
        rr.GraphNodes:        "/graph_nodes",
        rr.GraphType:         "/graph_type",
        rr.Image:             "/image",
        rr.InstancePoses3D:   "/spatial3d/instance_poses3d",
        rr.LineStrips2D:      "/line_strips2d",
        rr.LineStrips3D:      "/spatial3d/line_strips3d",
        rr.Mesh3D:            "/spatial3d/mesh3d",
        rr.Pinhole:           "/pinhole",
        rr.Points2D:          "/points2d",
        rr.Points3D:          "/spatial3d/points3d",
        rr.Quaternion:        "/quaternion",
        rr.Scalars:           "/scalars",
        rr.SegmentationImage: "/segmentation_image",
        rr.SeriesLines:       "/series_lines",
        rr.SeriesPoints:      "/series_points",
        rr.Tensor:            "/tensor",
        rr.TextDocument:      "/text_document",
        rr.TextLog:           "/text_log",
        # rr.Transform3D:       "/transform3d", # TODO: this one really only makes sense if its targeting some other entity
        rr.VideoStream:       "/video_stream",
        # rr.ViewCoordinates:   "/view_coordinates", # this is kinda "/world"
        # rr.CoordinateFrame:   "/coordinate_frame", # this is kinda "/world/frame"
        
        # FIXME: finish wiring this up to picking an entity
        
        
    }
    
    def __init__(self, *args, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(*args, **kwargs)
        self.viewer_blueprint = rrb.Blueprint(
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
                rrb.Spatial2DView(origin="image", name="Image"),
            ),
            collapse_panels=False,
        )
    
    render_image : In[RerunRender[rr.Image            , None]]  = None
    
    @rpc
    def start(self) -> None:
        # this runs (and the callback does too)
        self.rerun_blueprint.publish(BlueprintRecord(self.viewer_blueprint))
        
        # this callback never runs!
        self.render_image.subscribe(lambda *args: print(f"[RerunAllTabsLayout] got a message! {args}"))
        
        
        
        
        
        # this tells the DimOsDashboard what blueprint to render
        # FIXME: need to eventually 1). publish what types can be rendered / not rendered 2). mention what targets are available (ex: multiple camera streams)
        def process_message(message_value):
            print(f"[RerunAllTabsLayout] got a message! {message_value}")
            # FIXME: we kinda need a way to know what module is sending the message. If we knew (ex: camera) then we could default to one entity per module instead of per message type
            # NOTE: we can kinda compensate for this by the inherited base class, using the class name as the entity name
            if isinstance(message_value, (RerunRender, tuple)): # TODO: debatable if tuple should be supported here
                value, target = message_value
                print(f"[RerunAllTabsLayout] sending {value} to {target}")
                rr.log(target, value) # ex: rr.log("path", rr.GeoPoints(lat_lon=[some_coordinate], colors=[0xFF0000FF]))
            else:
                # FIXME: guess an entity target based on the type
                rr.log(None, message_value)
        
        # self._disposables.add(Disposable(self.render_arrows2d.subscribe(process_message)))
        # self._disposables.add(Disposable(self.render_asset3d.subscribe(process_message)))
        # self._disposables.add(Disposable(self.render_bar_chart.subscribe(process_message)))
        # self._disposables.add(Disposable(self.render_boxes2d.subscribe(process_message)))
        # self._disposables.add(Disposable(self.render_boxes3d.subscribe(process_message)))
        # self._disposables.add(Disposable(self.render_capsules3d.subscribe(process_message)))
        # self._disposables.add(Disposable(self.render_cylinders3d.subscribe(process_message)))
        # self._disposables.add(Disposable(self.render_depth_image.subscribe(process_message)))
        # self._disposables.add(Disposable(self.render_ellipsoids3d.subscribe(process_message)))
        # self._disposables.add(Disposable(self.render_encoded_image.subscribe(process_message)))
        # self._disposables.add(Disposable(self.render_geo_line_strings.subscribe(process_message)))
        # self._disposables.add(Disposable(self.render_geo_points.subscribe(process_message)))
        # self._disposables.add(Disposable(self.render_graph_edge.subscribe(process_message)))
        # self._disposables.add(Disposable(self.render_graph_edges.subscribe(process_message)))
        # self._disposables.add(Disposable(self.render_graph_nodes.subscribe(process_message)))
        # self._disposables.add(Disposable(self.render_graph_type.subscribe(process_message)))
        # self._disposables.add(Disposable(self.render_image.subscribe(process_message)))
        # self._disposables.add(Disposable(self.render_instance_poses3d.subscribe(process_message)))
        # self._disposables.add(Disposable(self.render_line_strips2d.subscribe(process_message)))
        # self._disposables.add(Disposable(self.render_line_strips3d.subscribe(process_message)))
        # self._disposables.add(Disposable(self.render_mesh3d.subscribe(process_message)))
        # self._disposables.add(Disposable(self.render_pinhole.subscribe(process_message)))
        # self._disposables.add(Disposable(self.render_points2d.subscribe(process_message)))
        # self._disposables.add(Disposable(self.render_points3d.subscribe(process_message)))
        # self._disposables.add(Disposable(self.render_quaternion.subscribe(process_message)))
        # self._disposables.add(Disposable(self.render_scalars.subscribe(process_message)))
        # self._disposables.add(Disposable(self.render_segmentation_image.subscribe(process_message)))
        # self._disposables.add(Disposable(self.render_series_lines.subscribe(process_message)))
        # self._disposables.add(Disposable(self.render_series_points.subscribe(process_message)))
        # self._disposables.add(Disposable(self.render_tensor.subscribe(process_message)))
        # self._disposables.add(Disposable(self.render_text_document.subscribe(process_message)))
        # self._disposables.add(Disposable(self.render_text_log.subscribe(process_message)))
        # self._disposables.add(Disposable(self.render_transform3d.subscribe(process_message)))
        # self._disposables.add(Disposable(self.render_video_stream.subscribe(process_message)))
        # self._disposables.add(Disposable(self.render_view_coordinates.subscribe(process_message)))
        
# def main() -> None:
#     parser = argparse.ArgumentParser(description="Display an image file in Rerun.")
#     parser.add_argument(
#         "--path",
#         type=pathlib.Path,
#         required=True,
#         help="Path to a PNG/JPEG/etc. image.",
#     )
#     rr.script_add_args(parser)
#     args = parser.parse_args()

#     rr.script_setup(args, "rerun_example_image_from_file")

#     # Ensure the viewer opens with a 2D view focused on the `image` entity.
#     rr.send_blueprint(rrb.Spatial2DView(origin="image", name="Image"))

#     pil_image = Image.open(args.path)
#     rr.log("image", rr.Image(np.array(pil_image)))

#     rr.script_teardown(args)


# if __name__ == "__main__":
#     rr.init("rerun_mega_blueprint", spawn=True)
#     rr.send_blueprint(build_mega_blueprint())
