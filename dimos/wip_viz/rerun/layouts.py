#!/usr/bin/env python3
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
    render_image              : In[RerunRender[rr.Image            , None]]  = None
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
    
    @rpc
    def start(self) -> None:
        # this runs (and the callback does too)
        self.rerun_blueprint.publish(BlueprintRecord(self.viewer_blueprint))
        
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
                
        hooks = [
            self.render_arrows2d.subscribe,
            self.render_asset3d.subscribe,
            self.render_bar_chart.subscribe,
            self.render_boxes2d.subscribe,
            self.render_boxes3d.subscribe,
            self.render_capsules3d.subscribe,
            self.render_cylinders3d.subscribe,
            self.render_depth_image.subscribe,
            self.render_ellipsoids3d.subscribe,
            self.render_encoded_image.subscribe,
            self.render_geo_line_strings.subscribe,
            self.render_geo_points.subscribe,
            self.render_graph_edge.subscribe,
            self.render_graph_edges.subscribe,
            self.render_graph_nodes.subscribe,
            self.render_graph_type.subscribe,
            self.render_image.subscribe,
            self.render_instance_poses3d.subscribe,
            self.render_line_strips2d.subscribe,
            self.render_line_strips3d.subscribe,
            self.render_mesh3d.subscribe,
            self.render_pinhole.subscribe,
            self.render_points2d.subscribe,
            self.render_points3d.subscribe,
            self.render_quaternion.subscribe,
            self.render_scalars.subscribe,
            self.render_segmentation_image.subscribe,
            self.render_series_lines.subscribe,
            self.render_series_points.subscribe,
            self.render_tensor.subscribe,
            self.render_text_document.subscribe,
            self.render_text_log.subscribe,
            self.render_transform3d.subscribe,
            self.render_video_stream.subscribe,
            self.render_view_coordinates.subscribe,
        ]
        print("[RerunAllTabsLayout] here")
        for each in hooks:
            try:
                self._disposables.add(Disposable(each(process_message)))
            except Exception as error:
                # it'll fail if a transport wasn't hooked up most won't be
                print(error)