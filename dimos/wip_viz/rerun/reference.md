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