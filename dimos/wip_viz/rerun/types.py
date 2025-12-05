from typing import NamedTuple, TypeVar, Generic, Union

# rerun "msg" types
import rerun as rr # pip install rerun-sdk
import rerun.blueprint as rrb

# dimos msgs
from dimos.msgs.sensor_msgs.PointCloud2                  import PointCloud2                # — maps directly to `rr.Points3D` from numpy points.
from dimos.msgs.nav_msgs.Path                            import Path                       # — maps to `rr.LineStrips3D` from pose positions.
from dimos.msgs.sensor_msgs.Image                        import Image                      # — maps directly to `rr.Image` (RGB conversion via impl).
from dimos.msgs.geometry_msgs.Transform                  import Transform                  # — maps to `rr.Transform3D` (translation + quaternion).
from dimos.msgs.nav_msgs.Odometry                        import Odometry                   # — maps to `rr.Transform3D` from pose.
from dimos.msgs.nav_msgs.OccupancyGrid                   import OccupancyGrid              # — maps to `rr.SegmentationImage` from grid.
from dimos.msgs.geometry_msgs.Twist                      import Twist                      # — maps to `rr.Arrows3D` (linear & angular vectors).
from dimos.msgs.geometry_msgs.TwistStamped               import TwistStamped               # — maps to `rr.Arrows3D` (stamped twist).
from dimos.msgs.geometry_msgs.TwistWithCovariance        import TwistWithCovariance        # — maps to `rr.Arrows3D` (covariance ignored).
from dimos.msgs.geometry_msgs.TwistWithCovarianceStamped import TwistWithCovarianceStamped # — maps to `rr.Arrows3D` (covariance ignored).
from dimos.msgs.geometry_msgs.Vector3                    import Vector3                    
from dimos.msgs.geometry_msgs.Quaternion                 import Quaternion                 # — maps to `rr.Transform3D` (rotation only).
from dimos.msgs.sensor_msgs.Joy                          import Joy                        # — coarse mapping to `rr.AnyValues` (axes.buttons telemetry).
from dimos.msgs.sensor_msgs.CameraInfo                   import CameraInfo                 # — mapped to `rr.TextDocument` summary of intrinsics.
from dimos.msgs.foxglove_msgs.Color                      import Color                      # — maps to `rr.Color`.

AutoConvertableToRerunMsgType = TypeVar("AutoConvertableToRerunMsgType", bound=Union[PointCloud2, Path, Image, Transform, Odometry, OccupancyGrid, Twist, TwistStamped, TwistWithCovariance, TwistWithCovarianceStamped, Vector3, Quaternion, Joy, CameraInfo, Color])
RerunMsgType = TypeVar("RerunMsgType", bound=Union[rr.Arrows2D, rr.Asset3D, rr.BarChart, rr.Boxes2D, rr.Boxes3D, rr.Capsules3D, rr.Cylinders3D, rr.DepthImage, rr.Ellipsoids3D, rr.EncodedImage, rr.GeoLineStrings, rr.GeoPoints, rr.GraphEdge, rr.GraphEdges, rr.GraphNodes, rr.GraphType, rr.Image, rr.InstancePoses3D, rr.LineStrips2D, rr.LineStrips3D, rr.Mesh3D, rr.Pinhole, rr.Points2D, rr.Points3D, rr.Quaternion, rr.Scalars, rr.SegmentationImage, rr.SeriesLines, rr.SeriesPoints, rr.Tensor, rr.TextDocument, rr.TextLog, rr.Transform3D, rr.VideoStream, rr.ViewCoordinates, rr.AnyValues])
RenderTarget = TypeVar("RenderTarget", bound=Union[str, None])

# TODO: look into: rr.TensorData

class RerunRenderType(Generic[RerunMsgType, RenderTarget]):
    def __init__(self, value, target=None):
        self.value = value
        self.target = target

class RerunTypeHelper:
    def __getitem__(self, key):
        if isinstance(key, tuple):
            if key[1] != None or type(key[0]) != str:
                raise Exception(f'''When using RerunRender[first, second], `second` must be a string literal, None, or not given at all (equivalent to None)''')
            return RerunRenderType[key[0], key[1]]
        elif isinstance(key, type):
            return RerunRenderType[key, None]
        raise Exception(f'''When using RerunRender, you must either provide RerunRender[type] or RerunRender[type, "target_string"]''')
    
    def __call__(self, value, target=None):
        return RerunRenderType(value, target)


"""
Used in two ways:
1. as a type annotation tool:
    thing1 : Out[RerunRender[rr.Image]] = None
    thing2 : Out[RerunRender[rr.Image, "/camera1/image"]] = None
2. as a return value for the publish call:
    thing1.publish(
        RerunRender(
            rr.Image(np.zeros((100, 100, 3)),
            "/camera1/image"
        )
    )
"""
RerunRender = tuple
# RerunTypeHelper()


def _traceback_to_string(traceback):
    import traceback as traceback_module
    from io import StringIO
    string_stream = StringIO()
    traceback_module.print_tb(traceback, limit=None, file=string_stream)
    return string_stream.getvalue()

def _get_trace(level=0):
    import sys
    import types
    try:
        raise Exception(f'''''')
    except:
        traceback = sys.exc_info()[2]
        back_frame = traceback.tb_frame
        for each in range(level+1):
            back_frame = back_frame.f_back
    traceback = types.TracebackType(
        tb_next=None,
        tb_frame=back_frame,
        tb_lasti=back_frame.f_lasti,
        tb_lineno=back_frame.f_lineno
    )
    return traceback


class BlueprintRecord:
    def __init__(self, blueprint: rrb.Blueprint):
        self.blueprint = blueprint
        self.trace = _get_trace(level=2)
    
    # for debugging when two sources send a blueprint
    def error_trace(self):
        return _traceback_to_string(self.trace)