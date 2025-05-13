import pytest
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.robot.unitree_webrtc.type.odometry import Odometry
from dimos.robot.unitree_webrtc.testing.multimock import Multimock
from dimos.utils.reactive import backpressure
from dimos.robot.unitree_webrtc.type.map import Map
from dimos.robot.unitree_webrtc.testing.helpers import show3d_stream
from reactivex import operators as ops


@pytest.mark.vis
def test_multimock_stream():
    backpressure(Multimock("athens_odom").stream().pipe(ops.map(Odometry.from_msg))).subscribe(lambda x: print(x))
    map = Map()

    def lidarmsg(msg):
        frame = LidarMessage.from_msg(msg)
        map.add_frame(frame)
        return [map, map.costmap.smudge()]

    mapstream = Multimock("athens_lidar").stream().pipe(ops.map(lidarmsg))
    show3d_stream(mapstream.pipe(ops.map(lambda x: x[0])), clearframe=True).run()


def test_multimock_timeseries():
    print(Multimock("athens_odom"))

