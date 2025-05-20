import time
import pytest

from reactivex import operators as ops

from dimos.utils.reactive import backpressure
from dimos.robot.unitree_webrtc.testing.helpers import show3d_stream
from dimos.web.websocket_vis.server import WebsocketVis
from dimos.robot.unitree_webrtc.type.map import Map
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.robot.unitree_webrtc.type.odometry import Odometry
from dimos.robot.unitree_webrtc.type.timeseries import to_datetime
from dimos.robot.unitree_webrtc.testing.multimock import Multimock

@pytest.mark.needsdata
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
    time.sleep(5)


@pytest.mark.needsdata
def test_clock_mismatch():
    for odometry_raw in Multimock("athens_odom").iterate():
        print(
            odometry_raw.ts - to_datetime(odometry_raw.data["data"]["header"]["stamp"]),
            odometry_raw.data["data"]["header"]["stamp"],
        )


@pytest.mark.needsdata
def test_odom_stream():
    for odometry_raw in Multimock("athens_odom").iterate():
        print(Odometry.from_msg(odometry_raw.data))


@pytest.mark.needsdata
def test_lidar_stream():
    for lidar_raw in Multimock("athens_lidar").iterate():
        lidarmsg = LidarMessage.from_msg(lidar_raw.data)
        print(lidarmsg)
        print(lidar_raw)


@pytest.mark.needsdata
def test_multimock_timeseries():
    odom = Odometry.from_msg(Multimock("athens_odom").load_one(1).data)
    lidar_raw = Multimock("athens_lidar").load_one(1).data
    lidar = LidarMessage.from_msg(lidar_raw)
    map = Map()
    map.add_frame(lidar)
    print(odom)
    print(lidar)
    print(lidar_raw)
    print(map.costmap)


@pytest.mark.needsdata
def test_origin_changes():
    for lidar_raw in Multimock("athens_lidar").iterate():
        print(LidarMessage.from_msg(lidar_raw.data).origin)


@pytest.mark.needsdata
@pytest.mark.vis
def test_webui_multistream():
    websocket_vis = WebsocketVis()
    websocket_vis.start()

    odom_stream = Multimock("athens_odom").stream().pipe(ops.map(Odometry.from_msg))
    lidar_stream = backpressure(Multimock("athens_lidar").stream().pipe(ops.map(LidarMessage.from_msg)))

    map = Map()
    map_stream = map.consume(lidar_stream)

    costmap_stream = map_stream.pipe(ops.map(lambda x: ["costmap", map.costmap.smudge(preserve_unknown=False)]))

    websocket_vis.connect(costmap_stream)
    websocket_vis.connect(odom_stream.pipe(ops.map(lambda pos: ["robot_pos", pos.pos.to_2d()])))

    show3d_stream(lidar_stream, clearframe=True).run()
