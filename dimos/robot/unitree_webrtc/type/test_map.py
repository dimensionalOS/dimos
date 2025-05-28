import pytest
from dimos.robot.unitree_webrtc.testing.mock import Mock
from dimos.robot.unitree_webrtc.testing.helpers import show3d_stream, show3d
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.utils.reactive import backpressure
from dimos.robot.unitree_webrtc.type.map import splice_sphere, Map
from dimos.utils.testing import SensorReplay


@pytest.mark.vis
def test_costmap_vis():
    map = Map()
    for frame in Mock("office").iterate():
        print(frame)
        map.add_frame(frame)
    costmap = map.costmap
    print(costmap)
    show3d(costmap.smudge().pointcloud, title="Costmap").run()


@pytest.mark.vis
def test_reconstruction_with_realtime_vis():
    show3d_stream(Map().consume(Mock("office").stream(rate_hz=60.0)), clearframe=True).run()


@pytest.mark.vis
def test_splice_vis():
    mock = Mock("test")
    target = mock.load("a")
    insert = mock.load("b")
    show3d(splice_sphere(target.pointcloud, insert.pointcloud, shrink=0.7)).run()


@pytest.mark.vis
def test_robot_vis():
    show3d_stream(
        Map().consume(backpressure(Mock("office").stream())),
        clearframe=True,
        title="gloal dynamic map test",
    )


def test_robot_mapping():
    lidar_stream = SensorReplay("office_lidar", autocast=lambda x: LidarMessage.from_msg(x))
    map = Map(voxel_size=0.5)
    map.consume(lidar_stream.stream(rate_hz=100.0)).subscribe(lambda x: ...)

    costmap = map.costmap

    shape = costmap.grid.shape
    assert shape[0] > 150
    assert shape[1] > 150

    assert costmap.unknown_percent > 80
    assert costmap.unknown_percent < 90

    assert costmap.free_percent > 5
    assert costmap.free_percent < 10

    assert costmap.occupied_percent > 8
    assert costmap.occupied_percent < 15
