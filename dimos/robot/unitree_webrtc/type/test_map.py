import pytest
from dimos.robot.unitree_webrtc.testing.mock import Mock
from dimos.robot.unitree_webrtc.testing.helpers import show3d_stream, show3d
from dimos.utils.reactive import backpressure
from dimos.robot.unitree_webrtc.type.map import splice_sphere, Map


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
