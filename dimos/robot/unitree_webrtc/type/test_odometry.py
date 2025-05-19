from dimos.robot.unitree_webrtc.type.odometry import Odometry
from dimos.robot.unitree_webrtc.testing.multimock import Multimock


def test_odometry_time():
    (timestamp, odom_raw) = Multimock("athens_odom").load_one(33)
    print("RAW MSG", odom_raw)
    print(Odometry.from_msg(odom_raw))
