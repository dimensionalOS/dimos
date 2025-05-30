import pytest
from dimos.robot.unitree_webrtc.type.odometry import Odometry
from dimos.robot.unitree_webrtc.testing.multimock import Multimock
from dimos.utils.testing import SensorReplay


@pytest.mark.tool
def test_store_odometry_stream():
    import os
    import threading
    import time
    from dotenv import load_dotenv
    from dimos.robot.unitree_webrtc.unitree_go2 import UnitreeGo2
    from dimos.utils.testing import SensorStorage

    load_dotenv()

    robot = UnitreeGo2(ip=os.getenv("ROBOT_IP"), mode="ai")
    robot.standup()

    def streamlogger(element):
        print(element)
        return element

    SensorStorage("raw_odometry_rotate_walk").save_stream(robot.raw_odom_stream()).subscribe(print)

    shutdown_event = threading.Event()
    print("Robot is running. Press Ctrl+C to stop.")

    try:
        # Keep the main thread alive while the robot loop runs in the background
        while not shutdown_event.is_set():
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received. Shutting down...")
    finally:
        print("Stopping robot...")
        robot.liedown()
        print("Robot stopped.")


def test_odometry_stream():
    counter = 0
    for message in SensorReplay(
        name="raw_odometry_rotate_walk", autocast=lambda x: Odometry.from_msg(x)
    ).iterate():
        counter += 1
        print(message)
