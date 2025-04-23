import os
import time
from dimos.robot.unitree.unitree_go2 import UnitreeGo2
from dimos.robot.unitree.unitree_ros_control import UnitreeROSControl
from dimos.web.websocket_vis.server import WebsocketVis
from dimos.robot.global_planner.planner import AstarPlanner
from dimos.types.costmap import Costmap
from dimos.types.vector import Vector
import argparse
import pickle


def parse_args():
    parser = argparse.ArgumentParser(description="Simple test for vis.")
    parser.add_argument(
        "--live",
        action="store_true",
    )
    return parser.parse_args()


def main():
    args = parse_args()

    websocket_vis = WebsocketVis()
    websocket_vis.start()

    if args.live:
        ros_control = UnitreeROSControl(node_name="simple_nav_test", mock_connection=False)
        robot = UnitreeGo2(ros_control=ros_control, ip=os.getenv("ROBOT_IP"))
        planner = robot.global_planner
    else:
        pickle_path = f"{__file__.rsplit('/', 1)[0]}/mockdata/costmap.pickle"
        print(f"Loading costmap from {pickle_path}")
        planner = AstarPlanner(
            costmap=lambda: pickle.load(open(pickle_path, "rb")),
            base_link=lambda: [Vector(6.0, -1.5), Vector(1, 1, 1)],
            local_nav=lambda x: time.sleep(1) and True,
        )

    websocket_vis.connect(planner.vis_stream())
    print(f"WebSocket server started on port {websocket_vis.port}")

    time.sleep(1)
    while True:
        planner.plan(Vector(0, 0))
        time.sleep(1)

    try:
        # Keep the server running
        while True:
            pass
    except KeyboardInterrupt:
        print("Stopping WebSocket server...")
        websocket_vis.stop()
        print("WebSocket server stopped")


if __name__ == "__main__":
    main()
