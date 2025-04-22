import time
from dimos.web.websocket_vis.server import WebsocketVis
from dimos.robot.global_planner.planner import AstarPlanner
from dimos.types.costmap import Costmap
from dimos.types.vector import Vector


def main():
    # Start the WebSocket server
    websocket_vis = WebsocketVis()
    websocket_vis.start()

    planner = AstarPlanner(
        costmap=lambda: Costmap.from_pickle(f"{__file__.rsplit('/', 1)[0]}/mockdata/costmap.pickle"),
        base_link=lambda: [Vector(1, 1, 0), Vector(1, 1, 1)],
        local_nav=lambda x: time.sleep(1) and True,
    )

    websocket_vis.connect(planner.vis_stream())

    time.sleep(1)  # Allow time for the server to start
    planner.plan(Vector([5, 5, 0]))
    planner.vis("testvector", Vector([0, 0, 0]))

    print(f"WebSocket server started on port {websocket_vis.port}")
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
