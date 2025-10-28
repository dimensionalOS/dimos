import { io, Socket } from "socket.io-client";

import {
  AppAction,
  Costmap,
  EncodedCostmap,
  EncodedPath,
  EncodedVector,
  FullStateData,
  LatLon,
  Path,
  TwistCommand,
  Vector,
} from "./types";

export default class Connection {
  socket: Socket;
  dispatch: React.Dispatch<AppAction>;

  constructor(dispatch: React.Dispatch<AppAction>) {
    this.dispatch = dispatch;
    this.socket = io("ws://localhost:7779");

    this.socket.on("costmap", (data: EncodedCostmap) => {
      const costmap = Costmap.decode(data);
      this.dispatch({ type: "SET_COSTMAP", payload: costmap });
    });

    this.socket.on("robot_pose", (data: EncodedVector) => {
      const robotPose = Vector.decode(data);
      this.dispatch({ type: "SET_ROBOT_POSE", payload: robotPose });
    });

    this.socket.on("gps_location", (data: LatLon) => {
      this.dispatch({ type: "SET_GPS_LOCATION", payload: data });
    });

    this.socket.on("gps_travel_goal_points", (data: LatLon[]) => {
      this.dispatch({ type: "SET_GPS_TRAVEL_GOAL_POINTS", payload: data });
    });

    this.socket.on("path", (data: EncodedPath) => {
      const path = Path.decode(data);
      this.dispatch({ type: "SET_PATH", payload: path });
    });

    this.socket.on("full_state", (data: FullStateData) => {
      const state: Partial<{ costmap: Costmap; robotPose: Vector; gpsLocation: LatLon; gpsTravelGoalPoints: LatLon[]; path: Path }> = {};

      if (data.costmap != undefined) {
        state.costmap = Costmap.decode(data.costmap);
      }
      if (data.robot_pose != undefined) {
        state.robotPose = Vector.decode(data.robot_pose);
      }
      if (data.gps_location != undefined) {
        state.gpsLocation = data.gps_location;
      }
      if (data.path != undefined) {
        state.path = Path.decode(data.path);
      }

      this.dispatch({ type: "SET_FULL_STATE", payload: state });
    });
  }

  worldClick(worldX: number, worldY: number): void {
    this.socket.emit("click", [worldX, worldY]);
  }

  startExplore(): void {
    this.socket.emit("start_explore");
  }

  stopExplore(): void {
    this.socket.emit("stop_explore");
  }

  sendMoveCommand(linear: [number, number, number], angular: [number, number, number]): void {
    const twist: TwistCommand = {
      linear: {
        x: linear[0],
        y: linear[1],
        z: linear[2],
      },
      angular: {
        x: angular[0],
        y: angular[1],
        z: angular[2],
      },
    };
    this.socket.emit("move_command", twist);
  }

  sendGpsGoal(goal: LatLon): void {
    this.socket.emit("gps_goal", goal);
  }

  stopMoveCommand(): void {
    const twist: TwistCommand = {
      linear: { x: 0, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: 0 },
    };
    this.socket.emit("move_command", twist);
  }

  disconnect(): void {
    this.socket.disconnect();
  }
}
