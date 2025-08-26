import { io, Socket } from "socket.io-client";

import {
  AppAction,
  Costmap,
  EncodedCostmap,
  EncodedPath,
  EncodedVector,
  FullStateData,
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

    this.socket.on("path", (data: EncodedPath) => {
      const path = Path.decode(data);
      this.dispatch({ type: "SET_PATH", payload: path });
    });

    this.socket.on("full_state", (data: FullStateData) => {
      const state: Partial<{ costmap: Costmap; robotPose: Vector; path: Path }> = {};

      if (data.costmap != undefined) {
        state.costmap = Costmap.decode(data.costmap);
      }
      if (data.robot_pose != undefined) {
        state.robotPose = Vector.decode(data.robot_pose);
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
