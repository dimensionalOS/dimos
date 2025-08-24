import * as React from "react";

import Connection from "./Connection";
import ControlPanel from "./ControlPanel";
import VisualizerWrapper from "./components/VisualizerWrapper";
import { AppAction, AppState } from "./types";

function appReducer(state: AppState, action: AppAction): AppState {
  switch (action.type) {
    case "SET_COSTMAP":
      return { ...state, costmap: action.payload };
    case "SET_ROBOT_POSE":
      return { ...state, robotPose: action.payload };
    case "SET_PATH":
      return { ...state, path: action.payload };
    case "SET_FULL_STATE":
      return { ...state, ...action.payload };
    default:
      return state;
  }
}

const initialState: AppState = {
  costmap: null,
  robotPose: null,
  path: null,
};

export default function App(): React.ReactElement {
  const [state, dispatch] = React.useReducer(appReducer, initialState);
  const connectionRef = React.useRef<Connection | null>(null);

  React.useEffect(() => {
    connectionRef.current = new Connection(dispatch);

    return () => {
      if (connectionRef.current) {
        connectionRef.current.disconnect();
      }
    };
  }, []);

  const handleWorldClick = React.useCallback((worldX: number, worldY: number) => {
    connectionRef.current?.worldClick(worldX, worldY);
  }, []);

  const handleStartExplore = React.useCallback(() => {
    connectionRef.current?.startExplore();
  }, []);

  const handleStopExplore = React.useCallback(() => {
    connectionRef.current?.stopExplore();
  }, []);

  return (
    <div style={{ position: "relative", width: "100%", height: "100%" }}>
      <VisualizerWrapper data={state} onWorldClick={handleWorldClick} />
      <div style={{ position: "absolute", bottom: 0, left: 0 }}>
        <ControlPanel onStartExplore={handleStartExplore} onStopExplore={handleStopExplore} />
      </div>
    </div>
  );
}
