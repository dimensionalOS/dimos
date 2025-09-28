import * as React from "react";

import Connection from "./Connection";
import ExplorePanel from "./ExplorePanel";
import GpsButton from "./GpsButton";
import KeyboardControlPanel from "./KeyboardControlPanel";
import VisualizerWrapper from "./components/VisualizerWrapper";
import LeafletMap from "./components/LeafletMap";
import { AppAction, AppState, LatLon } from "./types";

function appReducer(state: AppState, action: AppAction): AppState {
  switch (action.type) {
    case "SET_COSTMAP":
      return { ...state, costmap: action.payload };
    case "SET_ROBOT_POSE":
      return { ...state, robotPose: action.payload };
    case "SET_GPS_LOCATION":
      return { ...state, gpsLocation: action.payload };
    case "SET_GPS_TRAVEL_GOAL_POINTS":
      return { ...state, gpsTravelGoalPoints: action.payload };
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
  gpsLocation: null,
  gpsTravelGoalPoints: null,
  path: null,
};

export default function App(): React.ReactElement {
  const [state, dispatch] = React.useReducer(appReducer, initialState);
  const [isGpsMode, setIsGpsMode] = React.useState(false);
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

  const handleGpsGoal = React.useCallback((goal: LatLon) => {
    connectionRef.current?.sendGpsGoal(goal);
  }, []);

  const handleSendMoveCommand = React.useCallback(
    (linear: [number, number, number], angular: [number, number, number]) => {
      connectionRef.current?.sendMoveCommand(linear, angular);
    },
    [],
  );

  const handleStopMoveCommand = React.useCallback(() => {
    connectionRef.current?.stopMoveCommand();
  }, []);

  return (
    <div style={{ position: "relative", width: "100%", height: "100%" }}>
      {isGpsMode ? (
        <LeafletMap
          gpsLocation={state.gpsLocation}
          gpsTravelGoalPoints={state.gpsTravelGoalPoints}
          onGpsGoal={handleGpsGoal}
        />
      ) : (
        <VisualizerWrapper data={state} onWorldClick={handleWorldClick} />
      )}
      <div
        style={{
          position: "absolute",
          bottom: 0,
          left: 0,
          display: "flex",
          width: "100%",
          padding: 5,
          gap: 5,
          alignItems: "flex-end",
        }}
      >
        <GpsButton
          onUseGps={() => setIsGpsMode(true)}
          onUseCostmap={() => setIsGpsMode(false)}
        ></GpsButton>
        <ExplorePanel onStartExplore={handleStartExplore} onStopExplore={handleStopExplore} />
        <KeyboardControlPanel
          onSendMoveCommand={handleSendMoveCommand}
          onStopMoveCommand={handleStopMoveCommand}
        />
      </div>
    </div>
  );
}
