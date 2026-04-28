import * as React from "react";

import ActivationPanel from "./ActivationPanel";
import Connection from "./Connection";
import ExplorePanel from "./ExplorePanel";
import GpsButton from "./GpsButton";
import Button from "./Button";
import KeyboardControlPanel from "./KeyboardControlPanel";
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

interface ServerConfig {
  viser_url: string;
  camera_stream_url: string;
}

function defaultConfig(): ServerConfig {
  // Fallback if /config is unreachable — typical local-dev case.
  return {
    viser_url: `http://${window.location.hostname}:8082`,
    camera_stream_url: "/camera_stream",
  };
}

export default function App(): React.ReactElement {
  const [state, dispatch] = React.useReducer(appReducer, initialState);
  const [isGpsMode, setIsGpsMode] = React.useState(false);
  const [config, setConfig] = React.useState<ServerConfig>(defaultConfig);
  const connectionRef = React.useRef<Connection | null>(null);

  React.useEffect(() => {
    connectionRef.current = new Connection(dispatch);

    fetch("/config")
      .then((r) => (r.ok ? r.json() : Promise.reject(new Error(`config ${r.status}`))))
      .then((c: ServerConfig) => setConfig(c))
      .catch(() => {
        // Stick with default — already viable for local dev.
      });

    return () => {
      if (connectionRef.current) {
        connectionRef.current.disconnect();
      }
    };
  }, []);

  const handleGpsGoal = React.useCallback((goal: LatLon) => {
    connectionRef.current?.sendGpsGoal(goal);
  }, []);

  const handleStartExplore = React.useCallback(() => {
    connectionRef.current?.startExplore();
  }, []);

  const handleStopExplore = React.useCallback(() => {
    connectionRef.current?.stopExplore();
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

  const handleArm = React.useCallback(() => {
    connectionRef.current?.arm();
  }, []);

  const handleDisarm = React.useCallback(() => {
    connectionRef.current?.disarm();
  }, []);

  const handleSetDryRun = React.useCallback((enabled: boolean) => {
    connectionRef.current?.setDryRun(enabled);
  }, []);

  const handleReturnHome = React.useCallback(() => {
    connectionRef.current?.worldClick(0, 0);
  }, []);

  const handleStop = React.useCallback(() => {
    if (state.robotPose) {
      connectionRef.current?.worldClick(state.robotPose.coords[0]!, state.robotPose.coords[1]!);
    }
  }, [state.robotPose]);

  return (
    <div
      style={{
        display: "grid",
        gridTemplateColumns: "1fr 360px",
        width: "100%",
        height: "100%",
        background: "#111",
        color: "#eee",
      }}
    >
      <div style={{ position: "relative", overflow: "hidden" }}>
        {isGpsMode ? (
          <LeafletMap
            gpsLocation={state.gpsLocation}
            gpsTravelGoalPoints={state.gpsTravelGoalPoints}
            onGpsGoal={handleGpsGoal}
          />
        ) : (
          <iframe
            src={config.viser_url}
            title="viser"
            style={{
              width: "100%",
              height: "100%",
              border: "none",
              display: "block",
            }}
          />
        )}
      </div>

      <div
        style={{
          display: "grid",
          gridTemplateRows: "auto 1fr",
          padding: 8,
          gap: 8,
          background: "#1a1a1a",
          borderLeft: "1px solid #333",
          minHeight: 0,
        }}
      >
        <div
          style={{
            background: "#000",
            border: "1px solid #333",
            borderRadius: 4,
            overflow: "hidden",
            aspectRatio: "16 / 9",
          }}
        >
          <img
            src={config.camera_stream_url}
            alt="robot camera"
            style={{
              width: "100%",
              height: "100%",
              objectFit: "contain",
              display: "block",
            }}
          />
        </div>

        <div
          style={{
            display: "flex",
            flexDirection: "column",
            gap: 6,
            overflowY: "auto",
            paddingRight: 4,
          }}
        >
          <KeyboardControlPanel
            onSendMoveCommand={handleSendMoveCommand}
            onStopMoveCommand={handleStopMoveCommand}
          />
          <ActivationPanel
            onArm={handleArm}
            onDisarm={handleDisarm}
            onSetDryRun={handleSetDryRun}
          />
          <ExplorePanel onStartExplore={handleStartExplore} onStopExplore={handleStopExplore} />
          <GpsButton
            onUseGps={() => setIsGpsMode(true)}
            onUseCostmap={() => setIsGpsMode(false)}
          />
          <Button onClick={handleReturnHome} isActive={false}>
            Go Home
          </Button>
          <Button onClick={handleStop} isActive={false}>
            Stop
          </Button>
        </div>
      </div>
    </div>
  );
}
