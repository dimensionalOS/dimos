import { useCallback, useEffect, useRef, useState } from "react";
import { io, type Socket } from "socket.io-client";
import type { MoveCommand } from "@/lib/types";

const VIS = (process.env.NEXT_PUBLIC_DIMOS_VIS ?? "").replace(/\/+$/, "");

// Go2 velocity ranges (WEBAPP-INTEGRATION.md §3).
const MAX_VX = 0.6; // m/s forward/back
const MAX_VY = 0.4; // m/s strafe (unused — joystick maps L/R to yaw)
const MAX_YAW = 1.0; // rad/s turn

/**
 * Socket.IO teleop link to the DimOS visualization server (port 7779) for
 * direct, no-LLM joystick driving. Emits `move_command` Twist payloads.
 * If NEXT_PUBLIC_DIMOS_VIS is unset, drive()/stop() are no-ops.
 */
export function useTeleop() {
  const socketRef = useRef<Socket | null>(null);
  const [connected, setConnected] = useState(false);

  useEffect(() => {
    if (!VIS) return;
    const socket = io(VIS, {
      transports: ["websocket", "polling"],
      extraHeaders: { "ngrok-skip-browser-warning": "true" },
    });
    socketRef.current = socket;
    socket.on("connect", () => setConnected(true));
    socket.on("disconnect", () => setConnected(false));
    return () => {
      socket.disconnect();
      socketRef.current = null;
    };
  }, []);

  // stick right (turn > 0) = clockwise = negative angular.z (CCW is positive).
  const drive = useCallback((m: MoveCommand) => {
    socketRef.current?.emit("move_command", {
      linear: { x: m.vx * MAX_VX, y: m.vy * MAX_VY, z: 0 },
      angular: { x: 0, y: 0, z: -m.turn * MAX_YAW },
    });
  }, []);

  const stop = useCallback(() => {
    socketRef.current?.emit("move_command", {
      linear: { x: 0, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: 0 },
    });
  }, []);

  return { connected, configured: !!VIS, drive, stop };
}
