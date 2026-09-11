import { PanelFrame } from "./cockpit/PanelFrame.tsx";
import { useContext, useEffect, useState, useRef } from "react";
import type { PanelProps } from "@dimos/cockpit/panels/registry.tsx";
import { useOptionalSlot } from "@dimos/cockpit/panels/hooks.ts";
import { ViewerSession } from "./viewerSession.ts";
import { PlayerDirectory } from "./players.ts";
import { playerName, robotIds } from "./roster.ts";
import { FrameRate } from "./frameRate.ts";
import styles from "./ballCamera.module.css";

type CameraFrame = {
  robot: string;
  generation: string;
  ts: number;
  width: number;
  height: number;
  status: string;
  image: string;
  boxes: { xyxy: [number, number, number, number]; confidence: number }[];
};
export function readCameraFrame(value: unknown): CameraFrame | null {
  if (!value || typeof value !== "object") return null;
  const v = value as CameraFrame;
  if (
    !robotIds.some((id) => id === v.robot) || typeof v.generation !== "string" ||
    !Number.isFinite(v.ts) || !Number.isInteger(v.width) || v.width < 1 || v.width > 1920 ||
    !Number.isInteger(v.height) || v.height < 1 || v.height > 1080 || typeof v.image !== "string" ||
    !/^data:image\/jpeg;base64,[A-Za-z0-9+/=]+$/.test(v.image) || v.image.length > 500_000 ||
    !Array.isArray(v.boxes) || v.boxes.length > 100
  ) return null;
  if (
    !v.boxes.every((b) =>
      Array.isArray(b.xyxy) && b.xyxy.length === 4 && b.xyxy.every(Number.isFinite) &&
      Number.isFinite(b.confidence) && b.confidence >= 0 && b.confidence <= 1 && b.xyxy[0] >= 0 &&
      b.xyxy[1] >= 0 && b.xyxy[2] <= v.width && b.xyxy[3] <= v.height && b.xyxy[2] >= b.xyxy[0] &&
      b.xyxy[3] >= b.xyxy[1]
    )
  ) return null;
  return v;
}

export function BallCameraPanel({ spec, store }: PanelProps) {
  const session = useContext(ViewerSession);
  const players = useContext(PlayerDirectory);
  const [robot, setRobot] = useState(String(spec.params?.robot ?? "duck1"));
  const visible = true;
  const selectable = spec.params?.selectable === true;
  const channel = selectable ? `${robot}_ball_camera` : spec.channels[0];
  const slot = useOptionalSlot(store, channel);
  const latest = readCameraFrame(slot?.value);
  const [frame, setFrame] = useState<CameraFrame | null>(null);
  const rate = useRef(new FrameRate());
  const [fps, setFps] = useState(0);
  useEffect(() => { rate.current.reset(); setFps(0); }, [robot]);
  const [now, setNow] = useState(Date.now());
  useEffect(() => {
    const timer = setInterval(() => setNow(Date.now()), 250);
    return () => clearInterval(timer);
  }, []);
  useEffect(() => {
    if (visible && selectable && channel) return session?.subscribe(channel, () => {});
  }, [session, channel, selectable, visible]);
  useEffect(() => {
    if (!visible || !latest || latest.robot !== robot) {
      setFrame(null);
      return;
    }
    let canceled = false;
    const image = new Image();
    image.src = latest.image;
    image.decode().then(() => {
      if (!canceled) {
        setFrame(latest);
        const measured = rate.current.draw(performance.now());
        if (measured !== null) setFps(measured);
      }
    }).catch(() => {});
    return () => {
      canceled = true;
    };
  }, [slot, robot, visible]);
  const player = players.find((p) => p.id === robot);
  const fresh = frame && frame.robot === robot && frame.generation === player?.generation &&
    now - frame.ts * 1000 < 2000 && now - frame.ts * 1000 > -2000;
  return (
    <PanelFrame spec={spec} badge={<>
          {selectable && (
            <label className={styles.selector}>

              <select aria-label="YOLO camera duck"
                value={robot}
                onChange={(e) => {
                  setRobot(e.target.value);
                  setFrame(null);
                }}
              >
                {robotIds.map((id) => (
                  <option key={id} value={id}>
                    {players.find((p) => p.id === id)?.displayName || playerName(id)}
                  </option>
                ))}
              </select>
            </label>
          )}
</>}>
    <section className={styles.panel} aria-label="Football detection camera">
      {visible && (
        <>
          <div className={styles.picture}>
            <span className="mw-feed-fps" data-testid="football-camera-fps" title="Decoded detection frames per second">{fresh ? fps.toFixed(1) : "0"} FPS</span>
            {fresh
              ? (
                <>
                  <img
                    src={frame.image}
                    alt={`${player?.displayName || playerName(robot)} head camera`}
                  />
                  <svg
                    preserveAspectRatio="xMidYMid meet"
                    viewBox={`0 0 ${frame.width} ${frame.height}`}
                    aria-label={`${frame.boxes.length} ball detections`}
                  >
                    {frame.boxes.map((box, i) => (
                      <g key={i}>
                        <rect
                          x={box.xyxy[0]}
                          y={box.xyxy[1]}
                          width={box.xyxy[2] - box.xyxy[0]}
                          height={box.xyxy[3] - box.xyxy[1]}
                        />
                        <text x={Math.max(2, box.xyxy[0])} y={Math.max(16, box.xyxy[1] - 5)}>
                          ball {Math.round(box.confidence * 100)}%
                        </text>
                      </g>
                    ))}
                  </svg>
                </>
              )
              : (
                <p role="status">
                  {player?.generation
                    ? "Waiting for a fresh camera frame…"
                    : "This duck's camera starts when a player joins."}
                </p>
              )}
          </div>
          <p className={styles.status}>
            {fresh
              ? frame.status !== "ready"
                ? "Camera connected. Detector unavailable."
                : frame.boxes.length
                ? `${frame.boxes.length} ball${frame.boxes.length === 1 ? "" : "s"} detected`
                : "No ball detected in this frame"
              : "Camera waiting"}
          </p>
        </>
      )}
    </section>
    </PanelFrame>
  );
}
