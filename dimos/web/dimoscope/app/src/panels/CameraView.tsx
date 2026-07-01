// CameraView — live camera via the media plane (useVideo): a WebRTC/WebCodecs <video>/<canvas>
// when the gateway+browser support it, else the JPEG Image-topic floor. Auto-detects the topic.
import { type CSSProperties } from "react";
import type { MediaMode, TopicInfo } from "@dimos/react";
import { useTopics, useVideo } from "../dimos";
import { useCvOverlay } from "../cv/useCvOverlay";

function pickImage(topics: TopicInfo[]): string | null {
  const prefer = ["/color_image", "/camera/image_raw", "/image", "/cam/rgb"];
  for (const p of prefer) if (topics.find((t) => t.topic === p)) return p;
  return topics.find((t) => t.type === "sensor_msgs.Image")?.topic ?? null;
}

export function CameraView({ mode, primary }: { mode?: MediaMode; primary?: boolean }) {
  const topics = useTopics();
  const topic = pickImage(topics);
  // In-browser CV overlay (object detection on the decoded frames). When on, prefer the WebCodecs
  // frames path — CV needs real VideoFrames, which only the "frames" channels (webcodecs/jpeg) give.
  const cv = useCvOverlay();
  const effectiveMode = cv.enabled ? "webcodecs" : mode;
  // useVideo negotiates the media plane and reports what it ACTUALLY landed on (`active`) vs what
  // was forced (`requested`), so the panel can tell the truth when a mode falls back.
  const { kind, videoRef, canvasRef, label, active, requested } = useVideo(topic, {
    mode: effectiveMode,
    onFrame: cv.enabled ? cv.onFrame : undefined,
  });

  const fellBack = requested && requested !== "auto" && active !== requested;

  // ⛶ native fullscreen on the actual media element. Robust: no CSS overlay (a fixed overlay broke
  // against the column's animation-created stacking/containing block — it scrolled out the side or
  // got painted over by the sibling column), and it PRESERVES the element, so the live WebRTC stream
  // / WebCodecs canvas keeps playing. Esc exits.
  const fullscreen = () => {
    const el = (kind === "stream" ? videoRef.current : canvasRef.current) as HTMLElement | null;
    el?.requestFullscreen?.().catch(() => {});
  };

  // primary = the big center view; secondary = a 16:9 side strip.
  const style: CSSProperties = primary
    ? {
      width: "100%",
      height: "auto",
      maxHeight: "calc(100vh - 130px)",
      objectFit: "contain",
      display: "block",
      borderRadius: 8,
      background: "#000",
    }
    : {
      width: "100%",
      aspectRatio: "16 / 9",
      objectFit: "cover",
      display: "block",
      borderRadius: 8,
      background: "#000",
    };

  return (
    <div
      className="panel"
      style={primary ? { display: "flex", flexDirection: "column" } : undefined}
    >
      <div className="panel-title" style={{ display: "flex", alignItems: "center", gap: 8 }}>
        <span
          style={{ flex: 1, overflow: "hidden", textOverflow: "ellipsis", whiteSpace: "nowrap" }}
        >
          Camera · {topic ?? "no sensor_msgs.Image"}
          {label ? ` · ${label}` : ""}
          {fellBack && (
            <span style={{ color: "var(--accent)" }}>
              {" "}
              · ⚠ wanted {requested}, using {active}
            </span>
          )}
          {cv.enabled && kind === "stream" && (
            <span style={{ color: "var(--accent)" }}>· ⚠ CV needs webcodecs/jpeg (frames)</span>
          )}
          {cv.enabled && kind === "frames" && (
            <span className="muted">
              {" "}
              · cv: {cv.stats.loading ? "loading…" : `${cv.stats.count} obj · ${cv.stats.infMs}ms`}
            </span>
          )}
        </span>
        <button
          className={`tab ${cv.enabled ? "tab-active" : ""}`}
          onClick={cv.toggle}
          title="in-browser object detection on the camera frames (needs WebCodecs/JPEG)"
          style={{ padding: "2px 8px" }}
        >
          CV{cv.enabled ? " ✓" : ""}
        </button>
        <button
          className="tab"
          onClick={fullscreen}
          title="fullscreen (Esc to exit)"
          style={{ padding: "2px 8px" }}
        >
          ⛶ fullscreen
        </button>
      </div>
      {kind === "stream"
        ? <video ref={videoRef} autoPlay playsInline muted style={style} />
        : <canvas ref={canvasRef} style={style} />}
    </div>
  );
}
