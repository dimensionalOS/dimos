// Live camera via useVideo — WebRTC/WebCodecs <video>/<canvas> when available, else the JPEG Image-topic floor; auto-detects the topic.
import { type CSSProperties } from "react";
import type { MediaMode, TopicInfo } from "@dimos/react";
import { useTopics, useVideo } from "../dimos";

function pickImage(topics: TopicInfo[]): string | null {
  const prefer = ["/color_image", "/camera/image_raw", "/image", "/cam/rgb"];
  for (const p of prefer) if (topics.find((t) => t.topic === p)) return p;
  return topics.find((t) => t.type === "sensor_msgs.Image")?.topic ?? null;
}

export function CameraView({ mode, primary }: { mode?: MediaMode; primary?: boolean }) {
  const topics = useTopics();
  const topic = pickImage(topics);
  const { kind, videoRef, canvasRef, label, active, requested } = useVideo(topic, { mode });

  const fellBack = requested && requested !== "auto" && active !== requested;

  // Fullscreen the media element itself, not a CSS overlay — the overlay's stacking context would break the live stream.
  const fullscreen = () => {
    const el = (kind === "stream" ? videoRef.current : canvasRef.current) as HTMLElement | null;
    el?.requestFullscreen?.().catch(() => {});
  };

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
        </span>
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
