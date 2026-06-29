// CameraView — live camera via the media plane (useVideo): a WebRTC/WebCodecs <video>/<canvas>
// when the gateway+browser support it, else the JPEG Image-topic floor. Auto-detects the topic.
import { useState, type CSSProperties } from "react";
import { useTopics, useVideo, type MediaMode, type TopicInfo } from "@dimos/react";

function pickImage(topics: TopicInfo[]): string | null {
  const prefer = ["/dimos/color_image", "/color_image", "/dimos/camera/image_raw", "/image"];
  for (const p of prefer) if (topics.find((t) => t.topic === p)) return p;
  return topics.find((t) => t.type === "sensor_msgs.Image")?.topic ?? null;
}

export function CameraView({ mode }: { mode?: MediaMode }) {
  const topics = useTopics();
  const topic = pickImage(topics);
  // useVideo negotiates the media plane and reports what it ACTUALLY landed on (`active`) vs what
  // was forced (`requested`), so the panel can tell the truth when a mode falls back.
  const { kind, videoRef, canvasRef, label, active, requested } = useVideo(topic, { mode });
  const [big, setBig] = useState(false);

  const fellBack = requested && requested !== "auto" && active !== requested;

  // inline: fills the (now wider) side column at 16:9, cover for a clean big frame.
  const inlineStyle: CSSProperties = {
    width: "100%",
    aspectRatio: "16 / 9",
    objectFit: "cover",
    display: "block",
    borderRadius: 8,
    background: "#000",
  };
  // enlarged: a near-fullscreen overlay (contain → full frame, no crop) for a proper look.
  const bigStyle: CSSProperties = {
    position: "fixed",
    inset: "4vh 4vw",
    width: "92vw",
    height: "92vh",
    objectFit: "contain",
    background: "#000",
    borderRadius: 10,
    zIndex: 50,
    boxShadow: "0 24px 90px rgba(0,0,0,.8)",
  };
  const style = big ? bigStyle : inlineStyle;
  const el =
    kind === "stream" ? (
      <video ref={videoRef} autoPlay playsInline muted style={style} />
    ) : (
      <canvas ref={canvasRef} style={style} />
    );

  return (
    <div className="panel">
      <div className="panel-title" style={{ display: "flex", alignItems: "center", gap: 8 }}>
        <span style={{ flex: 1, overflow: "hidden", textOverflow: "ellipsis", whiteSpace: "nowrap" }}>
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
          onClick={() => setBig((b) => !b)}
          title={big ? "shrink" : "enlarge to fullscreen"}
          style={{ padding: "2px 8px" }}
        >
          {big ? "× close" : "⤢ enlarge"}
        </button>
      </div>
      {big && (
        <div
          onClick={() => setBig(false)}
          title="click to close"
          style={{ position: "fixed", inset: 0, background: "rgba(0,0,0,.55)", zIndex: 49, cursor: "zoom-out" }}
        />
      )}
      {el}
    </div>
  );
}
