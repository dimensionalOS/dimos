// CameraView — live sensor_msgs.Image (jpeg or raw) painted to a canvas via the
// SDK's useImageTopic hook. Auto-detects the Image topic by type.
import type { CSSProperties } from "react";
import { useTopics, useVideo, type MediaMode, type TopicInfo } from "@dimos/react";

function pickImage(topics: TopicInfo[]): string | null {
  const prefer = ["/dimos/color_image", "/color_image", "/dimos/camera/image_raw", "/image"];
  for (const p of prefer) if (topics.find((t) => t.topic === p)) return p;
  return topics.find((t) => t.type === "sensor_msgs.Image")?.topic ?? null;
}

export function CameraView({ mode }: { mode?: MediaMode }) {
  const topics = useTopics();
  const topic = pickImage(topics);
  // useVideo negotiates the media plane: a WebRTC <video> when the gateway+browser support it,
  // else the JPEG Image-topic floor on a <canvas>. The mode toggle forces one for A/B.
  const { kind, videoRef, canvasRef, label } = useVideo(topic, { mode });

  const style: CSSProperties = {
    width: "100%",
    maxHeight: 320,
    objectFit: "contain",
    display: "block",
    borderRadius: 8,
    background: "#0b0e14",
  };
  return (
    <div className="panel">
      <div className="panel-title">
        Camera · {topic ?? "no sensor_msgs.Image"}
        {label ? ` · ${label}` : ""}
      </div>
      {kind === "stream" ? (
        <video ref={videoRef} autoPlay playsInline muted style={style} />
      ) : (
        <canvas ref={canvasRef} style={style} />
      )}
    </div>
  );
}
