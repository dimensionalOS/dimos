// CameraView — live sensor_msgs.Image (jpeg or raw) painted to a canvas via the
// SDK's useImageTopic hook. Auto-detects the Image topic by type.
import { useImageTopic, useTopics, type TopicInfo } from "@dimos/react";

function pickImage(topics: TopicInfo[]): string | null {
  const prefer = ["/dimos/color_image", "/color_image", "/dimos/camera/image_raw", "/image"];
  for (const p of prefer) if (topics.find((t) => t.topic === p)) return p;
  return topics.find((t) => t.type === "sensor_msgs.Image")?.topic ?? null;
}

export function CameraView() {
  const topics = useTopics();
  const topic = pickImage(topics);
  const { canvasRef, info } = useImageTopic(topic, { maxFps: 20 });

  return (
    <div className="panel">
      <div className="panel-title">
        Camera · {topic ?? "no sensor_msgs.Image"}
        {info ? ` · ${info.width}×${info.height} ${info.encoding} ${info.fps}fps` : ""}
      </div>
      <canvas
        ref={canvasRef}
        style={{
          width: "100%",
          maxHeight: 320,
          objectFit: "contain",
          display: "block",
          borderRadius: 8,
          background: "#0b0e14",
        }}
      />
    </div>
  );
}
