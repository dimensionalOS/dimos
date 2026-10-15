import frames from "../../shared/fixtures/cdr_frames.json";
import { createDecoderRegistry } from "@dimos/sdk";
import type { ChannelSpec } from "@dimos/shared";

const registry = createDecoderRegistry();
const messages = document.querySelector<HTMLSelectElement>("#message")!;
const endian = document.querySelector<HTMLSelectElement>("#endian")!;
for (const [index, vector] of frames.vectors.entries()) {
  messages.add(new Option(vector.name, String(index)));
}
function render(): void {
  const vector = frames.vectors[Number(messages.value)];
  const channel: ChannelSpec = {
    ch: vector.name,
    dir: "rx",
    encoding: vector.encoding,
    delivery: "reliable",
    maxHz: 1,
    params: { cdr: vector.schema },
    publish: "none",
    requiredScope: null,
  };
  const encoded = endian.value === "big" ? vector.big_endian_b64 : vector.payload_b64;
  const payload = Uint8Array.from(atob(encoded), (c) => c.charCodeAt(0));
  const decoded = registry.resolve(channel)!(payload, {
    ch: vector.name,
    seq: 1,
    ts: 0,
    delivery: "reliable",
  });
  document.querySelector("#status")!.textContent =
    `Decoded ${payload.length} CDR bytes · ${endian.value} endian`;
  document.querySelector("#type")!.textContent = vector.schema.type;
  document.querySelector("#schema")!.textContent = vector.schema.definition;
  document.querySelector("#value")!.textContent = JSON.stringify(
    decoded.value,
    (_key, value) =>
      typeof value === "bigint"
        ? `${value}n`
        : ArrayBuffer.isView(value)
        ? Array.from(value as unknown as ArrayLike<number>)
        : value,
    2,
  );
  const canvas = document.querySelector<HTMLCanvasElement>("#image")!;
  canvas.hidden = vector.name !== "image";
  if (!canvas.hidden) {
    const image = decoded.value as { data: Uint8Array };
    const rgba = new Uint8ClampedArray(8);
    for (let i = 0; i < 2; i++) {
      rgba.set(image.data.subarray(i * 3, i * 3 + 3), i * 4);
      rgba[i * 4 + 3] = 255;
    }
    canvas.getContext("2d")!.putImageData(new ImageData(rgba, 2, 1), 0, 0);
  }
}
messages.onchange = render;
endian.onchange = render;
render();
