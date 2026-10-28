import frames from "../tests/fixtures/cdr_frames.json";
import { connect, createDecoderRegistry } from "@dimos/sdk";
import type { ChannelSpec } from "@dimos/shared";

function renderFixture(): void {
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
}

function renderLive(): void {
  const messages = document.querySelector<HTMLSelectElement>("#message")!;
  const endian = document.querySelector<HTMLSelectElement>("#endian")!;
  const status = document.querySelector<HTMLElement>("#status")!;
  const values = new Map<string, { value: unknown; frames: number; errors: number }>();
  const subscriptions = new Map<string, () => void>();
  endian.disabled = true;
  const session = connect({
    robot: "cdr-demo",
    url: new URLSearchParams(location.search).get("relay") ?? undefined,
  });
  function render(): void {
    const manifest = session.status.get().manifest;
    const channel = manifest?.channels.find((item) => item.ch === messages.value);
    const slot = values.get(messages.value);
    status.textContent = `Live relay: ${session.status.get().transport.phase}. ` +
      `Received ${values.size}/${subscriptions.size} channels. ` +
      [...values].map(([name, data]) => `${name}: ${data.frames} frames, ${data.errors} errors`)
        .join("; ");
    document.querySelector("#type")!.textContent = channel?.encoding ?? "Waiting for manifest";
    document.querySelector("#schema")!.textContent = JSON.stringify(channel?.params.cdr, null, 2) ??
      "";
    document.querySelector("#value")!.textContent = JSON.stringify(
      slot?.value,
      (_key, value: unknown) => typeof value === "bigint" ? `${value}n` : value,
      2,
    ) ?? "Waiting for a live frame";
    const canvas = document.querySelector<HTMLCanvasElement>("#image")!;
    canvas.hidden = true;
    const image = slot?.value;
    if (
      image && typeof image === "object" && "data" in image && image.data instanceof Uint8Array &&
      "width" in image && typeof image.width === "number" && "height" in image &&
      typeof image.height === "number" &&
      "step" in image && typeof image.step === "number" && "encoding" in image &&
      image.encoding === "rgb8" &&
      image.width > 0 && image.width <= 512 && image.height > 0 && image.height <= 512 &&
      image.data.length === image.height * image.step && image.step >= image.width * 3
    ) {
      canvas.width = image.width;
      canvas.height = image.height;
      const rgba = new Uint8ClampedArray(image.width * image.height * 4);
      for (let y = 0; y < image.height; y++) {
        for (let x = 0; x < image.width; x++) {
          const offset = (y * image.width + x) * 4;
          rgba.set(image.data.subarray(y * image.step + x * 3, y * image.step + x * 3 + 3), offset);
          rgba[offset + 3] = 255;
        }
      }
      canvas.getContext("2d")!.putImageData(new ImageData(rgba, image.width, image.height), 0, 0);
      canvas.hidden = false;
    }
  }
  session.status.subscribe(() => {
    for (const channel of session.status.get().manifest?.channels ?? []) {
      if (subscriptions.has(channel.ch)) continue;
      messages.add(new Option(channel.ch, channel.ch));
      subscriptions.set(
        channel.ch,
        session.subscribe(channel.ch, (snapshot) => {
          if (snapshot.slot) {
            values.set(channel.ch, {
              value: snapshot.slot.value,
              frames: snapshot.stats.frames,
              errors: snapshot.stats.decodeErrors,
            });
          }
          render();
        }),
      );
    }
    render();
  });
  messages.onchange = render;
  globalThis.addEventListener("pagehide", () => {
    for (const stop of subscriptions.values()) stop();
    session.close();
  }, { once: true });
  render();
}

if (new URLSearchParams(location.search).has("live")) {
  renderLive();
} else {
  renderFixture();
}
