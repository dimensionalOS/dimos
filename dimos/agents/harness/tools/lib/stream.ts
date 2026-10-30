import type { ChannelSpec, Session, Slot } from "@dimos/sdk";
import type { AgentTool } from "../base.ts";
import type { RuntimeClient } from "../../dimos/client.ts";
import type { FileRenderer } from "../../../shared/render/render.ts";
import type { FrameRecord } from "../../../sdk/streams/base.ts";
import { ChannelSchema } from "../../../shared/protocol/streams.ts";
import { Type } from "typebox";
import { mkdir, writeFile } from "node:fs/promises";
import { join } from "node:path";
import { createHash } from "node:crypto";
import { agentTool } from "./tool.ts";
import { wireDecoder } from "../../dimos/wire/decoder.ts";
import { channelFor } from "../../../sdk/relay.ts";
import { toFrame } from "../../../sdk/streams/remote.ts";
import { epochMs } from "../../../shared/protocol/time.ts";
import { gridFrame } from "../../../shared/render/frames.ts";

const Parameters = Type.Object({
  op: Type.Union([Type.Literal("list"), Type.Literal("read")]),
  channel: Type.Optional(ChannelSchema),
  mode: Type.Optional(Type.Union([Type.Literal("latest"), Type.Literal("next")])),
  timeout_s: Type.Optional(Type.Number({ exclusiveMinimum: 0, maximum: 100, default: 5 })),
  export: Type.Optional(Type.Boolean({ description: "Save and show an image, cloud or grid preview." })),
});

const argumentsDecoder = wireDecoder("stream_get arguments", Parameters);

/** Wait on the SDK's stores without another polling loop or copy of their state. */
async function waitFor<T>(
  read: () => T | null,
  subscribe: (changed: () => void) => () => void,
  signal: AbortSignal,
): Promise<T> {
  signal.throwIfAborted();
  let stop = () => {};
  let abort = () => {};
  try {
    return await new Promise<T>((resolve, reject) => {
      const changed = () => {
        try {
          const value = read();
          if (value !== null) resolve(value);
        } catch (error) {
          reject(error);
        }
      };
      abort = () => reject(signal.reason);
      signal.addEventListener("abort", abort, { once: true });
      stop = subscribe(changed);
      changed();
    });
  } finally {
    stop();
    signal.removeEventListener("abort", abort);
  }
}

async function readChannel(
  session: Session,
  spec: ChannelSpec,
  mode: "latest" | "next",
  signal: AbortSignal,
): Promise<Slot> {
  const previous = session.store.get(spec.ch);
  const stop = session.subscribe(spec.ch, () => {});
  try {
    return await waitFor(
      () => {
        const current = channelFor(session, spec.ch);
        if (current === undefined || current.encoding !== spec.encoding)
          throw new Error(`Relay channel ${spec.ch} changed or became unavailable during the read`);
        const slot = session.store.get(spec.ch);
        return mode === "next" && slot === previous ? null : slot;
      },
      (changed) => {
        const stopStore = session.store.subscribe(spec.ch, changed);
        const stopStatus = session.status.subscribe(changed);
        return () => {
          stopStore();
          stopStatus();
        };
      },
      signal,
    );
  } finally {
    stop();
  }
}

async function savePreview(frame: FrameRecord, directory: string): Promise<string> {
  let bytes: Uint8Array;
  let extension: string;
  switch (frame.kind) {
    case "image":
      bytes = frame.jpeg;
      extension = "jpg";
      break;
    case "cloud": {
      const points: number[][] = [];
      for (let i = 0; i + 2 < frame.xyz.length; i += 3) points.push(Array.from(frame.xyz.subarray(i, i + 3)));
      bytes = Buffer.from(JSON.stringify({ points }));
      extension = "points.json";
      break;
    }
    case "grid":
      bytes = await gridFrame(frame.cells, frame.width, frame.height, { width: frame.width, height: frame.height });
      extension = "png";
      break;
    case "error":
      throw new Error(frame.text);
    default:
      throw new Error("This stream has no image, point-cloud or grid preview to export");
  }
  const path = join(directory, `${createHash("sha256").update(bytes).digest("hex")}.${extension}`);
  await mkdir(directory, { recursive: true });
  await writeFile(path, bytes);
  return path;
}

export function streamTool(options: {
  readonly runtime: RuntimeClient;
  readonly renderer: FileRenderer;
  readonly exportsDir: string;
}): AgentTool<"stream_get"> {
  return agentTool(
    "stream_get",
    {
      name: "stream_get",
      description:
        "Discover streams with op=list, then use op=read with an exact returned channel ID. Latest returns the SDK's current frame; next waits for a new frame. Subscriptions close after the read. Export uses the relay's preview resolution.",
      parameters: Parameters,
      execute: async (args, _api, context) => {
        try {
          const input = argumentsDecoder.decode(args);
          const timeout = AbortSignal.timeout(Math.ceil((input.timeout_s ?? 5) * 1000));
          const signal = context.abortSignal === undefined ? timeout : AbortSignal.any([context.abortSignal, timeout]);
          const session = await options.runtime.relay(undefined, signal);
          const manifest = await waitFor(
            () => session.status.get().manifest,
            (changed) => session.status.subscribe(changed),
            signal,
          );
          if (input.op === "list")
            return {
              output: [
                {
                  type: "text",
                  text: JSON.stringify({
                    channels: manifest.channels
                      .filter((spec) => spec.dir === "rx")
                      .map((spec) => ({ channel: spec.ch, encoding: spec.encoding })),
                  }),
                },
              ],
              isError: false,
              details: { render: null },
            };
          const channel = input.channel;
          if (channel === undefined) throw new Error("op=read requires a channel ID from op=list");
          const spec = await waitFor(
            () => channelFor(session, channel) ?? null,
            (changed) => session.status.subscribe(changed),
            signal,
          );
          const slot = await readChannel(session, spec, input.mode ?? "latest", signal);
          const path = input.export
            ? await savePreview(await toFrame(channel, spec, slot, epochMs(Date.now())), options.exportsDir)
            : null;
          const rendered = path === null ? null : await options.renderer.render([{ path, sha256: null }], channel);
          return {
            output: [
              {
                type: "text",
                text: JSON.stringify({
                  status: "ok",
                  channel,
                  seq: slot.seq,
                  ts: slot.ts,
                  preview: slot.preview ?? null,
                  export: path,
                }),
              },
              ...(rendered === null ? [] : [{ type: "text" as const, text: rendered.text }, ...rendered.images]),
            ],
            isError: false,
            details: { render: rendered?.details ?? null },
          };
        } catch (error) {
          context.abortSignal?.throwIfAborted();
          return { output: [{ type: "text", text: String(error) }], isError: true, details: { render: null } };
        }
      },
    },
    { replay: "safe", settle: null, fullTextAt: null, wholeResult: null, background: { details: { render: null } } },
  );
}
