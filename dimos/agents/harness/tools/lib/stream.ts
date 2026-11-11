import type { AgentTool } from "#dimos/agents/harness/tools/base.ts";
import type { RuntimeClient } from "#dimos/agents/harness/dimos/client.ts";
import type { FileRenderer } from "#dimos/agents/shared/render/render.ts";
import type { FrameRecord } from "#dimos/agents/sdk/streams/base.ts";
import { ChannelSchema } from "#dimos/agents/shared/protocol/streams.ts";
import { Type } from "typebox";
import { mkdir, writeFile } from "node:fs/promises";
import { join } from "node:path";
import { createHash } from "node:crypto";
import { agentTool } from "#dimos/agents/harness/tools/lib/tool.ts";
import { wireDecoder } from "#dimos/agents/harness/dimos/wire/decoder.ts";
import { LATEST_REFRESH_MS, readTopic, toFrame } from "#dimos/agents/sdk/relay.ts";
import { gridFrame } from "#dimos/agents/shared/render/frames.ts";

const Parameters = Type.Object({
  op: Type.Union([Type.Literal("list"), Type.Literal("read")]),
  channel: Type.Optional(ChannelSchema),
  mode: Type.Optional(Type.Union([Type.Literal("latest"), Type.Literal("next")])),
  timeout_s: Type.Optional(Type.Number({ exclusiveMinimum: 0, maximum: 100, default: 5 })),
  export: Type.Optional(Type.Boolean({ description: "Save and show an image, cloud or grid preview." })),
});

const argumentsDecoder = wireDecoder("stream_get arguments", Parameters);

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
      description: `Discover streams with op=list, then use op=read with an exact returned channel ID. Latest returns the current frame; when none arrives within ${LATEST_REFRESH_MS / 1000} s it returns the last one with stale=true. Next waits for a new frame. Subscriptions close after the read. Export uses the topic provider's preview resolution.`,
      parameters: Parameters,
      execute: async (args, _api, context) => {
        try {
          const input = argumentsDecoder.decode(args);
          const timeout = AbortSignal.timeout(Math.ceil((input.timeout_s ?? 5) * 1000));
          const signal = context.abortSignal === undefined ? timeout : AbortSignal.any([context.abortSignal, timeout]);
          const session = await options.runtime.relay(undefined, signal);
          const streams = options.runtime.streams.value;
          if (input.op === "list")
            return {
              output: [
                {
                  type: "text",
                  text: JSON.stringify({
                    channels: streams.map((stream) => ({ channel: stream.channel, encoding: stream.encoding })),
                  }),
                },
              ],
              isError: false,
              details: { render: null },
            };
          const channel = input.channel;
          if (channel === undefined) throw new Error("op=read requires a channel ID from op=list");
          const stream = streams.find((stream) => stream.channel === channel);
          if (stream === undefined) throw new Error(`Unknown output topic ${channel}; use op=list`);
          const { sample, stale } = await readTopic(session, stream, input.mode ?? "latest", signal);
          const frame = await toFrame(stream, sample, 512);
          const preview = frame.kind === "value" ? frame.text : frame.kind;
          const path = input.export ? await savePreview(frame, options.exportsDir) : null;
          const rendered = path === null ? null : await options.renderer.render([{ path, sha256: null }], channel);
          return {
            output: [
              {
                type: "text",
                text: JSON.stringify({
                  status: "ok",
                  channel,
                  seq: sample.sequence,
                  ts: sample.sourceTs ?? sample.ingressTs,
                  stale,
                  preview: preview,
                  export: path,
                }),
              },
              ...(rendered === null ? [] : [{ type: "text" as const, text: rendered.text }]),
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
