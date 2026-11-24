import assert from "node:assert/strict";
import { test } from "node:test";
import { mkdir, mkdtemp, writeFile } from "node:fs/promises";
import { join } from "node:path";
import { tmpdir } from "node:os";
import sharp from "sharp";
import { FileRenderer } from "#dimos/agents/shared/render/render.ts";
import { memoryTool, resolveRecording } from "#dimos/agents/harness/tools/lib/memory.ts";
import type { DimosInstall } from "#dimos/agents/harness/dimos/install.ts";
import type { DetailsByTool } from "#dimos/agents/shared/protocol/tools/details.ts";

const install = {
  root: "/tmp/dimos",
  python: "/tmp/python",
  cli: "/tmp/dimos",
  mcpUrl: new URL("http://127.0.0.1:9/mcp"),
  runsDir: "/tmp/runs",
  stateDir: "/tmp/state",
} as DimosInstall;

test("resolveRecording prefers recording.db in the workspace", async () => {
  const dir = await mkdtemp(join(tmpdir(), "memory-tool-"));
  await writeFile(join(dir, "recording.db"), "");
  assert.equal(await resolveRecording(dir, undefined), join(dir, "recording.db"));
});

test("the memory tool returns the hit table and attaches saved frames", async () => {
  const dir = await mkdtemp(join(tmpdir(), "memory-tool-"));
  const recording = join(dir, "recording.db");
  const frame = join(dir, "memory-frames", "t2.000_id1.jpg");
  await writeFile(recording, "");
  await mkdir(join(dir, "memory-frames"), { recursive: true });
  await writeFile(
    frame,
    await sharp({ create: { width: 8, height: 8, channels: 3, background: "#405060" } })
      .jpeg()
      .toBuffer(),
  );
  const renderer = new FileRenderer({ cacheDir: join(dir, "render"), maxCacheBytes: 2 ** 20, maxSourceBytes: 2 ** 20 });
  const tool = memoryTool({
    install,
    renderer,
    workspace: () => dir,
    query: async (request) => {
      assert.equal(request.op, "inspect");
      assert.equal(request.path, recording);
      assert.equal(request.t, 2);
      return {
        ok: true,
        op: "inspect",
        origin_ts: 1000,
        stream: "color_image",
        hits: [{ t: 2, id: 1, score: null, path: frame, caption: "person in a blue hoodie" }],
      };
    },
  });
  const result = await tool.registration.execute(
    { op: "inspect", t: 2 },
    { callId: "1" } as never,
    { abortSignal: undefined, value: undefined } as never,
  );
  assert.equal(result.isError, false);
  const text = result.output?.find((block) => block.type === "text")?.text ?? "";
  const payload = JSON.parse(text) as { hits: Array<{ t: number; caption?: string }> };
  assert.equal(payload.hits[0]?.t, 2);
  assert.equal(payload.hits[0]?.caption, undefined);
  const details = result.details as DetailsByTool["memory"] | null | undefined;
  assert.equal(details?.render?.views.length, 1);
});

test("the memory tool asks questions beside the frame and drops the caption", async () => {
  const dir = await mkdtemp(join(tmpdir(), "memory-tool-"));
  const recording = join(dir, "recording.db");
  const frame = join(dir, "memory-frames", "t2.000_id1.jpg");
  await writeFile(recording, "");
  await mkdir(join(dir, "memory-frames"), { recursive: true });
  await writeFile(
    frame,
    await sharp({ create: { width: 8, height: 8, channels: 3, background: "#405060" } })
      .jpeg()
      .toBuffer(),
  );
  const renderer = new FileRenderer({ cacheDir: join(dir, "render"), maxCacheBytes: 2 ** 20, maxSourceBytes: 2 ** 20 });
  const tool = memoryTool({
    install,
    renderer,
    workspace: () => dir,
    query: async () => ({
      ok: true,
      op: "inspect",
      origin_ts: 1000,
      stream: "color_image",
      hits: [{ t: 2, id: 1, score: null, path: frame, caption: "empty mall floor" }],
    }),
  });
  const result = await tool.registration.execute(
    { op: "inspect", t: 2, ask: "Is a fence on the route? Did the robot walk through?" },
    { callId: "1" } as never,
    { abortSignal: undefined, value: undefined } as never,
  );
  const texts = (result.output ?? []).flatMap((block) => (block.type === "text" ? [block.text] : []));
  assert.equal(JSON.parse(texts[0] ?? "{}").hits[0]?.caption, undefined);
  assert.match(texts.join("\n"), /Look at the image and answer:\nIs a fence on the route\?/);
  assert.equal(texts.some((text) => text.includes("empty mall floor")), false);
});

test("multi-t inspect attaches the montage once and asks about labeled frames", async () => {
  const dir = await mkdtemp(join(tmpdir(), "memory-tool-"));
  const recording = join(dir, "recording.db");
  const frameA = join(dir, "memory-frames", "t2.000_id1.jpg");
  const frameB = join(dir, "memory-frames", "t4.000_id2.jpg");
  const montage = join(dir, "memory-frames", "montage_2.000_4.000.jpg");
  await writeFile(recording, "");
  await mkdir(join(dir, "memory-frames"), { recursive: true });
  for (const frame of [frameA, frameB, montage]) {
    await writeFile(
      frame,
      await sharp({ create: { width: 8, height: 8, channels: 3, background: "#203040" } })
        .jpeg()
        .toBuffer(),
    );
  }
  const renderer = new FileRenderer({ cacheDir: join(dir, "render"), maxCacheBytes: 2 ** 20, maxSourceBytes: 2 ** 20 });
  const tool = memoryTool({
    install,
    renderer,
    workspace: () => dir,
    query: async (request) => {
      assert.deepEqual(request.t, [2, 4]);
      return {
        ok: true,
        op: "inspect",
        origin_ts: 1000,
        stream: "color_image",
        hits: [
          { t: 2, id: 1, score: null, path: frameA },
          { t: 4, id: 2, score: null, path: frameB },
        ],
        montage,
      };
    },
  });
  const result = await tool.registration.execute(
    { op: "inspect", t: [2, 4], ask: "Did the car move left or right?" },
    { callId: "1" } as never,
    { abortSignal: undefined, value: undefined } as never,
  );
  assert.equal(result.isError, false);
  const texts = (result.output ?? []).flatMap((block) => (block.type === "text" ? [block.text] : []));
  assert.match(texts.join("\n"), /Look at the labeled frames/);
  const details = result.details as DetailsByTool["memory"] | null | undefined;
  assert.equal(details?.render?.views.length, 1);
});
