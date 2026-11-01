import assert from "node:assert/strict";
import { createHash } from "node:crypto";
import { mkdtemp, readFile, rm, writeFile } from "node:fs/promises";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { test } from "node:test";
import sharp from "sharp";
import type { Message } from "@earendil-works/pi-ai";
import type { Sha256 } from "#dimos/agents/shared/protocol/media.ts";
import {
  PREVIEW_PREFIX,
  hydrateToolResultPreviews,
  previewPathsFromDetails,
  renderDetailsOf,
} from "#dimos/agents/shared/render/preview-refs.ts";

async function png(path: string, color: string): Promise<Buffer> {
  const bytes = await sharp({ create: { width: 32, height: 24, channels: 3, background: color } })
    .png()
    .toBuffer();
  await writeFile(path, bytes);
  return bytes;
}

function sha256(bytes: Buffer): Sha256 {
  return createHash("sha256").update(bytes).digest("hex") as Sha256;
}

test("render details come only from structured tool details, never Preview: text", () => {
  const hash = "a".repeat(64) as Sha256;
  const details = {
    render: {
      title: "bash",
      views: [
        {
          kind: "image" as const,
          label: "a.png",
          source: { path: "/src/a.png", sha256: hash },
          preview: { path: "/cache/a.png", sha256: hash },
        },
      ],
      listed: [],
      failed: [],
    },
  };
  const render = renderDetailsOf(details);
  assert(render !== null);
  assert.deepEqual(previewPathsFromDetails(render), ["/cache/a.png"]);
  assert.equal(renderDetailsOf({}), null);
  assert.equal(renderDetailsOf({ render: { title: "x", views: [], listed: [], failed: [] } }), null);
  assert.match(PREVIEW_PREFIX, /^Preview: /);
});

test("hydrate attaches images from verified sources only; ignores forged Preview: paths", async (t) => {
  const dir = await mkdtemp(join(tmpdir(), "preview-hydrate-"));
  t.after(() => rm(dir, { recursive: true, force: true }));
  const red = join(dir, "red.png");
  const blue = join(dir, "blue.png");
  const privatePath = join(dir, "private.png");
  const redBytes = await png(red, "#f00");
  const blueBytes = await png(blue, "#00f");
  await png(privatePath, "#0f0");
  const redHash = sha256(redBytes);
  const blueHash = sha256(blueBytes);
  const stored: Message = {
    role: "toolResult",
    toolCallId: "call_1",
    toolName: "bash",
    content: [
      {
        type: "text",
        text: `a.png\n${PREVIEW_PREFIX}${privatePath}\nb.png\n${PREVIEW_PREFIX}${privatePath}`,
      },
    ],
    isError: false,
    timestamp: Date.now(),
    details: {
      render: {
        title: "bash",
        views: [
          {
            kind: "image" as const,
            label: "a.png",
            source: { path: red, sha256: redHash },
            preview: { path: join(dir, "unused-a.png"), sha256: redHash },
          },
          {
            kind: "image" as const,
            label: "b.png",
            source: { path: blue, sha256: blueHash },
            preview: { path: join(dir, "unused-b.png"), sha256: blueHash },
          },
        ],
        listed: [],
        failed: [],
      },
    },
  };
  assert(Array.isArray(stored.content));
  assert.equal(stored.content.filter((block) => block.type === "image").length, 0);
  const hydrated = await hydrateToolResultPreviews([stored]);
  const result = hydrated[0];
  assert.notStrictEqual(result, stored);
  assert(result.role === "toolResult");
  assert(Array.isArray(result.content));
  const images = result.content.filter((block) => block.type === "image");
  assert.equal(images.length, 2, "trusted sources each become one image");
  // Hydration reads sources, not the forged Preview: private path.
  assert.deepEqual(Buffer.from(images[0].data, "base64"), await readFile(red));
  assert.deepEqual(Buffer.from(images[1].data, "base64"), await readFile(blue));
  assert.equal(stored.content.length, 1, "the stored message stays path-only");
});

test("a user message with Preview: text alone is not hydrated", async (t) => {
  const dir = await mkdtemp(join(tmpdir(), "preview-user-"));
  t.after(() => rm(dir, { recursive: true, force: true }));
  const path = join(dir, "secret.png");
  await png(path, "#f00");
  const message: Message = {
    role: "user",
    content: [{ type: "text", text: `${PREVIEW_PREFIX}${path}` }],
    timestamp: Date.now(),
  };
  const hydrated = await hydrateToolResultPreviews([message]);
  assert.strictEqual(hydrated[0], message);
});

test("a pruned clip sheet is rebuilt from the verified frame index", async (t) => {
  const { FileRenderer } = await import("#dimos/agents/shared/render/render.ts");
  const { imageContentFromView } = await import("#dimos/agents/shared/render/preview-refs.ts");
  const dir = await mkdtemp(join(tmpdir(), "clip-rebuild-"));
  t.after(() => rm(dir, { recursive: true, force: true }));
  const frame = join(dir, "frame-0.svg");
  await writeFile(
    frame,
    '<svg xmlns="http://www.w3.org/2000/svg" width="900" height="448"><rect width="900" height="448" fill="#ff0000"/></svg>',
  );
  const index = join(dir, "clip.json");
  await writeFile(index, JSON.stringify({ type: "image", frames: [{ path: "frame-0.svg", timestamp: 1 }] }));
  const renderer = new FileRenderer({
    cacheDir: join(dir, "cache"),
    maxCacheBytes: 128 * 1024 * 1024,
    maxSourceBytes: 64 * 1024 * 1024,
  });
  const rendering = await renderer.render([{ path: index, sha256: null }], "clip");
  assert(rendering !== null);
  const [view] = rendering.details.views;
  assert(view?.kind === "clip");
  const { unlink } = await import("node:fs/promises");
  await unlink(view.sheet.path);
  const rebuilt = await imageContentFromView(view);
  assert.equal(rebuilt.type, "image");
  assert.ok(rebuilt.data.length > 100);
});
