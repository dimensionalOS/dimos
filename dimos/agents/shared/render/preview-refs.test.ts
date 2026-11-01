import assert from "node:assert/strict";
import { mkdtemp, rm, writeFile } from "node:fs/promises";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { test } from "node:test";
import sharp from "sharp";
import type { Message } from "@earendil-works/pi-ai";
import type { Sha256 } from "#dimos/agents/shared/protocol/media.ts";
import {
  PREVIEW_PREFIX,
  hydrateToolResultPreviews,
  previewPathsFromText,
  previewPathsFromToolResult,
} from "#dimos/agents/shared/render/preview-refs.ts";

async function png(path: string, color: string): Promise<void> {
  await writeFile(
    path,
    await sharp({ create: { width: 32, height: 24, channels: 3, background: color } })
      .png()
      .toBuffer(),
  );
}

test("preview paths prefer details.render.views, then Preview: text lines", () => {
  const hash = "a".repeat(64) as Sha256;
  assert.deepEqual(
    previewPathsFromToolResult(
      {
        render: {
          title: "bash",
          views: [
            {
              kind: "image",
              label: "a.png",
              source: { path: "/src/a.png", sha256: hash },
              preview: { path: "/cache/a.png", sha256: hash },
            },
          ],
          listed: [],
          failed: [],
        },
      },
      `${PREVIEW_PREFIX}/ignored.png`,
    ),
    ["/cache/a.png"],
  );
  assert.deepEqual(previewPathsFromText(`${PREVIEW_PREFIX}/one.png\nother\n${PREVIEW_PREFIX}/two.png`), [
    "/one.png",
    "/two.png",
  ]);
});

test("hydrate attaches distinct previews to path-only tool results; leaves stored content without images", async (t) => {
  const dir = await mkdtemp(join(tmpdir(), "preview-hydrate-"));
  t.after(() => rm(dir, { recursive: true, force: true }));
  const red = join(dir, "red.png");
  const blue = join(dir, "blue.png");
  await png(red, "#f00");
  await png(blue, "#00f");
  const hash = "b".repeat(64) as Sha256;
  const stored: Message = {
    role: "toolResult",
    toolCallId: "call_1",
    toolName: "bash",
    content: [{ type: "text", text: `a.png · ${red}\n${PREVIEW_PREFIX}${red}\nb.png · ${blue}\n${PREVIEW_PREFIX}${blue}` }],
    isError: false,
    timestamp: Date.now(),
    details: {
      render: {
        title: "bash",
        views: [
          {
            kind: "image" as const,
            label: "a.png",
            source: { path: red, sha256: hash },
            preview: { path: red, sha256: hash },
          },
          {
            kind: "image" as const,
            label: "b.png",
            source: { path: blue, sha256: hash },
            preview: { path: blue, sha256: hash },
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
  assert.equal(images.length, 2, "distinct preview paths each become one image");
  assert.equal(stored.content.length, 1, "the stored message stays path-only");
});
