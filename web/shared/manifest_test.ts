import { assertEquals, assertThrows } from "@std/assert";
import { ManifestError, parseManifest } from "./manifest.ts";
import manifestFixture from "./fixtures/manifests.json" with { type: "json" };

// Each vector pins either normalization ({name, data, manifest}) or the
// rejection code ({name, data, error}); test_manifest.py runs the same set.
type Vector = { name: string; data: unknown; manifest?: unknown; error?: string };
const vectors = manifestFixture.vectors as Vector[];

Deno.test("valid manifests normalize per the golden vectors", () => {
  const valid = vectors.filter((v) => v.manifest !== undefined);
  assertEquals(valid.length > 0, true);
  for (const v of valid) {
    assertEquals(parseManifest(v.data), v.manifest, v.name);
  }
});

Deno.test("invalid manifests are rejected with the pinned code", () => {
  const invalid = vectors.filter((v) => v.error !== undefined);
  assertEquals(invalid.length > 0, true);
  for (const v of invalid) {
    const err = assertThrows(() => parseManifest(v.data), ManifestError, undefined, v.name);
    assertEquals(err.code, v.error, v.name);
  }
});
