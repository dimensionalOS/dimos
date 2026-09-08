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

Deno.test("slot-kind rules are pinned by valid and invalid golden vectors", () => {
  // Guards against a slot kind silently losing its fixture coverage (the
  // Python mirror is pinned by the same vectors): each needs at least one
  // accepted manifest and one rejected with its own invalid_<kind>_panel code.
  for (const kind of ["chat", "navmap", "control"]) {
    const ofKind = vectors.filter((v) => v.name.startsWith(`${kind}_panel_`));
    assertEquals(ofKind.some((v) => v.manifest !== undefined), true, `${kind}: valid`);
    assertEquals(ofKind.some((v) => v.error === `invalid_${kind}_panel`), true, `${kind}: invalid`);
  }
  // The whole microduck cockpit layout parses.
  assertEquals(vectors.find((v) => v.name === "cockpit_microduck")?.manifest !== undefined, true);
});

Deno.test("maxHz beyond float64 is rejected", () => {
  // Not a golden vector: gen.ts cannot emit a number beyond float64
  // (JSON.stringify(Infinity) is null). JSON.parse turns such a literal into
  // Infinity; test_manifest.py pins the Python half (an exact huge int) to
  // the same code.
  const data = {
    version: 1,
    channels: [
      { ch: "odom", encoding: "pose.json.v1", delivery: "reliable", maxHz: Infinity },
    ],
  };
  const err = assertThrows(() => parseManifest(data), ManifestError);
  assertEquals(err.code, "invalid_max_hz");
});

Deno.test("layout share beyond float64 is rejected", () => {
  // Same non-golden reasoning as above, for the layout walker's share bound.
  const data = {
    version: 1,
    channels: [{ ch: "odom", encoding: "pose.json.v1", delivery: "reliable", maxHz: 20.5 }],
    panels: [{ id: "pose", kind: "readout", channels: ["odom"] }],
    layout: { row: ["pose"], shares: [Infinity] },
  };
  const err = assertThrows(() => parseManifest(data), ManifestError);
  assertEquals(err.code, "invalid_layout");
});
