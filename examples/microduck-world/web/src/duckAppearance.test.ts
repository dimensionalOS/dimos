import { describe, expect, it } from "vitest";
import { duckColor } from "./duckAppearance.ts";
import type { VisualGeom } from "./worldModel.ts";
const geom = (part: string): VisualGeom => ({
  id: 1,
  name: "duck part",
  body: 1,
  kind: "mesh",
  mesh: "1",
  part,
  size: [1, 1, 1],
  position: [0, 0, 0],
  quaternion: [1, 0, 0, 0],
  rgba: [.9, .3, .3, 1],
});
describe("cosmetic white and team-detail palette", () => {
  it("keeps both teams' main shells white", () => {
    for (
      const part of [
        "top_head_shell",
        "duck6_bottom_head_shell",
        "left_shell",
        "foot_left",
      ]
    ) {
      for (const team of ["#e34d4e", "#418ee8"]) {
        expect(duckColor(geom(part), team).getHexString()).toBe("f1f3f0");
      }
    }
  });
  it("colors only the accent parts and keeps bearings metallic", () => {
    expect(duckColor(geom("duck4_face_part"), "#418ee8").getHexString()).toBe(
      "418ee8",
    );
    expect(
      duckColor(geom("upper_leg_rigidity_plate"), "#e34d4e").getHexString(),
    ).toBe("e34d4e");
    expect(
      duckColor(geom("seeed_bearing__configuration_default"), "#e34d4e")
        .getHexString(),
    ).toBe("aebbc5");
  });
});
