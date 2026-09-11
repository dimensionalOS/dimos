import * as THREE from "three";
import type { VisualGeom } from "./worldModel.ts";

const ACCENTS = new Set([
  "face_part",
  "upper_leg_rigidity_plate",
  "ankle_left",
  "ankle_right",
]);
const SHELLS = new Set([
  "left_shell",
  "right_shell",
  "top_head_shell",
  "bottom_head_shell",
  "jaw",
  "jaw_soft",
  "soft_mouth_top",
  "leg",
  "upper_leg_left",
  "upper_leg_right",
  "foot_left",
  "foot_right",
  "sole_left",
  "sole_right",
  "trunk_base",
  "np_f970",
  "power_support",
  "hip_l",
  "yaw2roll",
  "yaw_roll_motion",
  "neck",
  "neck_pitch",
]);

/** Cosmetic browser palette, shared by the lobby, world and duck POV. */
export function duckColor(geom: VisualGeom, accent: string): THREE.Color {
  const part = (geom.part ?? "").replace(/^duck\d+_/, "").replace(
    /\.stl$/i,
    "",
  );
  if (ACCENTS.has(part)) return new THREE.Color(accent);
  if (part.includes("bearing")) return new THREE.Color("#aebbc5");
  if (SHELLS.has(part) || Math.max(...geom.rgba.slice(0, 3)) > .5) {
    return new THREE.Color("#f1f3f0");
  }
  return new THREE.Color().setRGB(
    geom.rgba[0],
    geom.rgba[1],
    geom.rgba[2],
    THREE.SRGBColorSpace,
  );
}
