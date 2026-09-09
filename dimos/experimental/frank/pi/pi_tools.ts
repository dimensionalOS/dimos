// Pi extension: FRANK's bundled checks as tools that return text and the picture together.
//
// `check_leg(x, y)` runs leg.py once (map path check, eight rays, frontier edges, corridor photo,
// map crop) and hands the model the text and both pictures in one tool result, so a move decision
// is one round trip instead of four shell calls and two Reads. Without x, y it is "what is around me".
//
// Loaded by loop.py with `pi --extension dimos/experimental/frank/pi/pi_tools.ts`.

import type { ExtensionAPI } from "@earendil-works/pi-coding-agent";
import { Type } from "@sinclair/typebox";
import { readFileSync } from "node:fs";
import { dirname, join, resolve } from "node:path";
import { fileURLToPath } from "node:url";

const HERE = dirname(fileURLToPath(import.meta.url));
const REPO = resolve(HERE, "..", "..", "..", "..");
const LEG = join(HERE, "..", "tools", "leg.py");
const RECALL = join(HERE, "..", "tools", "recall.py");

async function picture(pi: ExtensionAPI, script: string, args: string[], signal: AbortSignal, timeout: number) {
  const r = await pi.exec("uv", ["run", "python", script, "--json", ...args], {
    signal,
    timeout,
    cwd: REPO,
    env: { ...process.env, DIMOS_TRANSPORT: "lcm" },
  });
  if (r.code !== 0) {
    const why = (r.stderr || r.stdout || "").trim().split("\n").slice(-3).join("\n");
    return { content: [{ type: "text" as const, text: `failed: ${why}` }], details: {}, isError: true };
  }
  const last = r.stdout.trim().split("\n").pop() ?? "{}";
  const out = JSON.parse(last) as { text: string; image: string | null };
  const content: any[] = [{ type: "text", text: out.text }];
  if (out.image) {
    const mime = out.image.endsWith(".png") ? "image/png" : "image/jpeg";
    content.push({ type: "image", data: readFileSync(out.image).toString("base64"), mimeType: mime });
  }
  return { content, details: { image: out.image } };
}

export default function (pi: ExtensionAPI) {
  pi.registerTool({
    name: "check_leg",
    label: "Check leg",
    description:
      "Everything you need before one move, in one call: what the map says about the straight " +
      "line to world point (x, y) (clear, tight, blocked, unseen); the eight rays around you; the " +
      "reachable edges into unseen space; the camera frame with the rays, edges, and the corridor " +
      "to (x, y) drawn on it; and a crop of the map around you with the same leg drawn at your " +
      "width. Decide from the two pictures: the photo shows what the lidar misses, the map shows " +
      "what the camera cannot see. If the leg is more than 60 degrees off your nose it first turns " +
      "you in place to face it, so never turn before calling it. Call it without x and y to just " +
      "look around (that never moves you). Stale the moment you move.",
    parameters: Type.Object({
      x: Type.Optional(Type.Number({ description: "world x of the leg's end, metres" })),
      y: Type.Optional(Type.Number({ description: "world y of the leg's end, metres" })),
    }),
    async execute(_toolCallId, params, signal) {
      const args = ["run", "python", LEG, "check"];
      const x = params.x === undefined ? NaN : Number(params.x); // the model sometimes sends "1.0"
      const y = params.y === undefined ? NaN : Number(params.y);
      if (Number.isFinite(x) && Number.isFinite(y)) args.push(String(x), String(y));
      args.push("--json");
      const r = await pi.exec("uv", args, {
        signal,
        timeout: 120000,
        cwd: REPO,
        env: { ...process.env, DIMOS_TRANSPORT: "lcm" },
      });
      if (r.code !== 0) {
        const why = (r.stderr || r.stdout || "").trim().split("\n").slice(-3).join("\n");
        return { content: [{ type: "text", text: `check failed: ${why}` }], details: {}, isError: true };
      }
      const last = r.stdout.trim().split("\n").pop() ?? "{}";
      const out = JSON.parse(last) as { text: string; image: string; map: string };
      const photo = readFileSync(out.image).toString("base64");
      const map = readFileSync(out.map).toString("base64");
      return {
        content: [
          { type: "text", text: out.text },
          { type: "image", data: photo, mimeType: "image/jpeg" },
          { type: "image", data: map, mimeType: "image/png" },
        ],
        details: { image: out.image, map: out.map },
      };
    },
  });

  pi.registerTool({
    name: "recall",
    label: "Recall",
    description:
      "Your memory: the run's recording of camera, odometry, and lidar, queried. One of: " +
      "seconds=N shows the last N seconds as one picture, a frame for each heading and place you " +
      "were at (call it after turning around instead of turning again); x, y (and radius) shows " +
      "what you saw when you were at that spot; when='HH:MM:SS' shows the frame at that time; " +
      "trail=true draws where you have walked on the map; find='a staircase' searches the last " +
      "minutes of frames by description (slow, about 20 s; look at the tiles before believing it). " +
      "Captions give time, world pose, and the compass direction faced. Never moves you.",
    parameters: Type.Object({
      seconds: Type.Optional(Type.Number({ description: "look back this many seconds" })),
      x: Type.Optional(Type.Number({ description: "world x of a place you were at" })),
      y: Type.Optional(Type.Number({ description: "world y of that place" })),
      radius: Type.Optional(Type.Number({ description: "metres around x, y; default 2" })),
      when: Type.Optional(Type.String({ description: "HH:MM:SS today" })),
      trail: Type.Optional(Type.Boolean({ description: "draw the path walked" })),
      find: Type.Optional(Type.String({ description: "what to look for in past frames" })),
      minutes: Type.Optional(Type.Number({ description: "how far back near, trail, and find look; default 30, 10, 10" })),
    }),
    async execute(_toolCallId, params, signal) {
      const p = params as Record<string, unknown>;
      const num = (v: unknown) => (v === undefined ? NaN : Number(v));
      const args: string[] = [];
      if (Number.isFinite(num(p.seconds))) args.push("recent", "--seconds", String(num(p.seconds)));
      else if (Number.isFinite(num(p.x)) && Number.isFinite(num(p.y))) {
        args.push("near", String(num(p.x)), String(num(p.y)));
        if (Number.isFinite(num(p.radius))) args.push("--radius", String(num(p.radius)));
        if (Number.isFinite(num(p.minutes))) args.push("--minutes", String(num(p.minutes)));
      } else if (typeof p.when === "string" && p.when) args.push("at", p.when);
      else if (p.trail) {
        args.push("trail");
        if (Number.isFinite(num(p.minutes))) args.push("--minutes", String(num(p.minutes)));
      } else if (typeof p.find === "string" && p.find) {
        args.push("find", p.find);
        if (Number.isFinite(num(p.minutes))) args.push("--minutes", String(num(p.minutes)));
      } else args.push("streams");
      return picture(pi, RECALL, args, signal, 180000);
    },
  });
}
