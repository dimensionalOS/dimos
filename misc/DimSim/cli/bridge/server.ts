#!/usr/bin/env -S deno run --allow-net --allow-read --unstable-net

/**
 * DimSim Bridge Server
 *
 * - One control WebSocket plus multiple sensor WebSockets.
 *   Separate TCP streams so large sensor data never blocks real-time odom
 *   or other sensor streams.
 * - LCM multicast relay (WS ↔ LCM)
 * - Per-channel isolation for multi-page parallel evals
 * - Static file server for the pre-built DimSim frontend (dist/)
 * - Uses vendored LCM transport with joinMulticastV4 fix
 */

import { LCM } from "../vendor/lcm/lcm.ts";
import { decodePacket } from "../vendor/lcm/transport.ts";
import { MAGIC_SHORT, SHORT_HEADER_SIZE } from "../vendor/lcm/types.ts";
import { serveDir } from "@std/http/file-server";
import { ServerLidar } from "./lidar.ts";
import { ServerPhysics } from "./physics.ts";
import { geometry_msgs } from "@dimos/msgs";

// Magic prefix for Rapier world snapshot (ASCII "DSSN")
const SNAPSHOT_MAGIC = 0x4453534E;
const DEFAULT_LCM_PORT = 7667;
const DEFAULT_LCM_HOST = "239.255.76.67";

const SCENE_MIME: Record<string, string> = {
  js: "application/javascript; charset=utf-8",
  mjs: "application/javascript; charset=utf-8",
  json: "application/json; charset=utf-8",
  glb: "model/gltf-binary",
  gltf: "model/gltf+json",
  bin: "application/octet-stream",
  png: "image/png",
  jpg: "image/jpeg",
  jpeg: "image/jpeg",
  webp: "image/webp",
  avif: "image/avif",
  ktx2: "image/ktx2",
  hdr: "image/vnd.radiance",
  exr: "image/x-exr",
};

export interface BridgeServerOptions {
  port: number;
  distDir: string;
  scene?: string;
  evalOnly?: boolean;
  headless?: boolean;
  channels?: string[];
  lcmBasePort?: number;
  sensorRates?: Record<string, number>;
  sensorEnable?: Record<string, boolean>;
  cameraFov?: number;
}

/** Per-channel state: each channel gets its own LCM, physics, lidar, and WS client sets. */
interface ChannelState {
  name: string;
  controlClients: Set<WebSocket>;
  activeControlClient: WebSocket | null;
  sensorClients: Set<WebSocket>;
  lcm: LCM | null;
  sentSeqs: Set<number>;
  serverLidar: ServerLidar | null;
  serverPhysics: ServerPhysics | null;
  embodiment: Record<string, any> | null;
}

export async function startBridgeServer(options: BridgeServerOptions) {
  const {
    port, distDir, scene,
    evalOnly = false, headless = false,
    channels, lcmBasePort = DEFAULT_LCM_PORT,
    sensorRates, sensorEnable, cameraFov,
  } = options;

  // Scene the engine boots into.  Injected as window.__dimosScene below so
  // engine.js dynamically imports /scenes/<name>/index.js. Sanitize to a
  // restricted charset so the value can't escape the <script> string literal
  // it ends up interpolated into.
  const rawScene = scene || "empty";
  if (!/^[a-zA-Z0-9_-]+$/.test(rawScene)) {
    throw new Error(`invalid --scene "${rawScene}" — must match [a-zA-Z0-9_-]+`);
  }
  const activeSceneName: string = rawScene;

  // Event-loop lag probe — fire setTimeout(0) every 50ms; difference between
  // expected and actual fire time is contention-induced lag. If physics misses
  // ticks, this will show why.
  if (Deno.env.get("DIMSIM_PROFILE_PHYSICS") === "1") {
    let lagSum = 0, lagMax = 0, lagCount = 0;
    const tick = () => {
      const expected = performance.now() + 50;
      setTimeout(() => {
        const lag = performance.now() - expected;
        lagSum += Math.max(lag, 0);
        if (lag > lagMax) lagMax = lag;
        lagCount++;
        if (lagCount >= 20) { // ~1s
          console.log(`[loop-prof] lag avg=${(lagSum / lagCount).toFixed(1)}ms max=${lagMax.toFixed(1)}ms (over ${lagCount} probes)`);
          lagSum = 0; lagMax = 0; lagCount = 0;
        }
        tick();
      }, 50);
    };
    tick();
  }

  // ── Per-handler profiling --------------------------------------------------
  // Records sum/max/count for two hot callbacks so we can see whether they
  // dominate the bridge thread alongside lidar.
  const PROFILE = Deno.env.get("DIMSIM_PROFILE_PHYSICS") === "1";
  const profPose = { n: 0, sum: 0, max: 0 };
  const profRelay = { n: 0, sum: 0, max: 0, bytes: 0 };
  if (PROFILE) {
    setInterval(() => {
      if (profPose.n > 0) {
        console.log(
          `[pose-cb-prof] n=${profPose.n} avg=${(profPose.sum / profPose.n).toFixed(2)}ms ` +
          `max=${profPose.max.toFixed(2)}ms total=${profPose.sum.toFixed(0)}ms/sec`,
        );
        profPose.n = 0; profPose.sum = 0; profPose.max = 0;
      }
      if (profRelay.n > 0) {
        console.log(
          `[relay-prof] n=${profRelay.n} avg=${(profRelay.sum / profRelay.n).toFixed(2)}ms ` +
          `max=${profRelay.max.toFixed(2)}ms total=${profRelay.sum.toFixed(0)}ms/sec ` +
          `bytes=${profRelay.bytes}`,
        );
        profRelay.n = 0; profRelay.sum = 0; profRelay.max = 0; profRelay.bytes = 0;
      }
    }, 1000);
  }

  // Build channel list: if channels provided, use them; otherwise single default
  const channelNames = channels && channels.length > 0
    ? channels
    : [""];  // empty string = default (backward compat, no channel routing)

  const channelMap = new Map<string, ChannelState>();

  for (let i = 0; i < channelNames.length; i++) {
    const name = channelNames[i];
    const lcmPort = lcmBasePort + i;
    const lcmUrl = `udpm://${DEFAULT_LCM_HOST}:${lcmPort}?ttl=0`;
    const state: ChannelState = {
      name,
      controlClients: new Set(),
      activeControlClient: null,
      sensorClients: new Set(),
      lcm: null,
      sentSeqs: new Set(),
      serverLidar: null,
      serverPhysics: null,
      embodiment: null,
    };

    if (!evalOnly) {
      state.lcm = new LCM(lcmUrl);
      await state.lcm.start();
      console.log(`[bridge] channel "${name || "default"}" LCM on ${lcmUrl}`);

      // LCM → WS: forward external packets to this channel's active control client
      state.lcm.subscribePacket((packet: Uint8Array) => {
        if (packet.length < 8) return;
        const view = new DataView(packet.buffer, packet.byteOffset, packet.byteLength);
        const magic = view.getUint32(0, false);
        if (magic !== MAGIC_SHORT) return;

        const seq = view.getUint32(4, false);
        if (state.sentSeqs.has(seq)) {
          state.sentSeqs.delete(seq);
          return;
        }
        if (state.sentSeqs.size > 1000) state.sentSeqs.clear();

        const copy = packet.slice();
        const client = state.activeControlClient;
        if (client && client.readyState === WebSocket.OPEN) client.send(copy);
      });
    }

    channelMap.set(name, state);
  }

  /** Resolve channel from WS query param. Falls back to default ("") if not found. */
  function resolveChannel(channelParam: string | null): ChannelState {
    if (channelParam && channelMap.has(channelParam)) {
      return channelMap.get(channelParam)!;
    }
    // If no channel param or unknown, use default (first channel)
    return channelMap.values().next().value!;
  }

  // -- Server-side init from Rapier snapshot ----------------------------------
  async function initServerSystems(
    chState: ChannelState,
    snapshot: Uint8Array,
    spawnPos?: { x: number; y: number; z: number },
  ): Promise<void> {
    if (chState.serverLidar) { chState.serverLidar.stop(); chState.serverLidar = null; }
    if (chState.serverPhysics) { chState.serverPhysics.stop(); chState.serverPhysics = null; }
    if (!chState.lcm) return;

    try {
      const RAPIER = await import("@dimforge/rapier3d-compat");
      await RAPIER.init();
      const world = RAPIER.World.restoreSnapshot(snapshot);
      if (!world) { console.error(`[bridge:${chState.name || "default"}] failed to restore Rapier snapshot`); return; }

      const bodiesToRemove: any[] = [];
      world.bodies.forEach((body: any) => {
        if (!body.isFixed()) bodiesToRemove.push(body.handle);
      });
      for (const handle of bodiesToRemove) {
        world.removeRigidBody(world.getRigidBody(handle));
      }
      // Single canonical "physics live" marker — test fixtures grep for this.
      console.log(`[bridge:${chState.name || "default"}] ready`);

      chState.serverPhysics = new ServerPhysics(chState.lcm, world, RAPIER, chState.sentSeqs, chState.embodiment ?? undefined);
      if (spawnPos) {
        chState.serverPhysics.setPosition(spawnPos.x, spawnPos.y, spawnPos.z);
      }

      chState.serverLidar = new ServerLidar(chState.lcm, world, RAPIER, chState.sentSeqs, chState.embodiment ?? undefined);
      chState.serverLidar.setExcludeBody(chState.serverPhysics.getBody());

      chState.serverPhysics.setOnPoseUpdate((x, y, z, yaw) => {
        const t0 = PROFILE ? performance.now() : 0;
        const qw = Math.cos(yaw / 2);
        const qy = Math.sin(yaw / 2);
        chState.serverLidar!.updatePose(x, y, z, 0, qy, 0, qw);

        const msg = JSON.stringify({ type: "pose", x, y, z, yaw });
        const client = chState.activeControlClient;
        if (client && client.readyState === WebSocket.OPEN) {
          try { client.send(msg); } catch { /* ignore */ }
        }
        if (PROFILE) {
          const dt = performance.now() - t0;
          profPose.n++;
          profPose.sum += dt;
          if (dt > profPose.max) profPose.max = dt;
        }
      });

      chState.serverPhysics.start();
      chState.serverLidar.start();
    } catch (e) {
      console.error(`[bridge:${chState.name || "default"}] server systems init error:`, e);
    }
  }

  // ── HTTP + WebSocket server ─────────────────────────────────────────────
  Deno.serve({ port }, async (req: Request) => {
    const url = new URL(req.url);

    if (req.headers.get("upgrade") === "websocket") {
      const { socket, response } = Deno.upgradeWebSocket(req);
      socket.binaryType = "arraybuffer";
      const ch = url.searchParams.get("ch") || "control";
      const channelParam = url.searchParams.get("channel");
      const chState = resolveChannel(channelParam);
      const isSensor = ch !== "control";
      const logPrefix = `[bridge:${chState.name || "default"}]`;

      if (isSensor) {
        // ── SENSOR WebSocket ──────────────────────────────────────────
        socket.onopen = () => { chState.sensorClients.add(socket); };
        socket.onclose = () => { chState.sensorClients.delete(socket); };
        socket.onerror = () => chState.sensorClients.delete(socket);

        // Chunked snapshot reassembly state (DSC1 protocol).
        // Browser ships the Rapier snapshot in many small frames so a
        // CPU-starved main thread can drain the WebSocket pump.
        let chunkedSnapshot: {
          total: number;
          spawn: { x: number; y: number; z: number };
          received: number;
          parts: Uint8Array[];
        } | null = null;

        let _sensorLogN = 0;
        socket.onmessage = (event: MessageEvent) => {
          if (!(event.data instanceof ArrayBuffer) || !chState.lcm) return;
          const packet = new Uint8Array(event.data);

          // While reassembling a chunked snapshot, treat every binary frame
          // on this socket as the next chunk in order.
          if (chunkedSnapshot) {
            chunkedSnapshot.parts.push(packet);
            chunkedSnapshot.received += packet.byteLength;
            if (chunkedSnapshot.received >= chunkedSnapshot.total) {
              const combined = new Uint8Array(chunkedSnapshot.received);
              let off = 0;
              for (const p of chunkedSnapshot.parts) { combined.set(p, off); off += p.byteLength; }
              const snapshot = combined.subarray(0, chunkedSnapshot.total);
              const spawn = chunkedSnapshot.spawn;
              chunkedSnapshot = null;
              initServerSystems(chState, snapshot, spawn);
            }
            return;
          }

          // Check for Rapier snapshot
          if (packet.length > 4) {
            const dv = new DataView(packet.buffer, packet.byteOffset);
            const magic = dv.getUint32(0, false);

            if (magic === 0x44534331) { // "DSC1" — chunked prelude
              const total = dv.getUint32(4, true);
              const sx = dv.getFloat32(8, true);
              const sy = dv.getFloat32(12, true);
              const sz = dv.getFloat32(16, true);
              chunkedSnapshot = {
                total,
                spawn: { x: sx, y: sy, z: sz },
                received: 0,
                parts: [],
              };
              return;
            }

            if (magic === 0x44535332) { // "DSS2"
              const sx = dv.getFloat32(4, true);
              const sy = dv.getFloat32(8, true);
              const sz = dv.getFloat32(12, true);
              const snapshot = packet.slice(16);
              initServerSystems(chState, snapshot, { x: sx, y: sy, z: sz });
              return;
            }

            if (magic === SNAPSHOT_MAGIC) { // "DSSN"
              const snapshot = packet.slice(4);
              initServerSystems(chState, snapshot);
              return;
            }
          }

          const t0 = PROFILE ? performance.now() : 0;
          try {
            const decoded = decodePacket(packet);
            if (decoded && decoded.type === "small") {
              _sensorLogN++;
              if (_sensorLogN === 1 || _sensorLogN % 1000 === 0) {
                const chName = decoded.channel.split("#")[0].replace("/", "");
                // quiet — sensor relay logging removed
              }
              chState.sentSeqs.add(chState.lcm.getNextSeq());
              chState.lcm.publishRaw(decoded.channel, decoded.data).catch(() => {});
            }
          } catch { /* ignore */ }
          if (PROFILE) {
            const dt = performance.now() - t0;
            profRelay.n++;
            profRelay.sum += dt;
            profRelay.bytes += packet.byteLength;
            if (dt > profRelay.max) profRelay.max = dt;
          }
        };
      } else {
        // ── CONTROL WebSocket ─────────────────────────────────────────
        socket.onopen = () => {
          if (!chState.activeControlClient || chState.activeControlClient.readyState !== WebSocket.OPEN) {
            chState.activeControlClient = socket;
          }
          chState.controlClients.add(socket);
          // quiet
        };
        socket.onerror = () => chState.controlClients.delete(socket);

        let _odomLogN = 0;

        socket.onclose = () => {
          chState.controlClients.delete(socket);
          if (chState.activeControlClient === socket) chState.activeControlClient = null;
          // quiet
        };

        socket.onmessage = (event: MessageEvent) => {
          // Text messages: handle special types, relay the rest
          if (typeof event.data === "string") {
            try {
              const msg = JSON.parse(event.data);

              // -- Embodiment config: store & reconfigure running systems --
              if (msg.type === "embodimentConfig") {
                chState.embodiment = msg.config ?? msg;
                console.log(`${logPrefix} embodiment config stored:`, JSON.stringify(chState.embodiment));
                if (chState.serverPhysics) chState.serverPhysics.reconfigure(chState.embodiment as any);
                if (chState.serverLidar) chState.serverLidar.reconfigure(chState.embodiment as any);
                // fall through to relay to browser
              }

              // -- Teleport: reposition physics agent, don't relay --
              if (msg.type === "teleport") {
                if (chState.serverPhysics && msg.x != null && msg.y != null && msg.z != null) {
                  chState.serverPhysics.setPosition(msg.x, msg.y, msg.z);
                  console.log(`${logPrefix} teleport to (${msg.x},${msg.y},${msg.z})`);
                }
                return; // don't relay teleport commands
              }

              // -- Physics collider add/remove: also apply to ServerPhysics
              // world so live-authored colliders (floor, walls, dynamic balls)
              // become real obstacles for the server-side agent. Without this,
              // the dog falls through anything added after the boot snapshot.
              if (msg.type === "physicsColliderAdd" && msg.uuid && msg.desc) {
                chState.serverPhysics?.addCollider(msg.uuid, msg.desc);
              }
              if (msg.type === "physicsColliderRemove" && msg.uuid) {
                chState.serverPhysics?.removeCollider(msg.uuid);
              }
            } catch { /* not JSON, relay as-is */ }

            for (const client of chState.controlClients) {
              if (client !== socket && client.readyState === WebSocket.OPEN) {
                try { client.send(event.data); } catch { /* ignore */ }
              }
            }
            return;
          }
          if (!(event.data instanceof ArrayBuffer) || !chState.lcm) return;
          if (chState.activeControlClient !== socket) return;
          const packet = new Uint8Array(event.data);
          try {
            const decoded = decodePacket(packet);
            if (decoded && decoded.type === "small") {
              _odomLogN++;

              if (chState.serverPhysics && decoded.channel === "/odom#geometry_msgs.PoseStamped") {
                return;
              }

              chState.sentSeqs.add(chState.lcm.getNextSeq());
              chState.lcm.publishRaw(decoded.channel, decoded.data).catch(() => {});
            }
          } catch { /* ignore */ }
        };
      }

      return response;
    }


    if (url.pathname === "/" || url.pathname === "/index.html") {
      try {
        let html = await Deno.readTextFile(`${distDir}/index.html`);
        const ratesJs = sensorRates ? `window.__dimosSensorRates=${JSON.stringify(sensorRates)};` : "";
        const enableJs = sensorEnable ? `window.__dimosSensorEnable=${JSON.stringify(sensorEnable)};` : "";
        const fovJs = cameraFov ? `window.__dimosCameraFov=${cameraFov};` : "";
        const inject = `<script>window.__dimosMode=true;window.__dimosScene="${activeSceneName}";${headless ? "window.__dimosHeadless=true;" : ""}${ratesJs}${enableJs}${fovJs}</script>`;
        html = html.replace("</head>", `${inject}\n</head>`);
        return new Response(html, { headers: { "content-type": "text/html; charset=utf-8" } });
      } catch {
        return new Response("index.html not found", { status: 404 });
      }
    }

    // JS scene project folders. Resolution order:
    //   1. DIMSIM_SCENES_DIR env (dimos points this at user-authored scenes)
    //   2. dist/scenes/         (shipped built-ins, when running from a built binary)
    //   3. ../scenes/           (dev built-ins, when running directly from source)
    if (url.pathname.startsWith("/scenes/")) {
      const rel = url.pathname.slice("/scenes/".length);
      const candidates = [
        Deno.env.get("DIMSIM_SCENES_DIR"),
        `${distDir}/scenes`,
        `${distDir}/../scenes`,
      ].filter((d): d is string => !!d && d.length > 0);
      for (const dir of candidates) {
        try {
          const filePath = `${dir}/${rel}`;
          const data = await Deno.readFile(filePath);
          const ext = rel.split(".").pop()?.toLowerCase() ?? "";
          const contentType = SCENE_MIME[ext] ?? "application/octet-stream";
          return new Response(data, { headers: { "content-type": contentType } });
        } catch { /* try next */ }
      }
      return new Response(`Scene file not found: ${rel}`, { status: 404 });
    }

    return serveDir(req, { fsRoot: distDir, quiet: true });
  });

  const channelInfo = channelNames.length > 1
    ? ` (${channelNames.length} channels: ${channelNames.join(", ")})`
    : "";
  console.log(`[bridge] :${port}${evalOnly ? " (eval-only)" : " (LCM bridge)"}${channelInfo}`);

  // Run all LCM instances
  const lcmInstances = [...channelMap.values()].map(s => s.lcm).filter(Boolean) as LCM[];
  if (lcmInstances.length > 0) {
    await Promise.all(lcmInstances.map(l => l.run()));
  } else {
    await new Promise(() => {});
  }
}

if (import.meta.main) {
  const distDir = new URL("../../dist", import.meta.url).pathname;
  const scene = Deno.args.find((_a: string, i: number, arr: string[]) => arr[i - 1] === "--scene") || "apartment";
  const port = parseInt(Deno.args.find((_a: string, i: number, arr: string[]) => arr[i - 1] === "--port") || "8090");
  await startBridgeServer({ port, distDir, scene });
}
