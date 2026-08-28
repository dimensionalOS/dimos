/**
 * Server-side LiDAR raycasting using Rapier physics world snapshot.
 *
 * Runs 20K Fibonacci-sphere raycasts at 5 Hz on the Deno bridge server,
 * encodes PointCloud2 via @dimos/msgs, and publishes directly to LCM —
 * no WebSocket hop needed.
 */

import {
  sensor_msgs,
  std_msgs,
} from "@dimos/msgs";
import type { LCM } from "../vendor/lcm/lcm.ts";

// -- Lidar constants (must match engine.js) -----------------------------------
const NUM_POINTS = 15000;
const MIN_RANGE = 0.1;
const MAX_RANGE = 4;
const V_MIN_RAD = (-30 * Math.PI) / 180;
const V_MAX_RAD = (15 * Math.PI) / 180;
// Default publish interval (ms) when no rate is passed in (standalone `dimsim
// dev` without --lidar-rate). The dimos connector overrides this via
// --lidar-rate -> ServerLidar(rateMs). Note: ~15k raycasts/scan (~25-30ms on
// M4, ~60-80ms on weaker runners), so at high rates the `publishing` busy-guard
// drops frames and effective rate sits below the target on slow/CPU-render boxes.
const RATE_MS = 200; // 5 Hz default

const CH_LIDAR = "/lidar#sensor_msgs.PointCloud2";

// Agent capsule geometry → lidar mount offset (must match engine.js)
const DEFAULT_HALF_HEIGHT = 0.25;
const DEFAULT_RADIUS = 0.12;
const DEFAULT_LIDAR_MOUNT = 0.35;

/** Subset of EmbodimentConfig relevant to lidar mount offset. */
export interface LidarEmbodimentConfig {
  radius?: number;
  halfHeight?: number;
  lidarMountHeight?: number;
}

// -- _lidarToCamQuat: transforms FLU (x=forward, y=left, z=up) → Three.js camera-local --
// Matches engine.js _lidarToCamQuat derived from rotation matrix:
//   FLU x(forward) → cam -z,  FLU y(left) → cam -x,  FLU z(up) → cam +y
// Quaternion: (0.5, -0.5, -0.5, -0.5)
const L2C_QX = 0.5, L2C_QY = -0.5, L2C_QZ = -0.5, L2C_QW = -0.5;

// -- Pre-compute Fibonacci sphere ray directions (pre-rotated to camera-local) -
// Directions are computed in FLU frame then rotated by _lidarToCamQuat so that
// only the agent's yaw quaternion is needed at scan time (cam-local → world).
const fibDirs = (() => {
  const golden = (1 + Math.sqrt(5)) / 2;
  const zMin = Math.sin(V_MIN_RAD);
  const zMax = Math.sin(V_MAX_RAD);
  const dirs = new Float32Array(NUM_POINTS * 3);
  for (let i = 0; i < NUM_POINTS; i++) {
    const z = zMin + (zMax - zMin) * (i + 0.5) / NUM_POINTS;
    const r = Math.sqrt(1 - z * z);
    const phi = (2 * Math.PI * i) / golden;
    const fx = r * Math.cos(phi); // FLU x
    const fy = r * Math.sin(phi); // FLU y
    const fz = z;                 // FLU z

    // Rotate FLU → camera-local using _lidarToCamQuat
    const tx = 2 * (L2C_QY * fz - L2C_QZ * fy);
    const ty = 2 * (L2C_QZ * fx - L2C_QX * fz);
    const tz = 2 * (L2C_QX * fy - L2C_QY * fx);
    dirs[i * 3 + 0] = fx + L2C_QW * tx + (L2C_QY * tz - L2C_QZ * ty);
    dirs[i * 3 + 1] = fy + L2C_QW * ty + (L2C_QZ * tx - L2C_QX * tz);
    dirs[i * 3 + 2] = fz + L2C_QW * tz + (L2C_QX * ty - L2C_QY * tx);
  }
  return dirs;
})();

// Raycasting is chunked so one scan never blocks the event loop for its full
// duration (~25-80ms depending on hardware): a solid block starves the 50 Hz
// physics stepper and delays cmd_vel/odom handling. Between chunks we yield a
// macrotask so timers can fire. Chunk size targets a few ms of raycasts.
const RAYCAST_CHUNK = 2500;
const _yieldLoop = () => new Promise<void>((resolve) => setTimeout(resolve, 0));

// -- Quaternion rotation helper (q * v) ---------------------------------------
function rotateByQuat(
  vx: number, vy: number, vz: number,
  qx: number, qy: number, qz: number, qw: number,
): [number, number, number] {
  // t = 2 * cross(q.xyz, v)
  const tx = 2 * (qy * vz - qz * vy);
  const ty = 2 * (qz * vx - qx * vz);
  const tz = 2 * (qx * vy - qy * vx);
  // result = v + qw * t + cross(q.xyz, t)
  return [
    vx + qw * tx + (qy * tz - qz * ty),
    vy + qw * ty + (qz * tx - qx * tz),
    vz + qw * tz + (qx * ty - qy * tx),
  ];
}

// -- ServerLidar --------------------------------------------------------------

export class ServerLidar {
  // Resolved once at module load; hot-path callers should not Deno.env.get every scan.
  static readonly PROFILE = Deno.env.get("DIMSIM_PROFILE_PHYSICS") === "1";

  private lcm: LCM;
  private world: any; // RAPIER.World
  private RAPIER: any;
  private sentSeqs: Set<number>; // echo filter shared with bridge server
  private timer: ReturnType<typeof setInterval> | null = null;
  private scanCount = 0;
  private logN = 0;
  private publishing = false; // busy guard — skip scan if previous publish still in flight
  private excludeBody: any = null; // rigid body to exclude from raycasting (agent's own colliders)
  private lidarYOffset: number;

  // Current robot pose (Three.js Y-up world frame)
  private px = 0;
  private py = 0;
  private pz = 0;
  private qx = 0;
  private qy = 0;
  private qz = 0;
  private qw = 1;
  private hasPose = false;

  private ray: any; // Reusable Ray object (avoids 20k allocations per scan)
  private rateMs: number; // publish interval (ms); from --lidar-rate, else RATE_MS default

  constructor(lcm: LCM, rapierWorld: any, RAPIER: any, sentSeqs: Set<number>, embodiment?: LidarEmbodimentConfig, rateMs?: number) {
    this.lcm = lcm;
    this.rateMs = rateMs && rateMs > 0 ? rateMs : RATE_MS;
    this.world = rapierWorld;
    this.RAPIER = RAPIER;
    this.sentSeqs = sentSeqs;
    this.ray = new RAPIER.Ray({ x: 0, y: 0, z: 0 }, { x: 0, y: 0, z: 1 });

    const halfH = embodiment?.halfHeight ?? DEFAULT_HALF_HEIGHT;
    const radius = embodiment?.radius ?? DEFAULT_RADIUS;
    const mount = embodiment?.lidarMountHeight ?? DEFAULT_LIDAR_MOUNT;
    this.lidarYOffset = mount - (halfH + radius);

    // Step once with zero dt to initialize the query pipeline after snapshot restore.
    // queryPipeline.update() crashes on restored worlds (WASM type mismatch),
    // but world.step() internally updates the pipeline correctly.
    this.world.step();
  }

  /** Reconfigure lidar mount offset after embodiment change. */
  reconfigure(embodiment: LidarEmbodimentConfig): void {
    const halfH = embodiment.halfHeight ?? DEFAULT_HALF_HEIGHT;
    const radius = embodiment.radius ?? DEFAULT_RADIUS;
    const mount = embodiment.lidarMountHeight ?? DEFAULT_LIDAR_MOUNT;
    this.lidarYOffset = mount - (halfH + radius);
    console.log(`[lidar] reconfigured: lidarYOffset=${this.lidarYOffset.toFixed(3)}`);
  }

  /** Set rigid body to exclude from raycasting (agent's own capsule). */
  setExcludeBody(body: any): void {
    this.excludeBody = body;
  }

  /** Update robot pose. Position is capsule center (odom); we apply lidar mount offset internally. */
  updatePose(x: number, y: number, z: number, qx: number, qy: number, qz: number, qw: number): void {
    this.px = x;
    this.py = y + this.lidarYOffset; // capsule center → lidar mount height
    this.pz = z;
    this.qx = qx;
    this.qy = qy;
    this.qz = qz;
    this.qw = qw;
    this.hasPose = true;
  }

  start(): void {
    if (this.timer) return;
    // quiet
    this.timer = setInterval(() => this._scan(), this.rateMs);
  }

  stop(): void {
    if (this.timer) {
      clearInterval(this.timer);
      this.timer = null;
    }
  }

  private _scan(): void {
    if (!this.hasPose || this.publishing) return;
    this.publishing = true;
    this._doScan().catch((e) => {
      console.warn("[lidar] publish error (dropped frame):", e?.message || e);
    }).finally(() => { this.publishing = false; });
  }

  private async _doScan(): Promise<void> {
      const profile = ServerLidar.PROFILE;
      const scanStart = profile ? performance.now() : 0;
      this.scanCount++;
      const jitterAngle = this.scanCount * 2.399963; // golden angle per scan
      const cosJ = Math.cos(jitterAngle);
      const sinJ = Math.sin(jitterAngle);

      const RAPIER = this.RAPIER;
      const world = this.world;

      // Pre-allocate output buffers
      const worldPts = new Float32Array(NUM_POINTS * 3);
      const intensity = new Float32Array(NUM_POINTS);
      let n = 0;

      const ox = this.px, oy = this.py, oz = this.pz;
      const rqx = this.qx, rqy = this.qy, rqz = this.qz, rqw = this.qw;

      const raycastStart = profile ? performance.now() : 0;

      for (let i = 0; i < NUM_POINTS; i++) {
        // Yield between chunks so physics ticks and LCM I/O interleave with
        // the scan instead of stalling behind it.
        if (i > 0 && i % RAYCAST_CHUNK === 0) await _yieldLoop();
        // Fibonacci direction (pre-rotated to camera-local) with per-scan golden angle jitter.
        // In FLU frame, jitter rotates around Z (up). After lidarToCamQuat, FLU Z → cam Y,
        // so jitter must rotate around camera-local Y axis.
        const fx = fibDirs[i * 3 + 0], fy = fibDirs[i * 3 + 1], fz = fibDirs[i * 3 + 2];
        const lx =  fx * cosJ + fz * sinJ;
        const ly =  fy;
        const lz = -fx * sinJ + fz * cosJ;

        // Rotate local direction by robot quaternion → world direction
        const [dx, dy, dz] = rotateByQuat(lx, ly, lz, rqx, rqy, rqz, rqw);
        const len = Math.sqrt(dx * dx + dy * dy + dz * dz);
        if (len < 1e-8) continue;
        const nx = dx / len, ny = dy / len, nz = dz / len;

        // Reuse ray object — avoids 20k allocations per scan
        this.ray.origin.x = ox; this.ray.origin.y = oy; this.ray.origin.z = oz;
        this.ray.dir.x = nx; this.ray.dir.y = ny; this.ray.dir.z = nz;
        // Use world.castRay (not queryPipeline.castRayAndGetNormal) —
        // the pipeline API crashes on restored snapshot worlds.
        // Exclude agent's own rigid body so lidar doesn't hit its own colliders.
        const hit = world.castRay(
          this.ray, MAX_RANGE, false,
          undefined, undefined, undefined,
          this.excludeBody,
        );

        if (!hit) continue;
        const toi = hit.timeOfImpact ?? 0;
        if (toi < MIN_RANGE || toi > MAX_RANGE) continue;

        worldPts[n * 3 + 0] = ox + nx * toi;
        worldPts[n * 3 + 1] = oy + ny * toi;
        worldPts[n * 3 + 2] = oz + nz * toi;
        intensity[n] = 1.0 / (1.0 + 0.02 * toi * toi);
        n++;
      }

      const raycastEnd = profile ? performance.now() : 0;

      if (n === 0) return;

      this.logN++;
      // scan logging removed — too noisy

      // Encode PointCloud2: Y-up → Z-up (ROS) cyclic permutation x→y, y→z, z→x
      const pointStep = 16;
      const buf = new ArrayBuffer(n * pointStep);
      const view = new DataView(buf);

      for (let i = 0; i < n; i++) {
        const off = i * pointStep;
        const tx = worldPts[i * 3 + 0];
        const ty = worldPts[i * 3 + 1];
        const tz = worldPts[i * 3 + 2];
        view.setFloat32(off, tz, true);       // ROS x = Three.js z
        view.setFloat32(off + 4, tx, true);   // ROS y = Three.js x
        view.setFloat32(off + 8, ty, true);   // ROS z = Three.js y
        view.setFloat32(off + 12, intensity[i], true);
      }

      const now = Date.now();
      const header = new std_msgs.Header({
        stamp: new std_msgs.Time({ sec: Math.floor(now / 1000), nsec: (now % 1000) * 1_000_000 }),
        frame_id: "world",
      });

      const msg = new sensor_msgs.PointCloud2({
        header,
        height: 1,
        width: n,
        fields_length: 4,
        fields: [
          new sensor_msgs.PointField({ name: "x", offset: 0, datatype: 7, count: 1 }),
          new sensor_msgs.PointField({ name: "y", offset: 4, datatype: 7, count: 1 }),
          new sensor_msgs.PointField({ name: "z", offset: 8, datatype: 7, count: 1 }),
          new sensor_msgs.PointField({ name: "intensity", offset: 12, datatype: 7, count: 1 }),
        ],
        is_bigendian: false,
        point_step: pointStep,
        row_step: n * pointStep,
        data_length: n * pointStep,
        data: new Uint8Array(buf),
        is_dense: true,
      });

      // Mark seq for echo filtering (prevent server re-forwarding to browser WS)
      this.sentSeqs.add(this.lcm.getNextSeq());
      // Publish directly to LCM — no WS hop (await so buffer pressure is felt)
      await this.lcm.publish(CH_LIDAR, msg);

      if (profile) {
        const total = performance.now() - scanStart;
        const raycast = raycastEnd - raycastStart;
        // Log every scan — there are only 10 per second.
        if (this.scanCount % 5 === 0) {
          console.log(
            `[lidar-prof] scan=${this.scanCount} raycast=${raycast.toFixed(1)}ms ` +
            `total=${total.toFixed(1)}ms hits=${n}/${NUM_POINTS}`
          );
        }
      }
  }
}
