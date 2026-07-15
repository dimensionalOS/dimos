/**
 * Server-side agent physics (Deno/Rapier).
 *
 * Runs the agent's kinematic character controller at a fixed timestep on the
 * server, eliminating the browser from the control loop:
 *
 *   Python cmd_vel → LCM → Deno (physics step) → LCM odom → Python
 *                                 ↓
 *                         WS position → Browser (render only)
 *
 * The browser no longer integrates cmd_vel or steps physics — it just receives
 * position updates and moves the visual avatar.
 */

import { geometry_msgs, std_msgs } from "@dimos/msgs";

import type { LCM } from "../vendor/lcm/lcm.ts";

// -- Agent dimensions (must match AiAvatar.js / engine.js) --------------------
const DEFAULT_AGENT_RADIUS = 0.12;
const DEFAULT_AGENT_HALF_HEIGHT = 0.25;
const CONTROLLER_OFFSET = 0.05;

// -- Physics constants --------------------------------------------------------
const PHYSICS_HZ = 50;
const PHYSICS_DT = 1.0 / PHYSICS_HZ;
// Upper bound on a single integration step. When the event loop stalls (lidar
// raycast blocks, GC, slow CI runners) ticks fire late; we integrate the real
// elapsed time so speed stays correct, but cap it so one late tick can't jump
// the robot far through geometry.
const MAX_STEP_DT = 0.1;
const DEFAULT_GRAVITY_Y = -9.81;
const DEFAULT_SPEED_SCALE = 3.0; // Multiplier for cmd_vel (linear + angular)
const DEFAULT_TURN_SCALE = 3.0;
const DEFAULT_MAX_ALTITUDE = 50;
const DEFAULT_WHEEL_BASE = 1.0;     // ackermann: front-rear axle distance (m)
const DEFAULT_MAX_STEER = 0.6;      // ackermann: max steering angle (rad)

/** Embodiment configuration passed from SceneClient / control channel. */
export interface EmbodimentConfig {
  radius?: number;
  halfHeight?: number;
  lidarMountHeight?: number;
  embodimentType?: string;   // legacy alias: "ground"->holonomic, "drone"->flight
  motionModel?: string;      // preferred: "holonomic" | "flight" | "ackermann"
  maxSpeed?: number;
  turnRate?: number;
  gravity?: number;
  maxStepHeight?: number;
  groundSnapDist?: number;
  maxSlopeAngle?: number;
  friction?: number;
  maxAltitude?: number;
  wheelBase?: number;        // ackermann: front-rear axle distance (m)
  maxSteerAngle?: number;    // ackermann: max steering angle (rad)
}

// ── Embodiment motion models ───────────────────────────────────────────────
// Each model maps the (already speed-scaled) cmd_vel + current yaw to a desired
// world displacement and a yaw delta. The shared step path applies that through
// the kinematic character controller (collision-aware) and publishes odom — so
// adding an embodiment is just a new function here plus config that selects it.
type MotionCmd = { linX: number; linY: number; linZ: number; angZ: number };
type MotionCfg = { gravity: number; maxAltitude: number; wheelBase: number; maxSteerAngle: number };
type MotionOut = { dx: number; dy: number; dz: number; dyaw: number; clampMaxY?: number };

const _clamp = (v: number, lo: number, hi: number) => Math.max(lo, Math.min(hi, v));

const MOTION_MODELS: Record<
  string,
  (cmd: MotionCmd, yaw: number, cfg: MotionCfg, dt: number) => MotionOut
> = {
  // Ground / holonomic: forward drive along heading, yaw from angular vel, gravity down.
  holonomic(cmd, yaw, cfg, dt) {
    const dyaw = cmd.angZ * dt;
    const y = yaw + dyaw;
    return {
      dx: cmd.linX * Math.sin(y) * dt,
      dz: cmd.linX * Math.cos(y) * dt,
      dy: cfg.gravity * dt * dt * 0.5,
      dyaw,
    };
  },
  // Drone / flight: 6DoF (forward, lateral, vertical), no gravity, altitude clamp.
  flight(cmd, yaw, cfg, dt) {
    const dyaw = cmd.angZ * dt;
    const y = yaw + dyaw;
    return {
      dx: (cmd.linX * Math.sin(y) + cmd.linY * Math.cos(y)) * dt,
      dz: (cmd.linX * Math.cos(y) - cmd.linY * Math.sin(y)) * dt,
      dy: cmd.linZ * dt,
      dyaw,
      clampMaxY: cfg.maxAltitude,
    };
  },
  // Car / Ackermann (bicycle model): angular cmd is the steering angle, and turn
  // rate scales with forward speed — so it can't pivot in place, it has to drive.
  ackermann(cmd, yaw, cfg, dt) {
    const v = cmd.linX;
    const steer = _clamp(cmd.angZ, -cfg.maxSteerAngle, cfg.maxSteerAngle);
    const dyaw = (v / Math.max(cfg.wheelBase, 0.01)) * Math.tan(steer) * dt;
    const y = yaw + dyaw;
    return {
      dx: v * Math.sin(y) * dt,
      dz: v * Math.cos(y) * dt,
      dy: cfg.gravity * dt * dt * 0.5,
      dyaw,
    };
  },
};

// Legacy embodimentType → motionModel mapping (back-compat for existing scenes).
function resolveMotionModel(embodiment?: EmbodimentConfig): string {
  const m = embodiment?.motionModel;
  if (m && MOTION_MODELS[m]) return m;
  if (embodiment?.embodimentType === "drone") return "flight";
  return "holonomic";
}

const CH_ODOM = "/odom#geometry_msgs.PoseStamped";
const CH_CMD_VEL = "/cmd_vel#geometry_msgs.Twist";
const CMD_VEL_TIMEOUT_MS = 500;

// -- ServerPhysics ------------------------------------------------------------

export class ServerPhysics {
  private lcm: LCM;
  private world: any; // RAPIER.World
  private RAPIER: any;
  private sentSeqs: Set<number>;

  private body: any;
  private collider: any;
  private spineCollider: any;
  private controller: any;
  private timer: ReturnType<typeof setInterval> | null = null;

  // Embodiment params
  private embodimentType: string;
  private motionModel: string;
  private speedScale: number;
  private turnScale: number;
  private gravity: number;
  private maxAltitude: number;
  private agentRadius: number;
  private agentHalfHeight: number;
  private friction: number;
  private maxStepHeight: number;
  private groundSnapDist: number;
  private maxSlopeAngle: number;
  private wheelBase: number;
  private maxSteerAngle: number;

  // Agent state
  private yaw = 0;
  private seq = 0;

  // Wall-clock time of the previous _step (ms); 0 until the first step ran.
  private lastStepAt = 0;

  // Profiling (DIMSIM_PROFILE_PHYSICS=1) — rolling timing per phase.
  private profile = false;
  private lastStepStart = 0;
  private prof = {
    n: 0,
    sumCompute: 0, maxCompute: 0,
    sumStep: 0, maxStep: 0,
    sumPublish: 0, maxPublish: 0,
    sumTotal: 0, maxTotal: 0,
    sumInterval: 0, maxInterval: 0,
  };

  // cmd_vel (ROS frame: x=fwd, z=yaw)
  private linX = 0; // forward
  private linY = 0; // lateral
  private linZ = 0; // vertical
  private angZ = 0; // yaw rotation
  private cmdVelStamp = 0;

  // Callback to send position to browser
  private onPoseUpdate: ((x: number, y: number, z: number, yaw: number) => void) | null = null;

  constructor(
    lcm: LCM,
    rapierWorld: any,
    RAPIER: any,
    sentSeqs: Set<number>,
    embodiment?: EmbodimentConfig,
  ) {
    this.lcm = lcm;
    this.world = rapierWorld;
    this.RAPIER = RAPIER;
    this.sentSeqs = sentSeqs;

    // Apply embodiment config with defaults
    this.embodimentType = embodiment?.embodimentType ?? "ground";
    this.motionModel = resolveMotionModel(embodiment);
    this.speedScale = embodiment?.maxSpeed ?? DEFAULT_SPEED_SCALE;
    this.turnScale = embodiment?.turnRate ?? DEFAULT_TURN_SCALE;
    this.gravity = embodiment?.gravity ?? DEFAULT_GRAVITY_Y;
    this.maxAltitude = embodiment?.maxAltitude ?? DEFAULT_MAX_ALTITUDE;
    this.agentRadius = embodiment?.radius ?? DEFAULT_AGENT_RADIUS;
    this.agentHalfHeight = embodiment?.halfHeight ?? DEFAULT_AGENT_HALF_HEIGHT;
    this.friction = embodiment?.friction ?? 0.8;
    this.maxStepHeight = embodiment?.maxStepHeight ?? 0.25;
    this.groundSnapDist = embodiment?.groundSnapDist ?? 0.5;
    this.maxSlopeAngle = embodiment?.maxSlopeAngle ?? 45;
    this.wheelBase = embodiment?.wheelBase ?? DEFAULT_WHEEL_BASE;
    this.maxSteerAngle = embodiment?.maxSteerAngle ?? DEFAULT_MAX_STEER;

    this._createBodyAndColliders();

    // Count colliders to verify world integrity
    let colliderCount = 0;
    this.world.colliders.forEach(() => { colliderCount++; });
    // Quiet init — only log on error or reconfigure

    this.profile = Deno.env.get("DIMSIM_PROFILE_PHYSICS") === "1";
    if (this.profile) {
      console.log(`[physics-prof] enabled — colliderCount=${colliderCount} target=${PHYSICS_HZ}Hz (${(1000 / PHYSICS_HZ).toFixed(1)}ms interval)`);
    }
  }

  private _createBodyAndColliders(): void {
    const RAPIER = this.RAPIER;

    // Create agent body (kinematic position-based, like AiAvatar)
    this.body = this.world.createRigidBody(
      RAPIER.RigidBodyDesc.kinematicPositionBased().setTranslation(0, 3, 0),
    );

    // Main capsule collider
    this.collider = this.world.createCollider(
      RAPIER.ColliderDesc.capsule(this.agentHalfHeight, this.agentRadius)
        .setFriction(this.friction),
      this.body,
    );

    // Spine collider (horizontal, behind body center — matches AiAvatar)
    const spineHalfLen = Math.max(this.agentRadius * 1.2, 0.13);
    const spineRadius = Math.max(this.agentRadius * 0.62, 0.07);
    const spineOffsetBack = Math.max(
      this.agentRadius * 2.2,
      spineHalfLen + spineRadius + 0.02,
    );
    const spineOffsetY = Math.max(this.agentHalfHeight * 0.35, 0.08);
    this.spineCollider = this.world.createCollider(
      RAPIER.ColliderDesc.capsule(spineHalfLen, spineRadius)
        .setFriction(this.friction)
        .setTranslation(0, spineOffsetY, -spineOffsetBack)
        .setRotation({
          x: Math.SQRT1_2,
          y: 0,
          z: 0,
          w: Math.SQRT1_2,
        }),
      this.body,
    );

    // Character controller
    this.controller = this.world.createCharacterController(CONTROLLER_OFFSET);
    this.controller.enableAutostep(this.maxStepHeight, 0.15, true);
    this.controller.enableSnapToGround(this.groundSnapDist);
    this.controller.setSlideEnabled(true);
    this.controller.setMaxSlopeClimbAngle((this.maxSlopeAngle * Math.PI) / 180);
    this.controller.setMinSlopeSlideAngle((75 * Math.PI) / 180);
  }

  /** Reconfigure physics with new embodiment params (e.g. after set_embodiment). */
  reconfigure(embodiment: EmbodimentConfig): void {
    // Save current position and yaw
    const pos = this.body.translation();
    const savedYaw = this.yaw;

    // Update params
    this.embodimentType = embodiment.embodimentType ?? this.embodimentType;
    if (embodiment.motionModel && MOTION_MODELS[embodiment.motionModel]) {
      this.motionModel = embodiment.motionModel;
    } else if (embodiment.embodimentType) {
      this.motionModel = resolveMotionModel(embodiment);
    }
    this.speedScale = embodiment.maxSpeed ?? this.speedScale;
    this.turnScale = embodiment.turnRate ?? this.turnScale;
    this.gravity = embodiment.gravity ?? this.gravity;
    this.maxAltitude = embodiment.maxAltitude ?? this.maxAltitude;
    this.agentRadius = embodiment.radius ?? this.agentRadius;
    this.agentHalfHeight = embodiment.halfHeight ?? this.agentHalfHeight;
    this.friction = embodiment.friction ?? this.friction;
    this.maxStepHeight = embodiment.maxStepHeight ?? this.maxStepHeight;
    this.groundSnapDist = embodiment.groundSnapDist ?? this.groundSnapDist;
    this.maxSlopeAngle = embodiment.maxSlopeAngle ?? this.maxSlopeAngle;
    this.wheelBase = embodiment.wheelBase ?? this.wheelBase;
    this.maxSteerAngle = embodiment.maxSteerAngle ?? this.maxSteerAngle;

    // Remove old colliders and body
    if (this.spineCollider) this.world.removeCollider(this.spineCollider, false);
    if (this.collider) this.world.removeCollider(this.collider, false);
    if (this.body) this.world.removeRigidBody(this.body);

    // Recreate with new params
    this._createBodyAndColliders();

    // Restore position and yaw
    this.body.setNextKinematicTranslation({ x: pos.x, y: pos.y, z: pos.z });
    this.yaw = savedYaw;
    this.world.step();

    console.log(`[physics] reconfigured: type=${this.embodimentType} model=${this.motionModel} radius=${this.agentRadius} halfHeight=${this.agentHalfHeight} speed=${this.speedScale} gravity=${this.gravity}`);
  }

  /** Set spawn position (Three.js Y-up). */
  setPosition(x: number, y: number, z: number): void {
    this.body.setNextKinematicTranslation({ x, y, z });
    this.world.step(); // apply immediately
    // quiet
  }

  /** Set callback for browser position sync. */
  setOnPoseUpdate(
    cb: (x: number, y: number, z: number, yaw: number) => void,
  ): void {
    this.onPoseUpdate = cb;
  }

  /** Handle incoming cmd_vel (ROS frame). */
  handleCmdVel(twist: any): void {
    this.linX = twist.linear.x; // forward (ROS +x)
    this.linY = twist.linear.y; // lateral
    this.linZ = twist.linear.z; // vertical
    this.angZ = twist.angular.z; // yaw (ROS +z = rotate left)
    this.cmdVelStamp = Date.now();
  }

  /** Subscribe to cmd_vel on LCM. */
  subscribeCmdVel(): void {
    this.lcm.subscribe(CH_CMD_VEL, geometry_msgs.Twist, (msg: any) => {
      this.handleCmdVel(msg.data);
    });
    // quiet
  }

  /** Start fixed-rate physics stepping + odom publish. */
  start(): void {
    if (this.timer) return;
    this.subscribeCmdVel();
    this.timer = setInterval(() => this._step(), 1000 / PHYSICS_HZ);
    // quiet
  }

  stop(): void {
    if (this.timer) {
      clearInterval(this.timer);
      this.timer = null;
    }
  }

  /** Get current position in Three.js Y-up frame. */
  getPosition(): { x: number; y: number; z: number } {
    return this.body.translation();
  }

  /** Get the agent's rigid body (for lidar exclusion). */
  getBody(): any {
    return this.body;
  }

  getYaw(): number {
    return this.yaw;
  }

  private _step(): void {
    const stepStart = this.profile ? performance.now() : 0;
    if (this.profile && this.lastStepStart) {
      const interval = stepStart - this.lastStepStart;
      this.prof.sumInterval += interval;
      if (interval > this.prof.maxInterval) this.prof.maxInterval = interval;
    }
    this.lastStepStart = stepStart;

    // Integrate over the real elapsed time, not the nominal tick period:
    // setInterval fires late whenever the event loop is busy (lidar scans take
    // tens of ms), and a fixed dt makes robot speed proportional to the
    // achieved tick rate instead of the commanded velocity.
    const now = performance.now();
    const dt = this.lastStepAt > 0
      ? Math.min((now - this.lastStepAt) / 1000, MAX_STEP_DT)
      : PHYSICS_DT;
    this.lastStepAt = now;

    // Safety timeout — zero velocity if no cmd_vel received recently
    const hasVel = Date.now() - this.cmdVelStamp < CMD_VEL_TIMEOUT_MS;
    const linX = hasVel ? this.linX * this.speedScale : 0;
    const linY = hasVel ? this.linY * this.speedScale : 0;
    const linZ = hasVel ? this.linZ * this.speedScale : 0;
    const angZ = hasVel ? this.angZ * this.turnScale : 0;

    const pos = this.body.translation();

    // Dispatch to the embodiment's motion model: (cmd_vel, yaw) -> displacement
    // + yaw delta. The collision/odom path below is shared across all models.
    const model = MOTION_MODELS[this.motionModel] ?? MOTION_MODELS.holonomic;
    const out = model(
      { linX, linY, linZ, angZ },
      this.yaw,
      {
        gravity: this.gravity,
        maxAltitude: this.maxAltitude,
        wheelBase: this.wheelBase,
        maxSteerAngle: this.maxSteerAngle,
      },
      dt,
    );
    this.yaw += out.dyaw;

    // Resolve the desired displacement against the world (collision-aware).
    this.controller.computeColliderMovement(
      this.collider,
      { x: out.dx, y: out.dy, z: out.dz },
      this.RAPIER.QueryFilterFlags.EXCLUDE_SENSORS,
    );
    const m = this.controller.computedMovement();
    const newPos = {
      x: pos.x + m.x,
      y: out.clampMaxY != null ? Math.min(pos.y + m.y, out.clampMaxY) : pos.y + m.y,
      z: pos.z + m.z,
    };

    this.body.setNextKinematicTranslation(newPos);

    const computeEnd = this.profile ? performance.now() : 0;

    // Step world to apply kinematic translation (needed for next computeColliderMovement)
    this.world.step();

    const stepEnd = this.profile ? performance.now() : 0;

    // Publish odom to LCM (Three.js Y-up → ROS Z-up)
    this._publishOdom(newPos);

    // Notify browser for visual sync
    if (this.onPoseUpdate) {
      this.onPoseUpdate(newPos.x, newPos.y, newPos.z, this.yaw);
    }

    if (this.profile) {
      const publishEnd = performance.now();
      const compute = computeEnd - stepStart;
      const step = stepEnd - computeEnd;
      const publish = publishEnd - stepEnd;
      const total = publishEnd - stepStart;
      const p = this.prof;
      p.n++;
      p.sumCompute += compute; if (compute > p.maxCompute) p.maxCompute = compute;
      p.sumStep += step;       if (step > p.maxStep) p.maxStep = step;
      p.sumPublish += publish; if (publish > p.maxPublish) p.maxPublish = publish;
      p.sumTotal += total;     if (total > p.maxTotal) p.maxTotal = total;

      // Log rolling averages every 20 steps (~0.4s at 50Hz target, ~4s at 5Hz actual).
      if (p.n >= 20) {
        const intervalAvg = p.sumInterval / Math.max(p.n - 1, 1);
        const effHz = intervalAvg > 0 ? 1000 / intervalAvg : 0;
        console.log(
          `[physics-prof] n=${p.n} ` +
          `compute=${(p.sumCompute / p.n).toFixed(2)}/${p.maxCompute.toFixed(1)}ms ` +
          `step=${(p.sumStep / p.n).toFixed(2)}/${p.maxStep.toFixed(1)}ms ` +
          `pub=${(p.sumPublish / p.n).toFixed(2)}/${p.maxPublish.toFixed(1)}ms ` +
          `total=${(p.sumTotal / p.n).toFixed(2)}/${p.maxTotal.toFixed(1)}ms ` +
          `interval=${intervalAvg.toFixed(1)}/${p.maxInterval.toFixed(1)}ms ` +
          `effHz=${effHz.toFixed(1)}`
        );
        // reset rolling counters but keep maxima — useful to spot worst-case drift
        p.n = 0;
        p.sumCompute = p.sumStep = p.sumPublish = p.sumTotal = p.sumInterval = 0;
      }
    }
  }

  private _publishOdom(pos: { x: number; y: number; z: number }): void {
    // Three.js Y-up → ROS Z-up: (x,y,z) → (z,x,y)
    const rosX = pos.z;
    const rosY = pos.x;
    const rosZ = pos.y;

    // Yaw quaternion (Three.js Y-axis → ROS Z-axis)
    const qw = Math.cos(this.yaw / 2);
    const qRosZ = Math.sin(this.yaw / 2); // rotation about ROS Z

    const now = Date.now();

    const header = new std_msgs.Header({
      seq: this.seq++,
      stamp: new std_msgs.Time({ sec: Math.floor(now / 1000), nsec: (now % 1000) * 1_000_000 }),
      frame_id: "world",
    });

    const pose = new geometry_msgs.Pose();
    pose.position = new geometry_msgs.Point();
    pose.position.x = rosX;
    pose.position.y = rosY;
    pose.position.z = rosZ;
    pose.orientation = new geometry_msgs.Quaternion();
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = qRosZ;
    pose.orientation.w = qw;

    const odom = new geometry_msgs.PoseStamped();
    odom.header = header;
    odom.pose = pose;

    try {
      this.sentSeqs.add(this.lcm.getNextSeq());
      this.lcm.publishRaw(CH_ODOM, odom.encode()).catch(() => {});
    } catch (e: unknown) {
      if (this.seq <= 3) console.warn("[physics] odom publish error:", e);
    }
  }
}
