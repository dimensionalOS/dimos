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
const DEFAULT_GRAVITY_Y = -9.81;
const DEFAULT_SPEED_SCALE = 3.0; // Multiplier for cmd_vel (linear + angular)
const DEFAULT_TURN_SCALE = 3.0;
const DEFAULT_MAX_ALTITUDE = 50;

/** Embodiment configuration passed from SceneClient / control channel. */
export interface EmbodimentConfig {
  radius?: number;
  halfHeight?: number;
  lidarMountHeight?: number;
  embodimentType?: string;   // "ground" | "drone"
  maxSpeed?: number;
  turnRate?: number;
  gravity?: number;
  maxStepHeight?: number;
  groundSnapDist?: number;
  maxSlopeAngle?: number;
  friction?: number;
  maxAltitude?: number;
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

  // Agent state
  private yaw = 0;
  private seq = 0;

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

    this._createBodyAndColliders();

    // Count colliders to verify world integrity
    let colliderCount = 0;
    this.world.colliders.forEach(() => { colliderCount++; });
    // Quiet init — only log on error or reconfigure
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

    console.log(`[physics] reconfigured: type=${this.embodimentType} radius=${this.agentRadius} halfHeight=${this.agentHalfHeight} speed=${this.speedScale} gravity=${this.gravity}`);
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
    // Safety timeout — zero velocity if no cmd_vel received recently
    const hasVel = Date.now() - this.cmdVelStamp < CMD_VEL_TIMEOUT_MS;
    const linX = hasVel ? this.linX * this.speedScale : 0;
    const linY = hasVel ? this.linY * this.speedScale : 0;
    const linZ = hasVel ? this.linZ * this.speedScale : 0;
    const angZ = hasVel ? this.angZ * this.turnScale : 0;

    // Integrate yaw (ROS angZ → Three.js Y rotation)
    // ROS +z yaw = CCW from above = Three.js +Y rotation
    this.yaw += angZ * PHYSICS_DT;

    const pos = this.body.translation();
    const cosY = Math.cos(this.yaw);
    const sinY = Math.sin(this.yaw);

    let newPos: { x: number; y: number; z: number };

    if (this.embodimentType === "drone") {
      // Drone: 6DoF movement, no gravity, altitude clamping
      const fwd = linX;
      const lat = linY;
      const vert = linZ; // ROS z = vertical for drone
      const desired = {
        x: (fwd * sinY + lat * cosY) * PHYSICS_DT,
        y: vert * PHYSICS_DT,
        z: (fwd * cosY - lat * sinY) * PHYSICS_DT,
      };

      this.controller.computeColliderMovement(
        this.collider,
        desired,
        this.RAPIER.QueryFilterFlags.EXCLUDE_SENSORS,
      );
      const m = this.controller.computedMovement();
      newPos = {
        x: pos.x + m.x,
        y: Math.min(pos.y + m.y, this.maxAltitude),
        z: pos.z + m.z,
      };
    } else {
      // Ground robot: gravity, collision-aware
      const fwd = linX;
      const desired = {
        x: (fwd * sinY) * PHYSICS_DT,
        y: this.gravity * PHYSICS_DT * PHYSICS_DT * 0.5, // gravity
        z: (fwd * cosY) * PHYSICS_DT,
      };

      this.controller.computeColliderMovement(
        this.collider,
        desired,
        this.RAPIER.QueryFilterFlags.EXCLUDE_SENSORS,
      );
      const m = this.controller.computedMovement();
      newPos = {
        x: pos.x + m.x,
        y: pos.y + m.y,
        z: pos.z + m.z,
      };
    }

    this.body.setNextKinematicTranslation(newPos);

    // Step world to apply kinematic translation (needed for next computeColliderMovement)
    this.world.step();

    // Publish odom to LCM (Three.js Y-up → ROS Z-up)
    this._publishOdom(newPos);

    // Debug: log first few steps
    // step logging removed — too noisy for dimos subprocess output

    // Notify browser for visual sync
    if (this.onPoseUpdate) {
      this.onPoseUpdate(newPos.x, newPos.y, newPos.z, this.yaw);
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
