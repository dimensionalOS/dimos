/**
 * DimSim's Rapier physics as a sim2 backend: one world, stepped only when asked.
 *
 * Reads one JSON request per line on stdin and writes one JSON reply per line on stdout:
 *   {op:"init", timestep, bodies:[scene_data.v1 body], robots:[{id, pos, yaw, halfExtents, center, mass}]}
 *   {op:"spawn", body:<scene_data.v1 body>, mass}   {op:"remove", entity}
 *   {op:"velocity", entity, linear:[x,y,z], angular:[x,y,z]}   (in the entity's own frame, held)
 *   {op:"pose", entity, pos:[x,y,z], quat:[x,y,z,w]}
 *   {op:"attach", entity, to}   {op:"detach", entity}   (carry an entity with another, as placed now)
 *   {op:"step", steps}  -> {time, entities:{name:{pos, quat, linear, angular}}}  (only the awake: Rapier's
 *   sleeping bodies have not moved; init's reply has every one)
 * Z is up. A robot is a kinematic box moved by DimSim's character controller, so it slides along
 * static geometry and pushes dynamic bodies; everything else is ordinary Rapier rigid bodies.
 */

import RAPIER from "@dimforge/rapier3d-compat";

type V3 = [number, number, number];
type Q = [number, number, number, number];
interface Geom {
  name: string;
  type: string;
  size: number[];
  pos: V3;
  quat: Q;
  mesh?: string;
}
interface Body {
  name: string;
  pos: V3;
  quat: Q;
  geoms: Geom[];
  mass?: number;
}
interface Mesh {
  vertices: string;
  faces: string;
}
interface Robot {
  id: string;
  pos: V3; // the root body's
  yaw: number;
  halfExtents: V3;
  center: V3; // the box's center from the root body
  mass: number;
}

// The character controller's settings, as DimSim's ServerPhysics has them.
const CONTROLLER_OFFSET = 0.05;
const MAX_STEP_HEIGHT = 0.25;
const GROUND_SNAP = 0.5;
const MAX_SLOPE_DEG = 45;
const GRAVITY = -9.81;
// MuJoCo's cylinders and capsules run along z; Rapier's along y.
const Y_TO_Z = { x: Math.SQRT1_2, y: 0, z: 0, w: Math.SQRT1_2 };

await RAPIER.init();
let world = new RAPIER.World({ x: 0, y: 0, z: GRAVITY });
let time = 0;
const named = new Map<string, any>(); // entity -> rigid body
const robots = new Map<string, { body: any; collider: any; controller: any; yaw: number; linear: V3; angular: V3 }>();
const velocities = new Map<string, { linear: V3; angular: V3 }>(); // held velocities of non-robot bodies
const carried = new Map<string, { to: string; offset: any; rotation: any }>(); // entity -> its carrier and pose in it

const floats = (b64: string) => {
  const raw = Uint8Array.from(atob(b64), (c) => c.charCodeAt(0));
  return new Float32Array(raw.buffer, raw.byteOffset, raw.byteLength / 4);
};
const uints = (b64: string) => {
  const raw = Uint8Array.from(atob(b64), (c) => c.charCodeAt(0));
  return new Uint32Array(raw.buffer, raw.byteOffset, raw.byteLength / 4);
};
const quat = (q: Q) => ({ x: q[0], y: q[1], z: q[2], w: q[3] });
const multiply = (a: any, b: any) => ({
  w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
  x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
  y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
  z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
});

function shape(geom: Geom, meshes: Record<string, Mesh>): any | null {
  const [a, b, c] = geom.size;
  switch (geom.type) {
    case "plane":
      // A slab under the plane (MuJoCo's size 0 means infinite); this Rapier has no half-space shape.
      return RAPIER.ColliderDesc.cuboid(a > 0 ? a : 1000, b > 0 ? b : 1000, 0.5);
    case "box":
      return RAPIER.ColliderDesc.cuboid(a, b, c);
    case "sphere":
      return RAPIER.ColliderDesc.ball(a);
    case "capsule":
      return RAPIER.ColliderDesc.capsule(b, a);
    case "cylinder":
      return RAPIER.ColliderDesc.cylinder(b, a);
    case "mesh": {
      const mesh = geom.mesh ? meshes[geom.mesh] : undefined;
      return mesh ? RAPIER.ColliderDesc.trimesh(floats(mesh.vertices), uints(mesh.faces)) : null;
    }
    default:
      return null;
  }
}

function addBody(body: Body, meshes: Record<string, Mesh>): void {
  const dynamic = (body.mass ?? 0) > 0;
  const desc = (dynamic ? RAPIER.RigidBodyDesc.dynamic() : RAPIER.RigidBodyDesc.fixed())
    .setTranslation(...body.pos)
    .setRotation(quat(body.quat));
  const rigid = world.createRigidBody(desc);
  const colliders = body.geoms.map((geom) => [geom, shape(geom, meshes)] as const).filter(([, d]) => d);
  for (const [geom, desc] of colliders) {
    let rotation = quat(geom.quat);
    if (geom.type === "capsule" || geom.type === "cylinder") {
      rotation = multiply(rotation, Y_TO_Z);
    }
    const [x, y, z] = geom.pos;
    desc.setTranslation(x, y, geom.type === "plane" ? z - 0.5 : z).setRotation(rotation);
    if (dynamic) {
      desc.setMass(body.mass! / colliders.length);
    }
    world.createCollider(desc, rigid);
  }
  named.set(body.name, rigid);
}

function addRobot(robot: Robot): void {
  const body = world.createRigidBody(
    RAPIER.RigidBodyDesc.kinematicPositionBased()
      .setTranslation(...robot.pos)
      .setRotation({ x: 0, y: 0, z: Math.sin(robot.yaw / 2), w: Math.cos(robot.yaw / 2) }),
  );
  const collider = world.createCollider(
    RAPIER.ColliderDesc.cuboid(...robot.halfExtents).setTranslation(...robot.center),
    body,
  );
  const controller = world.createCharacterController(CONTROLLER_OFFSET);
  controller.setUp({ x: 0, y: 0, z: 1 });
  controller.enableAutostep(MAX_STEP_HEIGHT, 0.15, true);
  controller.enableSnapToGround(GROUND_SNAP);
  controller.setSlideEnabled(true);
  controller.setMaxSlopeClimbAngle((MAX_SLOPE_DEG * Math.PI) / 180);
  // Walking into a dynamic body pushes it, as hard as a body this heavy would.
  controller.setApplyImpulsesToDynamicBodies(true);
  controller.setCharacterMass(robot.mass);
  robots.set(robot.id, { body, collider, controller, yaw: robot.yaw, linear: [0, 0, 0], angular: [0, 0, 0] });
  named.set(robot.id, body);
}

function moveRobots(dt: number): void {
  for (const robot of robots.values()) {
    robot.yaw += robot.angular[2] * dt;
    const [vx, vy, vz] = robot.linear;
    const cos = Math.cos(robot.yaw);
    const sin = Math.sin(robot.yaw);
    const desired = {
      x: (vx * cos - vy * sin) * dt,
      y: (vx * sin + vy * cos) * dt,
      z: vz * dt + GRAVITY * dt * dt * 0.5,
    };
    robot.controller.computeColliderMovement(robot.collider, desired, RAPIER.QueryFilterFlags.EXCLUDE_SENSORS);
    const moved = robot.controller.computedMovement();
    const at = robot.body.translation();
    robot.body.setNextKinematicTranslation({ x: at.x + moved.x, y: at.y + moved.y, z: at.z + moved.z });
    robot.body.setNextKinematicRotation({ x: 0, y: 0, z: Math.sin(robot.yaw / 2), w: Math.cos(robot.yaw / 2) });
  }
}

const conjugate = (q: any) => ({ w: q.w, x: -q.x, y: -q.y, z: -q.z });
const rotate = (q: any, v: any) => {
  const p = multiply(multiply(q, { w: 0, x: v.x, y: v.y, z: v.z }), conjugate(q));
  return { x: p.x, y: p.y, z: p.z };
};

function holdCarried(): void {
  for (const [name, { to, offset, rotation }] of carried) {
    const body = named.get(name);
    const carrier = named.get(to);
    if (!body || !carrier) {
      continue;
    }
    const at = carrier.translation();
    const q = carrier.rotation();
    const shift = rotate(q, offset);
    body.setTranslation({ x: at.x + shift.x, y: at.y + shift.y, z: at.z + shift.z }, true);
    body.setRotation(multiply(q, rotation), true);
    body.setLinvel({ x: 0, y: 0, z: 0 }, true);
    body.setAngvel({ x: 0, y: 0, z: 0 }, true);
  }
}

function holdVelocities(): void {
  for (const [name, velocity] of velocities) {
    const body = named.get(name);
    if (!body) {
      continue;
    }
    const r = body.rotation();
    const rotate = (v: V3) => {
      const p = multiply(multiply(r, { w: 0, x: v[0], y: v[1], z: v[2] }), { w: r.w, x: -r.x, y: -r.y, z: -r.z });
      return { x: p.x, y: p.y, z: p.z };
    };
    body.setLinvel(rotate(velocity.linear), true);
    body.setAngvel(rotate(velocity.angular), true);
  }
}

function state(awakeOnly: boolean): unknown {
  const entities: Record<string, unknown> = {};
  for (const [name, body] of named) {
    if (body.isFixed() || (awakeOnly && body.isSleeping() && !robots.has(name))) {
      continue;
    }
    const p = body.translation();
    const q = body.rotation();
    const robot = robots.get(name);
    const v = body.linvel();
    const w = body.angvel();
    // A robot's velocity is the one commanded (its own frame); a body's is Rapier's (world frame).
    entities[name] = {
      pos: [p.x, p.y, p.z],
      quat: [q.x, q.y, q.z, q.w],
      linear: robot ? robot.linear : [v.x, v.y, v.z],
      angular: robot ? robot.angular : [w.x, w.y, w.z],
    };
  }
  return { time, entities };
}

function handle(request: any): unknown {
  switch (request.op) {
    case "init": {
      world = new RAPIER.World({ x: 0, y: 0, z: GRAVITY });
      world.timestep = request.timestep;
      named.clear();
      robots.clear();
      velocities.clear();
      carried.clear();
      time = 0;
      for (const body of request.bodies) {
        addBody(body, request.meshes ?? {});
      }
      for (const robot of request.robots) {
        addRobot(robot);
      }
      return state(false);
    }
    case "spawn":
      addBody({ ...request.body, mass: request.mass }, request.meshes ?? {});
      return {};
    case "remove": {
      const body = named.get(request.entity);
      if (body) {
        world.removeRigidBody(body);
        named.delete(request.entity);
        velocities.delete(request.entity);
      }
      return {};
    }
    case "velocity": {
      const robot = robots.get(request.entity);
      if (robot) {
        robot.linear = request.linear;
        robot.angular = request.angular;
      } else if (named.has(request.entity)) {
        velocities.set(request.entity, { linear: request.linear, angular: request.angular });
      } else {
        return { error: `no entity ${request.entity}` };
      }
      return {};
    }
    case "pose": {
      const body = named.get(request.entity);
      if (!body) {
        return { error: `no entity ${request.entity}` };
      }
      body.setTranslation({ x: request.pos[0], y: request.pos[1], z: request.pos[2] }, true);
      body.setRotation(quat(request.quat), true);
      const robot = robots.get(request.entity);
      if (robot) {
        const q = request.quat;
        robot.yaw = Math.atan2(2 * (q[3] * q[2] + q[0] * q[1]), 1 - 2 * (q[1] * q[1] + q[2] * q[2]));
      }
      return {};
    }
    case "attach": {
      const body = named.get(request.entity);
      const carrier = named.get(request.to);
      if (!body || !carrier || body.isFixed()) {
        return { error: `cannot carry ${request.entity} with ${request.to}` };
      }
      const inverse = conjugate(carrier.rotation());
      const a = body.translation();
      const b = carrier.translation();
      carried.set(request.entity, {
        to: request.to,
        offset: rotate(inverse, { x: a.x - b.x, y: a.y - b.y, z: a.z - b.z }),
        rotation: multiply(inverse, body.rotation()),
      });
      return {};
    }
    case "detach":
      carried.delete(request.entity);
      return {};
    case "step": {
      for (let i = 0; i < (request.steps ?? 1); i++) {
        moveRobots(world.timestep);
        holdVelocities();
        holdCarried();
        world.step();
        time += world.timestep;
      }
      return state(true);
    }
    default:
      return { error: `unknown op ${request.op}` };
  }
}

const encoder = new TextEncoder();
let buffered = "";
for await (const chunk of Deno.stdin.readable.pipeThrough(new TextDecoderStream())) {
  buffered += chunk;
  let newline;
  while ((newline = buffered.indexOf("\n")) >= 0) {
    const line = buffered.slice(0, newline);
    buffered = buffered.slice(newline + 1);
    if (!line.trim()) {
      continue;
    }
    let reply: unknown;
    try {
      reply = handle(JSON.parse(line));
    } catch (error) {
      reply = { error: String(error) };
    }
    await Deno.stdout.write(encoder.encode(JSON.stringify(reply) + "\n"));
  }
}
