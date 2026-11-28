/**
 * DimSim's Rapier physics as a sim2 backend: one world, stepped only when asked.
 *
 * Reads one JSON request per line on stdin and writes one JSON reply per line on stdout:
 *   {op:"init", timestep, bodies:[scene_data.v1 body], robots:[{id, pos, yaw, halfExtents, center, mass}]}
 *   {op:"velocity", entity, linear:[x,y,z], angular:[x,y,z]}   (a robot's, in world axes, held)
 *   {op:"step", steps}  -> {time, entities:{name:{pos, quat, velocity, acceleration, yaw?}}}  (only the awake and
 *   those that fell asleep since the last reply: Rapier's sleeping bodies have not moved; init's reply has every one)
 * A velocity or acceleration is linear then angular, world axes, at the body's origin; an acceleration is
 * the change in velocity over the last timestep, so it leaves gravity out. A robot's yaw is the turn it has
 * made, never wrapped, so its yaw servo reads as the turns add up.
 * Z is up. A robot is a kinematic box moved by DimSim's character controller, so it slides along
 * static geometry and pushes dynamic bodies; everything else is ordinary Rapier rigid bodies.
 */

import RAPIER from "@dimforge/rapier3d-compat"

type V3 = [number, number, number]
type Q = [number, number, number, number]
interface Geom {
    name: string
    type: string
    size: number[]
    pos: V3
    quat: Q
    mesh?: string
}
interface Body {
    name: string
    pos: V3
    quat: Q
    geoms: Geom[]
    mass?: number
}
interface Mesh {
    vertices: string
    faces: string
}
interface Robot {
    id: string
    pos: V3 // the root body's
    yaw: number
    halfExtents: V3
    center: V3 // the box's center from the root body
    mass: number
}

// The character controller's settings, as DimSim's ServerPhysics has them.
const CONTROLLER_OFFSET = 0.05
const MAX_STEP_HEIGHT = 0.25
const GROUND_SNAP = 0.5
const MAX_SLOPE_DEG = 45
const GRAVITY = -9.81
// MuJoCo's cylinders and capsules run along z; Rapier's along y.
const Y_TO_Z = { x: Math.SQRT1_2, y: 0, z: 0, w: Math.SQRT1_2 }

await RAPIER.init()
let world = new RAPIER.World({ x: 0, y: 0, z: GRAVITY })
let time = 0
const named = new Map<string, any>() // entity -> rigid body
const robots = new Map<
    string,
    { body: any; collider: any; controller: any; yaw: number; linear: V3; angular: V3 }
>()
const motions = new Map<string, { velocity: number[]; acceleration: number[] }>() // entity -> its motion after the last timestep
const awake = new Set<string>() // the entities the last reply held awake
const STILL = [0, 0, 0, 0, 0, 0]

const floats = (b64: string) => {
    const raw = Uint8Array.from(atob(b64), (c) => c.charCodeAt(0))
    return new Float32Array(raw.buffer, raw.byteOffset, raw.byteLength / 4)
}
const uints = (b64: string) => {
    const raw = Uint8Array.from(atob(b64), (c) => c.charCodeAt(0))
    return new Uint32Array(raw.buffer, raw.byteOffset, raw.byteLength / 4)
}
const quat = (q: Q) => ({ x: q[0], y: q[1], z: q[2], w: q[3] })
const multiply = (a: any, b: any) => ({
    w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
    y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
    z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
})

function shape(geom: Geom, meshes: Record<string, Mesh>): any | null {
    const [a, b, c] = geom.size
    switch (geom.type) {
        case "plane":
            // A slab under the plane (MuJoCo's size 0 means infinite); this Rapier has no half-space shape.
            return RAPIER.ColliderDesc.cuboid(a > 0 ? a : 1000, b > 0 ? b : 1000, 0.5)
        case "box":
            return RAPIER.ColliderDesc.cuboid(a, b, c)
        case "sphere":
            return RAPIER.ColliderDesc.ball(a)
        case "capsule":
            return RAPIER.ColliderDesc.capsule(b, a)
        case "cylinder":
            return RAPIER.ColliderDesc.cylinder(b, a)
        case "mesh": {
            const mesh = geom.mesh ? meshes[geom.mesh] : undefined
            return mesh ? RAPIER.ColliderDesc.trimesh(floats(mesh.vertices), uints(mesh.faces)) : null
        }
        default:
            return null
    }
}

function addBody(body: Body, meshes: Record<string, Mesh>): void {
    const dynamic = (body.mass ?? 0) > 0
    const desc = (dynamic ? RAPIER.RigidBodyDesc.dynamic() : RAPIER.RigidBodyDesc.fixed())
        .setTranslation(...body.pos)
        .setRotation(quat(body.quat))
    const rigid = world.createRigidBody(desc)
    const colliders = body.geoms.map((geom) => [geom, shape(geom, meshes)] as const).filter(([, d]) => d)
    for (const [geom, desc] of colliders) {
        let rotation = quat(geom.quat)
        if (geom.type === "capsule" || geom.type === "cylinder") {
            rotation = multiply(rotation, Y_TO_Z)
        }
        const [x, y, z] = geom.pos
        desc.setTranslation(x, y, geom.type === "plane" ? z - 0.5 : z).setRotation(rotation)
        if (dynamic) {
            desc.setMass(body.mass! / colliders.length)
        }
        world.createCollider(desc, rigid)
    }
    named.set(body.name, rigid)
}

function addRobot(robot: Robot): void {
    const body = world.createRigidBody(
        RAPIER.RigidBodyDesc.kinematicPositionBased()
            .setTranslation(...robot.pos)
            .setRotation({ x: 0, y: 0, z: Math.sin(robot.yaw / 2), w: Math.cos(robot.yaw / 2) }),
    )
    const collider = world.createCollider(
        RAPIER.ColliderDesc.cuboid(...robot.halfExtents).setTranslation(...robot.center),
        body,
    )
    const controller = world.createCharacterController(CONTROLLER_OFFSET)
    controller.setUp({ x: 0, y: 0, z: 1 })
    controller.enableAutostep(MAX_STEP_HEIGHT, 0.15, true)
    controller.enableSnapToGround(GROUND_SNAP)
    controller.setSlideEnabled(true)
    controller.setMaxSlopeClimbAngle((MAX_SLOPE_DEG * Math.PI) / 180)
    // Walking into a dynamic body pushes it, as hard as a body this heavy would.
    controller.setApplyImpulsesToDynamicBodies(true)
    controller.setCharacterMass(robot.mass)
    robots.set(robot.id, {
        body,
        collider,
        controller,
        yaw: robot.yaw,
        linear: [0, 0, 0],
        angular: [0, 0, 0],
    })
    named.set(robot.id, body)
}

function moveRobots(dt: number): void {
    for (const robot of robots.values()) {
        robot.yaw += robot.angular[2] * dt
        const [vx, vy, vz] = robot.linear
        const desired = { x: vx * dt, y: vy * dt, z: vz * dt + GRAVITY * dt * dt * 0.5 }
        robot.controller.computeColliderMovement(
            robot.collider,
            desired,
            RAPIER.QueryFilterFlags.EXCLUDE_SENSORS,
        )
        const moved = robot.controller.computedMovement()
        const at = robot.body.translation()
        robot.body.setNextKinematicTranslation({ x: at.x + moved.x, y: at.y + moved.y, z: at.z + moved.z })
        robot.body.setNextKinematicRotation({
            x: 0,
            y: 0,
            z: Math.sin(robot.yaw / 2),
            w: Math.cos(robot.yaw / 2),
        })
    }
}

const conjugate = (q: any) => ({ w: q.w, x: -q.x, y: -q.y, z: -q.z })
const rotate = (q: any, v: any) => {
    const p = multiply(multiply(q, { w: 0, x: v.x, y: v.y, z: v.z }), conjugate(q))
    return { x: p.x, y: p.y, z: p.z }
}

// Rapier's velocity, which is its center of mass's, moved to the body's origin: v + w x (origin - com).
function velocityAtOrigin(body: any): number[] {
    const v = body.linvel()
    const w = body.angvel()
    // From its local center of mass: worldCom() is stale on a kinematic body, which Rapier never re-masses.
    const offset = rotate(body.rotation(), body.localCom())
    const r = { x: -offset.x, y: -offset.y, z: -offset.z }
    return [v.x + w.y * r.z - w.z * r.y, v.y + w.z * r.x - w.x * r.z, v.z + w.x * r.y - w.y * r.x, w.x, w.y, w.z]
}

// Each moving body's velocity, and its change over this timestep, after a step.
function measureMotions(dt: number): void {
    for (const [name, body] of named) {
        if (body.isFixed() || body.isSleeping()) {
            motions.delete(name)
            continue
        }
        const velocity = velocityAtOrigin(body)
        const before = motions.get(name)?.velocity ?? STILL
        motions.set(name, { velocity, acceleration: velocity.map((value, i) => (value - before[i]) / dt) })
    }
}

function state(awakeOnly: boolean): unknown {
    const entities: Record<string, unknown> = {}
    const nowAwake = new Set<string>()
    for (const [name, body] of named) {
        if (body.isFixed()) {
            continue
        }
        const sleeping = body.isSleeping() && !robots.has(name)
        if (!sleeping) {
            nowAwake.add(name)
        }
        // One that fell asleep since the last reply is told once more, still.
        if (awakeOnly && sleeping && !awake.has(name)) {
            continue
        }
        const p = body.translation()
        const q = body.rotation()
        const motion = sleeping ? undefined : motions.get(name)
        entities[name] = {
            pos: [p.x, p.y, p.z],
            quat: [q.x, q.y, q.z, q.w],
            velocity: motion?.velocity ?? STILL,
            acceleration: motion?.acceleration ?? STILL,
            ...(robots.has(name) ? { yaw: robots.get(name)!.yaw } : {}),
        }
    }
    awake.clear()
    for (const name of nowAwake) {
        awake.add(name)
    }
    return { time, entities }
}

function handle(request: any): unknown {
    switch (request.op) {
        case "init": {
            world = new RAPIER.World({ x: 0, y: 0, z: GRAVITY })
            world.timestep = request.timestep
            named.clear()
            robots.clear()
            motions.clear()
            awake.clear()
            time = 0
            for (const body of request.bodies) {
                addBody(body, request.meshes ?? {})
            }
            for (const robot of request.robots) {
                addRobot(robot)
            }
            return state(false)
        }
        case "velocity": {
            const robot = robots.get(request.entity)
            if (!robot) {
                return { error: `no robot ${request.entity}` }
            }
            robot.linear = request.linear
            robot.angular = request.angular
            return {}
        }
        case "step": {
            for (let i = 0; i < (request.steps ?? 1); i++) {
                moveRobots(world.timestep)
                world.step()
                measureMotions(world.timestep)
                time += world.timestep
            }
            return state(true)
        }
        default:
            return { error: `unknown op ${request.op}` }
    }
}

const encoder = new TextEncoder()
let buffered = ""
for await (const chunk of Deno.stdin.readable.pipeThrough(new TextDecoderStream())) {
    buffered += chunk
    let newline
    while ((newline = buffered.indexOf("\n")) >= 0) {
        const line = buffered.slice(0, newline)
        buffered = buffered.slice(newline + 1)
        if (!line.trim()) {
            continue
        }
        let reply: unknown
        try {
            reply = handle(JSON.parse(line))
        } catch (error) {
            reply = { error: String(error) }
        }
        await Deno.stdout.write(encoder.encode(JSON.stringify(reply) + "\n"))
    }
}
