// Interactive demo of the Rust repulsive-field local planner compiled to WASM.
// Drag obstacles / the goal; the costmap and path recompute live (a genuine
// solve per frame, same code the robot runs).
import * as THREE from "https://esm.sh/three@0.170.0"
import { OrbitControls } from "https://esm.sh/three@0.170.0/examples/jsm/controls/OrbitControls.js"
import init, { DemoPlanner } from "./pkg/dimos_repulsive_field.js"

await init()

const WORLD_HALF = 8.0     // matches the runtime half_extent
const RES = 0.1            // matches the runtime costmap resolution
const ROBOT_Z = 0.4

const planner = new DemoPlanner(RES, WORLD_HALF)

// --- three.js scaffolding -------------------------------------------------
const scene = new THREE.Scene()
scene.background = new THREE.Color(0x0b0e12)
const camera = new THREE.PerspectiveCamera(55, innerWidth / innerHeight, 0.1, 200)
camera.position.set(0, -10, 12)
camera.up.set(0, 0, 1)
const renderer = new THREE.WebGLRenderer({ antialias: true })
renderer.setSize(innerWidth, innerHeight)
document.body.appendChild(renderer.domElement)
addEventListener("resize", () => {
    camera.aspect = innerWidth / innerHeight
    camera.updateProjectionMatrix()
    renderer.setSize(innerWidth, innerHeight)
})

const controls = new OrbitControls(camera, renderer.domElement)
controls.mouseButtons = { LEFT: null, MIDDLE: THREE.MOUSE.PAN, RIGHT: THREE.MOUSE.ROTATE }
controls.target.set(0, 0, 0)

scene.add(new THREE.AmbientLight(0xffffff, 0.7))
const sun = new THREE.DirectionalLight(0xffffff, 1.2)
sun.position.set(5, -8, 12)
scene.add(sun)

// Costmap raster rendered as a texture on the ground plane.
const texSize = Math.round(2 * WORLD_HALF / RES)
const texData = new Uint8Array(texSize * texSize * 4)
const costTex = new THREE.DataTexture(texData, texSize, texSize, THREE.RGBAFormat)
costTex.needsUpdate = true
const ground = new THREE.Mesh(
    new THREE.PlaneGeometry(2 * WORLD_HALF, 2 * WORLD_HALF),
    new THREE.MeshBasicMaterial({ map: costTex })
)
scene.add(ground)

// Robot: a little wedge that chases the path kinematically.
const robot = { x: -5, y: -5, yaw: 0, speed: 0 }
const robotMesh = new THREE.Mesh(
    new THREE.ConeGeometry(0.25, 0.6, 12),
    new THREE.MeshStandardMaterial({ color: 0x7fd0ff })
)
robotMesh.rotation.x = Math.PI / 2 // cone +y -> planar heading, rotated per-frame
scene.add(robotMesh)

// Goal: draggable green disc.
const goal = new THREE.Mesh(
    new THREE.CylinderGeometry(0.35, 0.35, 0.06, 24),
    new THREE.MeshStandardMaterial({ color: 0x4ade80 })
)
goal.rotation.x = Math.PI / 2
goal.position.set(5, 5, 0.03)
scene.add(goal)

// Obstacles: draggable orange boxes (their footprints become terrain points).
const obstacles = []
function addObstacle(x, y, w, d) {
    const h = 1.4
    const box = new THREE.Mesh(
        new THREE.BoxGeometry(w, d, h),
        new THREE.MeshStandardMaterial({ color: 0xf59e0b })
    )
    box.position.set(x, y, h / 2)
    box.userData = { w, d, h }
    scene.add(box)
    obstacles.push(box)
}
addObstacle(0, 0, 2.0, 0.4)
addObstacle(2.5, 2.5, 0.4, 2.0)
addObstacle(-2.0, 2.0, 1.2, 1.2)

// Path line.
const pathGeom = new THREE.BufferGeometry()
const pathLine = new THREE.Line(
    pathGeom,
    new THREE.LineBasicMaterial({ color: 0xffffff, linewidth: 2 })
)
pathLine.position.z = 0.06
scene.add(pathLine)

// --- terrain synthesis ----------------------------------------------------
// Ground grid at costmap resolution + obstacle walls sampled densely.
function buildTerrain() {
    const pts = []
    // Ground grid around the ROBOT (the costmap window follows it).
    const gx = Math.round(robot.x / RES) * RES
    const gy = Math.round(robot.y / RES) * RES
    for (let x = gx - WORLD_HALF; x <= gx + WORLD_HALF; x += RES) {
        for (let y = gy - WORLD_HALF; y <= gy + WORLD_HALF; y += RES) {
            pts.push(x, y, 0)
        }
    }
    for (const box of obstacles) {
        const { w, d, h } = box.userData
        const cx = box.position.x, cy = box.position.y
        for (let x = -w / 2; x <= w / 2; x += RES / 2) {
            for (let y = -d / 2; y <= d / 2; y += RES / 2) {
                for (let z = 0; z <= h; z += RES) {
                    pts.push(cx + x, cy + y, z)
                }
            }
        }
    }
    return new Float32Array(pts)
}

let terrainDirty = true
let lastBuild = [Infinity, Infinity]

function paintCostmap() {
    const cells = planner.costmap_cells()
    const w = planner.costmap_width()
    const h = planner.costmap_height()
    if (w !== texSize || h !== texSize) {
        return
    }
    // The costmap window is robot-centered; move the raster under it.
    ground.position.x = planner.costmap_origin_x() + WORLD_HALF
    ground.position.y = planner.costmap_origin_y() + WORLD_HALF
    for (let i = 0; i < w * h; i++) {
        const c = cells[i]
        let r, g, b
        if (c < 0) {              // unknown
            r = 26; g = 30; b = 38
        } else if (c >= 100) {    // lethal
            r = 220; g = 60; b = 60
        } else {                  // free..costly
            r = 30 + c * 1.6; g = 120 - c * 0.5; b = 60
        }
        texData[i * 4] = r
        texData[i * 4 + 1] = g
        texData[i * 4 + 2] = b
        texData[i * 4 + 3] = 255
    }
    costTex.needsUpdate = true
}

// --- dragging ---------------------------------------------------------------
const ray = new THREE.Raycaster()
const groundPlane = new THREE.Plane(new THREE.Vector3(0, 0, 1), 0)
let dragging = null
function pointerWorld(e) {
    const ndc = new THREE.Vector2(
        (e.clientX / innerWidth) * 2 - 1,
        -(e.clientY / innerHeight) * 2 + 1
    )
    ray.setFromCamera(ndc, camera)
    const hit = new THREE.Vector3()
    ray.ray.intersectPlane(groundPlane, hit)
    return hit
}
renderer.domElement.addEventListener("pointerdown", e => {
    if (e.button !== 0) {
        return
    }
    const ndc = new THREE.Vector2(
        (e.clientX / innerWidth) * 2 - 1,
        -(e.clientY / innerHeight) * 2 + 1
    )
    ray.setFromCamera(ndc, camera)
    const hits = ray.intersectObjects([goal, ...obstacles])
    if (hits.length > 0) {
        dragging = hits[0].object
        controls.enabled = false
    }
})
addEventListener("pointermove", e => {
    if (!dragging) {
        return
    }
    const p = pointerWorld(e)
    dragging.position.x = p.x
    dragging.position.y = p.y
    if (dragging !== goal) {
        terrainDirty = true
    }
})
addEventListener("pointerup", () => {
    dragging = null
    controls.enabled = true
})

// --- main loop --------------------------------------------------------------
const stats = document.getElementById("stats")
let emaSolve = 0
function frame() {
    if (terrainDirty) {
        planner.update_terrain(buildTerrain(), robot.x, robot.y, ROBOT_Z)
        paintCostmap()
        terrainDirty = false
        lastBuild = [robot.x, robot.y]
    }
    const t0 = performance.now()
    const flat = planner.plan(robot.x, robot.y, robot.yaw, goal.position.x, goal.position.y, robot.speed)
    emaSolve = 0.95 * emaSolve + 0.05 * (performance.now() - t0)

    const n = flat.length / 3
    const pos = new Float32Array(n * 3)
    for (let i = 0; i < n; i++) {
        pos[i * 3] = flat[i * 3]
        pos[i * 3 + 1] = flat[i * 3 + 1]
        pos[i * 3 + 2] = 0
    }
    pathGeom.setAttribute("position", new THREE.BufferAttribute(pos, 3))
    pathGeom.computeBoundingSphere()

    // Kinematic chase: head toward the first point ~0.6 m along the path.
    if (n >= 2) {
        let target = null
        for (let i = 1; i < n; i++) {
            const dx = flat[i * 3] - robot.x, dy = flat[i * 3 + 1] - robot.y
            if (Math.hypot(dx, dy) > 0.6 || i === n - 1) {
                target = [flat[i * 3], flat[i * 3 + 1]]
                break
            }
        }
        const dGoal = Math.hypot(goal.position.x - robot.x, goal.position.y - robot.y)
        if (target && dGoal > 0.3) {
            const want = Math.atan2(target[1] - robot.y, target[0] - robot.x)
            let dyaw = want - robot.yaw
            dyaw = Math.atan2(Math.sin(dyaw), Math.cos(dyaw))
            robot.yaw += Math.max(-0.05, Math.min(0.05, dyaw))
            robot.speed = Math.abs(dyaw) > 1.0 ? 0 : Math.min(1.25, dGoal)
            robot.x += Math.cos(robot.yaw) * robot.speed / 60
            robot.y += Math.sin(robot.yaw) * robot.speed / 60
            // Rebuild the window only after real motion (build is ~ms, not free).
            const dx = robot.x - lastBuild[0], dy = robot.y - lastBuild[1]
            if (Math.hypot(dx, dy) > 0.5) {
                terrainDirty = true
            }
        } else {
            robot.speed = 0
        }
    }
    robotMesh.position.set(robot.x, robot.y, 0.3)
    robotMesh.rotation.z = robot.yaw - Math.PI / 2

    stats.textContent = `solve ${emaSolve.toFixed(2)} ms · path ${n} pts`
    controls.update()
    renderer.render(scene, camera)
    requestAnimationFrame(frame)
}
frame()
