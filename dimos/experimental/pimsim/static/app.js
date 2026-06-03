// Copyright 2025-2026 Dimensional Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

const canvas = document.getElementById("renderCanvas");
const statusEl = document.getElementById("status");
const ui = window.PimSimUI || {};
const engine = new BABYLON.Engine(canvas, true, {
  preserveDrawingBuffer: false,
  stencil: false,
  antialias: false,
});
const scene = new BABYLON.Scene(engine);
scene.useRightHandedSystem = true;
scene.clearColor = new BABYLON.Color4(0.055, 0.063, 0.078, 1);
scene.skipPointerMovePicking = true;
scene.autoClear = true;
scene.autoClearDepthAndStencil = true;

const camera = new BABYLON.ArcRotateCamera(
  "camera",
  -Math.PI / 2,
  Math.PI / 3,
  8,
  new BABYLON.Vector3(0, 0, 1),
  scene,
);
camera.upVector = new BABYLON.Vector3(0, 0, 1);
camera.minZ = 0.01;
camera.maxZ = 100000;
camera.wheelPrecision = 40;
camera.attachControl(canvas, true);

new BABYLON.HemisphericLight("skyLight", new BABYLON.Vector3(0.2, 0.4, 1), scene);
const sun = new BABYLON.DirectionalLight("sun", new BABYLON.Vector3(-0.4, -0.6, -1), scene);
sun.position = new BABYLON.Vector3(20, 30, 40);
scene.environmentIntensity = 0.85;
try {
  scene.environmentTexture = BABYLON.CubeTexture.CreateFromPrefilteredData(
    "https://assets.babylonjs.com/environments/environmentSpecular.env",
    scene,
  );
} catch (error) {
  console.warn("environment map unavailable", error);
}

const bodyNodes = new Map();
const bodyNodeList = [];
const bodyNameList = [];
const sceneMeshes = [];
const sceneMeshSet = new Set();
const collisionMeshes = [];
const collisionMeshSet = new Set();
const robotMeshes = [];
const maxAutoSceneBytes = 2 * 1024 * 1024 * 1024;

// Havok physics. Right-handed, z-up: gravity is -Z.
// `physicsReady` is a promise that resolves once HavokPhysics() returns and
// the plugin is wired. Consumers (collision-mesh wrapper, entity spawn)
// await it before creating aggregates.
const physicsGravity = new BABYLON.Vector3(0, 0, -9.8);
const physicsReady = (async () => {
  if (typeof HavokPhysics !== "function") {
    console.warn("HavokPhysics not loaded — physics disabled");
    return false;
  }
  try {
    const havok = await HavokPhysics();
    scene.enablePhysics(physicsGravity, new BABYLON.HavokPlugin(true, havok));
    console.log("Havok physics enabled");
    return true;
  } catch (error) {
    console.error("Havok init failed:", error);
    return false;
  }
})();

// Entity world. Browser is authoritative for state; Python pushes
// spawn/despawn/teleport via WS, browser reports per-tick state back.
const entities = new Map();              // entity_id -> {mesh, aggregate, descriptor}
let entityBroadcastIntervalMs = 1000 / 15;   // state changes only; 15 Hz is enough for sensors
let lastEntityBroadcast = 0;
let lastEntityBroadcastSignature = "";
let lastEntityBroadcastKeepalive = 0;
const entityBroadcastKeepaliveMs = 2000;

// Kinematic robot collider. Parented to the root body node so the odom-
// driven transform drags it through the world; entities collide with it.
// Humanoid defaults — adjust per robot if needed (no real-world cost to
// being a bit oversized for nav testing).
const ROBOT_COLLIDER_HEIGHT = 1.4;
const ROBOT_COLLIDER_RADIUS = 0.25;
const ROBOT_COLLIDER_Z_OFFSET = 0.7;    // capsule center above the root body origin
let robotColliderMesh = null;
let robotColliderAggregate = null;
const params = new URLSearchParams(window.location.search);
const useRobotMesh = params.get("robot") !== "proxy";
const sceneMode = params.get("scene") || "auto";
const showPerfHud = params.get("perf") === "1";
let sceneConfig = null;
let sceneLoadStarted = false;
let sceneVisible = true;
let pathMesh = null;
let lidarMesh = null;
let splatMesh = null;
let splatLoadStarted = false;
let splatVisible = false;
let lidarMaterial = null;
let lidarPointCount = 0;
let lidarVisible = true;
const lidarPointSizePx = 5.0;
let clickMode = null;
let navGoalMarker = null;
let pointGoalMarker = null;
let spawnMarker = null;
let latestRootPosition = null;
let sceneDepthEnabled = true;
let sceneWireEnabled = false;
let collisionVisible = false;
let collisionMaterial = null;
let forceVisibleEnabled = false;
let driveEnabled = false;
let lastDriveSendTime = 0;
let lastDriveSignature = "";
let proxyMaterial = null;
const pressedKeys = new Set();
const driveSendPeriod = 0.08;
const driveLinearSpeed = 0.35;
const driveStrafeSpeed = 0.25;
const driveAngularSpeed = 0.8;
let browserPhysicsEnabled = false;
let browserPhysicsPose = { x: 0, y: 0, z: 0.75, yaw: 0 };
let browserPhysicsCommand = {
  linear: [0, 0, 0],
  angular: [0, 0, 0],
};
let browserPhysicsLastStepMs = null;
let browserPhysicsLastOdomMs = 0;
let browserVehicleHeight = 0.75;
let browserStepOffset = 0.22;
let supportFloorEnabled = false;
let supportFloorZ = 0.0;
let supportFloorSize = 0.0;
let supportFloorMesh = null;
let supportFloorAggregate = null;
const browserMaxStepDown = 0.5;
const browserGroundProbeExtra = 1.0;
const browserPhysicsOdomPeriodMs = 1000 / 50;
const browserCharacterRadius = ROBOT_COLLIDER_RADIUS;
const browserCollisionProbeHeights = [0.25, 0.75, 1.2];
const supportFloorThickness = 0.08;
const robotPoseHeaderBytes = 16;
const stateQueueMaxLength = 8;
const stateImmediateDeltaMs = 100;
const stateMaxLagMs = 100;
const stateEarlyThresholdMs = 3;
const statePacingDamping = 0.95;
const robotRootBodyNames = new Set(["pelvis", "torso_link", "body_1", "base"]);
const posePositionEpsilon = 1e-5;
const poseQuaternionEpsilon = 1e-5;
const queuedStateFrames = [];
const lastAppliedRobotPoses = [];
let latestRobotPosePayload = null;
let robotRootBodyIndex = -1;
const robotPoseComposeScale = BABYLON.Vector3.One();
const robotPoseDecomposeScale = BABYLON.Vector3.One();
const robotPosePositionScratch = BABYLON.Vector3.Zero();
const robotPoseQuaternionScratch = BABYLON.Quaternion.Identity();
const robotPoseMatrixScratch = BABYLON.Matrix.Identity();
const robotPoseRootMatrixScratch = BABYLON.Matrix.Identity();
const robotPoseRootInverseMatrixScratch = BABYLON.Matrix.Identity();
const robotPoseBrowserRootMatrixScratch = BABYLON.Matrix.Identity();
const robotPoseRebaseMatrixScratch = BABYLON.Matrix.Identity();
const robotPoseRebasedMatrixScratch = BABYLON.Matrix.Identity();
const stateTiming = {
  previousSourceMs: null,
  lastIdealJsMs: null,
  jsMinusSourceMs: Number.POSITIVE_INFINITY,
};
const streamRef = { worker: null, ready: false };
let lastPathVersion = null;
const perfCounters = {
  lastReportMs: performance.now(),
  frames: 0,
  poseFrames: 0,
  poseBodiesUpdated: 0,
  poseBodiesSkipped: 0,
  pointcloudFrames: 0,
  pointcloudBufferUpdates: 0,
  pointcloudRecreates: 0,
};

const vec3 = (values) => new BABYLON.Vector3(values[0], values[1], values[2]);
const quatWxyz = (values) =>
  new BABYLON.Quaternion(values[1], values[2], values[3], values[0]);

function finiteNumber(value, fallback) {
  const number = Number(value);
  return Number.isFinite(number) ? number : fallback;
}

function registerSceneMesh(mesh) {
  sceneMeshes.push(mesh);
  sceneMeshSet.add(mesh);
}

function registerCollisionMesh(mesh) {
  collisionMeshes.push(mesh);
  collisionMeshSet.add(mesh);
}

function unregisterCollisionMesh(mesh) {
  const index = collisionMeshes.indexOf(mesh);
  if (index >= 0) collisionMeshes.splice(index, 1);
  collisionMeshSet.delete(mesh);
}

function setStatus(message) {
  if (ui.setStatus) {
    ui.setStatus(message);
    return;
  }
  statusEl.textContent = message;
}

function updatePerfCounters() {
  perfCounters.frames += 1;
  const now = performance.now();
  const elapsedMs = now - perfCounters.lastReportMs;
  if (elapsedMs < 1000) return;

  const elapsedSeconds = elapsedMs / 1000;
  const summary = [
    `fps=${(perfCounters.frames / elapsedSeconds).toFixed(1)}`,
    `pose=${perfCounters.poseFrames}/s`,
    `bodies=${perfCounters.poseBodiesUpdated}/${perfCounters.poseBodiesSkipped}`,
    `pc=${perfCounters.pointcloudFrames}/s`,
    `pcbuf=${perfCounters.pointcloudBufferUpdates}/${perfCounters.pointcloudRecreates}`,
    `q=${queuedStateFrames.length}`,
  ].join(" ");
  statusEl.title = summary;
  if (showPerfHud) statusEl.textContent = summary;

  perfCounters.lastReportMs = now;
  perfCounters.frames = 0;
  perfCounters.poseFrames = 0;
  perfCounters.poseBodiesUpdated = 0;
  perfCounters.poseBodiesSkipped = 0;
  perfCounters.pointcloudFrames = 0;
  perfCounters.pointcloudBufferUpdates = 0;
  perfCounters.pointcloudRecreates = 0;
}

function setButtonActive(id, active) {
  if (ui.setButtonActive) {
    ui.setButtonActive(id, active);
    return;
  }
  document.getElementById(id).dataset.active = String(active);
}

function setRenderableVisible(node, visible) {
  if (!node) return;
  if ("isVisible" in node) node.isVisible = visible;
  if ("visibility" in node) node.visibility = visible ? 1 : 0;
}

function setEntityVisualsVisible(entry, visible) {
  if (!entry) return;
  if (entry.visualNodes && entry.visualNodes.length > 0) {
    for (const node of entry.visualNodes) {
      setRenderableVisible(node, visible);
    }
    return;
  }
  // Primitive entities often use their physics mesh as the visible mesh.
  // Hide the renderable surface only; keep the body enabled for Havok and
  // for browser->native entity-state publication.
  setRenderableVisible(entry.mesh, visible);
}

function setAllEntityVisualsVisible(visible) {
  for (const entry of entities.values()) {
    setEntityVisualsVisible(entry, visible);
  }
}

function applyCollisionVisibility() {
  const visible = sceneVisible && collisionVisible;
  for (const mesh of collisionMeshes) {
    // Only toggle visibility — setEnabled(false) would also stop the
    // PhysicsAggregate from colliding, which defeats the point of
    // running Havok against the cooked collision scene.
    mesh.visibility = visible ? 1 : 0;
  }
}

function setSceneVisibility(visible) {
  sceneVisible = visible;
  for (const mesh of sceneMeshes) setRenderableVisible(mesh, visible);
  setAllEntityVisualsVisible(visible);
  applyCollisionVisibility();
  setButtonActive("toggleScene", visible);
}

function setRobotVisibility(visible) {
  for (const mesh of robotMeshes) mesh.setEnabled(visible);
  setButtonActive("toggleRobot", visible);
}

function setLidarVisibility(visible) {
  lidarVisible = visible;
  if (lidarMesh) lidarMesh.setEnabled(visible);
  setButtonActive("toggleLidar", visible);
}

function setSplatVisibility(visible) {
  splatVisible = visible;
  setButtonActive("toggleSplat", visible);
  if (visible && !splatMesh && !splatLoadStarted && sceneConfig) {
    loadSplat(sceneConfig);
    return;
  }
  if (splatMesh) splatMesh.setEnabled(visible);
}

async function loadSplat(config) {
  if (splatLoadStarted || !config || !config.splatFile) return;
  splatLoadStarted = true;
  setStatus("loading splat");
  try {
    // splat is parented to the same scene root the mesh uses, so the package's
    // scene alignment (scale / translation / rotation) is applied exactly once
    // in the browser and the per-splat alignment stays source-frame relative.
    const root = new BABYLON.TransformNode("splatRoot", scene);
    root.position = vec3(config.scenePosition);
    root.scaling = new BABYLON.Vector3(config.sceneScale, config.sceneScale, config.sceneScale);
    root.rotationQuaternion = quatWxyz(config.sceneWxyz);

    const ply = "/assets/" + config.splatFile;
    const GS = BABYLON.GaussianSplattingMesh;
    if (!GS) {
      console.error("BABYLON.GaussianSplattingMesh not available — Babylon too old?");
      setStatus("splat unsupported");
      return;
    }
    const gs = new GS("splat", null, scene);
    await gs.loadFileAsync(ply);
    gs.parent = root;

    const a = config.splatAlignment || {};
    const s = Number(a.scale ?? 1.0);
    gs.scaling = new BABYLON.Vector3(s, s, s);
    const t = a.translation || [0, 0, 0];
    gs.position = new BABYLON.Vector3(t[0], t[1], t[2]);
    // SplatAlignment.rotation_zyx is intrinsic ZYX in degrees: R = Rz @ Ry @ Rx.
    const [rzDeg, ryDeg, rxDeg] = a.rotation_zyx || [0, 0, 0];
    const rz = (rzDeg * Math.PI) / 180;
    const ry = (ryDeg * Math.PI) / 180;
    const rx = (rxDeg * Math.PI) / 180;
    const qx = BABYLON.Quaternion.RotationAxis(new BABYLON.Vector3(1, 0, 0), rx);
    const qy = BABYLON.Quaternion.RotationAxis(new BABYLON.Vector3(0, 1, 0), ry);
    const qz = BABYLON.Quaternion.RotationAxis(new BABYLON.Vector3(0, 0, 1), rz);
    gs.rotationQuaternion = qz.multiply(qy).multiply(qx);
    if (a.y_up) {
      // Compose an extra Y-up -> Z-up rotation (-90° around X) on splat-local
      // before the ZYX above. Equivalent to inserting Rx(-90°) on the right.
      const qYupToZup = BABYLON.Quaternion.RotationAxis(
        new BABYLON.Vector3(1, 0, 0),
        -Math.PI / 2,
      );
      gs.rotationQuaternion = gs.rotationQuaternion.multiply(qYupToZup);
    }

    splatMesh = gs;
    splatMesh.setEnabled(splatVisible);
    setStatus("splat loaded");
  } catch (err) {
    console.error("splat load failed", err);
    setStatus("splat load failed");
    splatLoadStarted = false;
  }
}

function createCollisionMaterial() {
  if (collisionMaterial) return collisionMaterial;
  collisionMaterial = new BABYLON.StandardMaterial("collisionDebugMaterial", scene);
  collisionMaterial.diffuseColor = BABYLON.Color3.FromHexString("#00d7ff");
  collisionMaterial.emissiveColor = BABYLON.Color3.FromHexString("#00a8ff");
  collisionMaterial.specularColor = BABYLON.Color3.Black();
  collisionMaterial.alpha = 0.55;
  collisionMaterial.wireframe = true;
  collisionMaterial.backFaceCulling = false;
  collisionMaterial.disableLighting = true;
  collisionMaterial.transparencyMode = BABYLON.Material.MATERIAL_ALPHABLEND;
  return collisionMaterial;
}

function setCollisionVisibility(visible) {
  collisionVisible = visible;
  applyCollisionVisibility();
  setButtonActive("toggleCollision", visible);
}

function sceneMaterials() {
  const visited = new Set();
  const materials = [];
  for (const mesh of sceneMeshes) {
    const material = mesh.material;
    if (!material || visited.has(material.uniqueId)) continue;
    visited.add(material.uniqueId);
    materials.push(material);
  }
  return materials;
}

function ensureMaterialDefaults(material) {
  material.metadata = material.metadata || {};
  if (material.metadata.dimosDefaults) return material.metadata.dimosDefaults;
  material.metadata.dimosDefaults = {
    alpha: material.alpha,
    backFaceCulling: material.backFaceCulling,
    disableDepthWrite: material.disableDepthWrite,
    forceDepthWrite: material.forceDepthWrite,
    transparencyMode: material.transparencyMode,
    needDepthPrePass: material.needDepthPrePass,
    metallic: material.metallic,
    roughness: material.roughness,
    environmentIntensity: material.environmentIntensity,
  };
  return material.metadata.dimosDefaults;
}

function editMaterial(material, apply) {
  if (material.unfreeze) material.unfreeze();
  apply(material, ensureMaterialDefaults(material));
  if (material.freeze) material.freeze();
}

function configureStaticSceneMaterial(material) {
  editMaterial(material, (mat) => {
    mat.disableDepthWrite = false;
    mat.forceDepthWrite = true;
    mat.needDepthPrePass = false;
  });
}

function setClickMode(mode) {
  clickMode = clickMode === mode ? null : mode;
  setButtonActive("navClick", clickMode === "nav");
  setButtonActive("pointClick", clickMode === "point");
  setButtonActive("spawnClick", clickMode === "spawn");
  if (clickMode === "nav") setStatus("click nav target");
  if (clickMode === "point") setStatus("click point target");
  if (clickMode === "spawn") setStatus("click spawn point");
  if (clickMode === null) setStatus("live");
}

function markerMaterial(name, color) {
  const material = new BABYLON.StandardMaterial(name, scene);
  material.diffuseColor = color;
  material.emissiveColor = color.scale(0.75);
  material.specularColor = BABYLON.Color3.Black();
  material.disableLighting = true;
  return material;
}

const navMarkerMaterial = markerMaterial(
  "navGoalMaterial",
  new BABYLON.Color3(0.06, 0.82, 1.0),
);
const pointMarkerMaterial = markerMaterial(
  "pointGoalMaterial",
  new BABYLON.Color3(1.0, 0.18, 0.78),
);
const spawnMarkerMaterial = markerMaterial(
  "spawnMarkerMaterial",
  new BABYLON.Color3(0.14, 1.0, 0.42),
);

function placeMarker(existingMarker, name, position, material, diameter) {
  if (existingMarker) existingMarker.dispose();
  const marker = BABYLON.MeshBuilder.CreateSphere(
    name,
    { diameter, segments: 16 },
    scene,
  );
  marker.position = position;
  marker.material = material;
  marker.isPickable = false;
  return marker;
}

function updateKeyboardCamera() {
  if (driveEnabled) return;
  const deltaSeconds = Math.min(engine.getDeltaTime() / 1000, 0.05);
  const speed = (pressedKeys.has("shift") ? 8.0 : 2.7) * deltaSeconds;
  const up = new BABYLON.Vector3(0, 0, 1);
  const forward = camera.getForwardRay().direction;
  forward.z = 0;
  if (forward.lengthSquared() < 1e-8) return;
  forward.normalize();
  const right = BABYLON.Vector3.Cross(forward, up).normalize();
  const move = BABYLON.Vector3.Zero();

  if (pressedKeys.has("w")) move.addInPlace(forward);
  if (pressedKeys.has("s")) move.subtractInPlace(forward);
  if (pressedKeys.has("d")) move.addInPlace(right);
  if (pressedKeys.has("a")) move.subtractInPlace(right);
  if (pressedKeys.has("e")) move.addInPlace(up);
  if (pressedKeys.has("q")) move.subtractInPlace(up);

  if (move.lengthSquared() === 0) return;
  move.normalize().scaleInPlace(speed);
  camera.target.addInPlace(move);
}

function sendSocketPayload(payload) {
  if (!streamRef.worker || !streamRef.ready) return false;
  streamRef.worker.postMessage({ type: "send_json", payload });
  return true;
}

function currentDriveTwist() {
  const speedScale = pressedKeys.has("shift") ? 1.8 : 1.0;
  let linearX = 0.0;
  let linearY = 0.0;
  let angularZ = 0.0;

  if (pressedKeys.has("w")) linearX += driveLinearSpeed * speedScale;
  if (pressedKeys.has("s")) linearX -= driveLinearSpeed * speedScale;
  if (pressedKeys.has("q")) linearY += driveStrafeSpeed * speedScale;
  if (pressedKeys.has("e")) linearY -= driveStrafeSpeed * speedScale;
  if (pressedKeys.has("a")) angularZ += driveAngularSpeed * speedScale;
  if (pressedKeys.has("d")) angularZ -= driveAngularSpeed * speedScale;

  return {
    linear: [linearX, linearY, 0.0],
    angular: [0.0, 0.0, angularZ],
  };
}

function sendDriveCommand(force = false) {
  if (!driveEnabled && !force) return;
  const now = performance.now() / 1000;
  if (!force && now - lastDriveSendTime < driveSendPeriod) return;

  const twist = force
    ? { linear: [0.0, 0.0, 0.0], angular: [0.0, 0.0, 0.0] }
    : currentDriveTwist();
  const signature = JSON.stringify(twist);
  const isZero =
    twist.linear.every((value) => Math.abs(value) < 1e-6) &&
    twist.angular.every((value) => Math.abs(value) < 1e-6);
  if (!force && isZero && signature === lastDriveSignature) return;

  const M = window.dimosMsgs;
  const lcm = window.dimosLcm;
  if (M && lcm) {
    try {
      const t = new M.geometry_msgs.Twist({
        linear: new M.geometry_msgs.Vector3({
          x: twist.linear[0], y: twist.linear[1], z: twist.linear[2],
        }),
        angular: new M.geometry_msgs.Vector3({
          x: twist.angular[0], y: twist.angular[1], z: twist.angular[2],
        }),
      });
      lcm.publish("/cmd_vel", t);
    } catch (err) {
      console.warn("[lcm] cmd_vel publish failed", err);
      return;
    }
  }
  if (browserPhysicsEnabled) setBrowserPhysicsCommand(twist);
  lastDriveSendTime = now;
  lastDriveSignature = signature;
}

function setDriveEnabled(enabled) {
  driveEnabled = enabled;
  setButtonActive("toggleDrive", enabled);
  setStatus(enabled ? "drive: WASD turn/move, QE strafe" : "live");
  if (!enabled) sendDriveCommand(true);
}

function setBrowserPhysicsCommand(twist) {
  if (!twist) return;
  const linear = Array.isArray(twist.linear) ? twist.linear : [0, 0, 0];
  const angular = Array.isArray(twist.angular) ? twist.angular : [0, 0, 0];
  browserPhysicsCommand = {
    linear: [Number(linear[0]) || 0, Number(linear[1]) || 0, Number(linear[2]) || 0],
    angular: [Number(angular[0]) || 0, Number(angular[1]) || 0, Number(angular[2]) || 0],
  };
}

function setBrowserPhysicsPose(pose, publish = true) {
  if (!pose) return;
  browserPhysicsPose = {
    x: Number(pose.x) || 0,
    y: Number(pose.y) || 0,
    z: Number(pose.z) || 0,
    yaw: Number(pose.yaw) || 0,
  };
  browserPhysicsLastStepMs = performance.now();
  if (publish) publishBrowserSimOdom(true);
}

function yawQuaternion(yaw) {
  return BABYLON.Quaternion.FromEulerAngles(0, 0, yaw);
}

function capsuleQuaternion(yaw) {
  return yawQuaternion(yaw).multiply(BABYLON.Quaternion.FromEulerAngles(Math.PI / 2, 0, 0));
}

function publishBrowserSimOdom(force = false) {
  if (!browserPhysicsEnabled) return;
  const now = performance.now();
  if (!force && now - browserPhysicsLastOdomMs < browserPhysicsOdomPeriodMs) return;
  browserPhysicsLastOdomMs = now;
  const q = yawQuaternion(browserPhysicsPose.yaw);
  const M = window.dimosMsgs;
  const lcm = window.dimosLcm;
  if (!M || !lcm) return;
  try {
    const ts = Date.now() / 1000;
    const sec = Math.floor(ts);
    const nanosec = Math.floor((ts - sec) * 1e9);
    const pose = new M.geometry_msgs.PoseStamped({
      header: new M.std_msgs.Header({
        stamp: new M.builtin_interfaces.Time({ sec, nanosec }),
        frame_id: "map",
      }),
      pose: new M.geometry_msgs.Pose({
        position: new M.geometry_msgs.Point({
          x: browserPhysicsPose.x,
          y: browserPhysicsPose.y,
          z: browserPhysicsPose.z,
        }),
        orientation: new M.geometry_msgs.Quaternion({
          x: q.x, y: q.y, z: q.z, w: q.w,
        }),
      }),
    });
    lcm.publish("/odom", pose);
  } catch (err) {
    console.warn("[lcm] sim_odom publish failed", err);
  }
}

function horizontalCollisionDistance(origin, direction, maxDistance) {
  if (collisionMeshes.length === 0 || maxDistance <= 1e-6) return maxDistance;
  let allowed = maxDistance;
  const groundZ = origin.z - browserVehicleHeight;
  for (const height of browserCollisionProbeHeights) {
    const rayOrigin = new BABYLON.Vector3(origin.x, origin.y, groundZ + height);
    const ray = new BABYLON.Ray(rayOrigin, direction, maxDistance + browserCharacterRadius);
    const pick = scene.pickWithRay(
      ray,
      (mesh) => collisionMeshSet.has(mesh),
    );
    if (pick && pick.hit && Number.isFinite(pick.distance)) {
      allowed = Math.min(allowed, Math.max(0, pick.distance - browserCharacterRadius));
    }
  }
  return allowed;
}

function groundHeightAt(x, y, referenceBaseZ) {
  if (collisionMeshes.length === 0) return null;
  const origin = new BABYLON.Vector3(
    x,
    y,
    referenceBaseZ + browserStepOffset + browserGroundProbeExtra,
  );
  const length =
    browserVehicleHeight + browserStepOffset + browserMaxStepDown + browserGroundProbeExtra;
  const ray = new BABYLON.Ray(origin, new BABYLON.Vector3(0, 0, -1), length);
  const pick = scene.pickWithRay(ray, (mesh) => collisionMeshSet.has(mesh));
  if (pick && pick.hit && pick.pickedPoint) return pick.pickedPoint.z;
  return null;
}

function snapCandidateToGround(candidate, currentBaseZ) {
  const currentGroundZ = currentBaseZ - browserVehicleHeight;
  const groundZ = groundHeightAt(candidate.x, candidate.y, currentBaseZ);
  if (groundZ === null) return candidate;
  const stepDelta = groundZ - currentGroundZ;
  if (stepDelta > browserStepOffset || stepDelta < -browserMaxStepDown) return null;
  return new BABYLON.Vector3(candidate.x, candidate.y, groundZ + browserVehicleHeight);
}

function resolvePlanarMove(position, delta) {
  const distance = Math.hypot(delta.x, delta.y);
  if (distance <= 1e-8 || collisionMeshes.length === 0) {
    return new BABYLON.Vector3(position.x + delta.x, position.y + delta.y, position.z);
  }
  const direction = new BABYLON.Vector3(delta.x / distance, delta.y / distance, 0);
  const allowed = horizontalCollisionDistance(position, direction, distance);
  if (allowed >= distance - 1e-5) {
    const snapped = snapCandidateToGround(
      new BABYLON.Vector3(position.x + delta.x, position.y + delta.y, position.z),
      position.z,
    );
    return snapped || position.clone();
  }

  // Cheap slide: try each axis independently when the diagonal path is blocked.
  const candidates = [
    new BABYLON.Vector3(delta.x, 0, 0),
    new BABYLON.Vector3(0, delta.y, 0),
  ];
  let best = BABYLON.Vector3.Zero();
  for (const candidate of candidates) {
    const d = Math.hypot(candidate.x, candidate.y);
    if (d <= 1e-8) continue;
    const dir = new BABYLON.Vector3(candidate.x / d, candidate.y / d, 0);
    const a = horizontalCollisionDistance(position, dir, d);
    const moved = dir.scale(a);
    const snapped = snapCandidateToGround(
      new BABYLON.Vector3(position.x + moved.x, position.y + moved.y, position.z),
      position.z,
    );
    if (snapped && a > best.length()) {
      best = new BABYLON.Vector3(snapped.x - position.x, snapped.y - position.y, snapped.z - position.z);
    }
  }
  return new BABYLON.Vector3(position.x + best.x, position.y + best.y, position.z + best.z);
}

function snapBrowserPhysicsToGround(publish = false) {
  if (!browserPhysicsEnabled || collisionMeshes.length === 0) return;
  const groundZ = groundHeightAt(
    browserPhysicsPose.x,
    browserPhysicsPose.y,
    browserPhysicsPose.z,
  );
  if (groundZ === null) return;
  browserPhysicsPose = {
    ...browserPhysicsPose,
    z: groundZ + browserVehicleHeight,
  };
  if (publish) publishBrowserSimOdom(true);
}

function stepBrowserPhysics() {
  if (!browserPhysicsEnabled) return;
  const now = performance.now();
  const previous = browserPhysicsLastStepMs ?? now;
  browserPhysicsLastStepMs = now;
  const dt = Math.min(Math.max((now - previous) / 1000, 0), 0.05);
  if (dt <= 0) return;

  const linear = browserPhysicsCommand.linear;
  const angular = browserPhysicsCommand.angular;
  const yaw = browserPhysicsPose.yaw + (angular[2] || 0) * dt;
  const c = Math.cos(yaw);
  const s = Math.sin(yaw);
  const vx = c * (linear[0] || 0) - s * (linear[1] || 0);
  const vy = s * (linear[0] || 0) + c * (linear[1] || 0);
  const current = new BABYLON.Vector3(
    browserPhysicsPose.x,
    browserPhysicsPose.y,
    browserPhysicsPose.z,
  );
  const next = resolvePlanarMove(current, new BABYLON.Vector3(vx * dt, vy * dt, 0));
  browserPhysicsPose = { x: next.x, y: next.y, z: next.z, yaw };
  if (robotColliderMesh) {
    robotColliderMesh.position.set(next.x, next.y, next.z);
    robotColliderMesh.rotationQuaternion = capsuleQuaternion(yaw);
  }
  publishBrowserSimOdom(false);
}

function configureBrowserPhysics(config) {
  browserPhysicsEnabled = Boolean(config.browserPhysics);
  if (!browserPhysicsEnabled) return;
  browserVehicleHeight = Number(config.vehicleHeight) || browserVehicleHeight;
  browserStepOffset = Number(config.stepOffset) || browserStepOffset;
  supportFloorEnabled = Boolean(config.supportFloor);
  supportFloorZ = finiteNumber(config.supportFloorZ, supportFloorZ);
  supportFloorSize = Math.max(0, finiteNumber(config.supportFloorSize, supportFloorSize));
  setBrowserPhysicsPose(config.browserPhysicsInitialPose || {}, false);
  setStatus("browser physics ready");
}

function setSceneDepthWrite(enabled) {
  sceneDepthEnabled = enabled;
  for (const material of sceneMaterials()) {
    editMaterial(material, (mat) => {
      mat.disableDepthWrite = !enabled;
      mat.needDepthPrePass = false;
    });
  }
  setButtonActive("toggleDepth", enabled);
}

function setSceneWireframe(enabled) {
  sceneWireEnabled = enabled;
  for (const material of sceneMaterials()) {
    editMaterial(material, (mat) => {
      mat.wireframe = enabled;
    });
  }
  setButtonActive("toggleWire", enabled);
}

function setForceVisible(enabled) {
  forceVisibleEnabled = enabled;
  for (const material of sceneMaterials()) {
    editMaterial(material, (mat, defaults) => {
      if (!enabled) {
        mat.alpha = defaults.alpha;
        mat.backFaceCulling = defaults.backFaceCulling;
        mat.disableDepthWrite = false;
        mat.forceDepthWrite = true;
        mat.transparencyMode = defaults.transparencyMode;
        mat.needDepthPrePass = false;
        return;
      }
      mat.backFaceCulling = false;
      mat.alpha = 1;
      mat.disableDepthWrite = false;
      mat.forceDepthWrite = true;
      mat.needDepthPrePass = false;
      mat.transparencyMode = BABYLON.Material.MATERIAL_OPAQUE;
      if (mat.albedoColor) {
        mat.metallic = 0;
        mat.roughness = 0.9;
        mat.environmentIntensity = 1;
      }
      if (mat.diffuseColor) {
        mat.diffuseColor = mat.diffuseColor || new BABYLON.Color3(0.8, 0.8, 0.8);
      }
    });
  }
  setButtonActive("forceVisible", enabled);
}

function computeMeshBounds(meshes) {
  const min = new BABYLON.Vector3(Number.POSITIVE_INFINITY, Number.POSITIVE_INFINITY, Number.POSITIVE_INFINITY);
  const max = new BABYLON.Vector3(Number.NEGATIVE_INFINITY, Number.NEGATIVE_INFINITY, Number.NEGATIVE_INFINITY);
  let count = 0;
  for (const mesh of meshes) {
    if (!mesh.getTotalVertices || mesh.getTotalVertices() === 0) continue;
    mesh.computeWorldMatrix(true);
    mesh.refreshBoundingInfo(true);
    const box = mesh.getBoundingInfo().boundingBox;
    min.x = Math.min(min.x, box.minimumWorld.x);
    min.y = Math.min(min.y, box.minimumWorld.y);
    min.z = Math.min(min.z, box.minimumWorld.z);
    max.x = Math.max(max.x, box.maximumWorld.x);
    max.y = Math.max(max.y, box.maximumWorld.y);
    max.z = Math.max(max.z, box.maximumWorld.z);
    count += 1;
  }
  if (count === 0) return null;
  const center = min.add(max).scale(0.5);
  const extent = max.subtract(min);
  return { min, max, center, extent, count };
}

function fitCameraToMeshes(meshes) {
  const bounds = computeMeshBounds(meshes);
  if (!bounds) return;
  const { center, extent, count } = bounds;
  camera.setTarget(center);
  camera.radius = Math.max(2, extent.length() * 0.55);
  setStatus(`scene ${count} meshes`);
}

function disposeSupportFloor() {
  if (supportFloorAggregate) {
    try {
      supportFloorAggregate.dispose();
    } catch (_) {
      // best-effort cleanup
    }
    supportFloorAggregate = null;
  }
  if (supportFloorMesh) {
    unregisterCollisionMesh(supportFloorMesh);
    try {
      supportFloorMesh.dispose();
    } catch (_) {
      // best-effort cleanup
    }
    supportFloorMesh = null;
  }
}

async function ensureSupportFloor(config) {
  supportFloorEnabled = Boolean(config.supportFloor);
  supportFloorZ = finiteNumber(config.supportFloorZ, supportFloorZ);
  supportFloorSize = Math.max(0, finiteNumber(config.supportFloorSize, supportFloorSize));
  if (!browserPhysicsEnabled || !supportFloorEnabled) return;
  if (!(await physicsReady)) return;

  disposeSupportFloor();
  const bounds = computeMeshBounds(collisionMeshes.length > 0 ? collisionMeshes : sceneMeshes);
  const center = bounds
    ? new BABYLON.Vector3(bounds.center.x, bounds.center.y, supportFloorZ - supportFloorThickness * 0.5)
    : new BABYLON.Vector3(browserPhysicsPose.x, browserPhysicsPose.y, supportFloorZ - supportFloorThickness * 0.5);
  const derivedSize = bounds
    ? Math.max(bounds.extent.x, bounds.extent.y) + 8.0
    : 20.0;
  const size = Math.max(20.0, supportFloorSize > 0 ? supportFloorSize : derivedSize);

  supportFloorMesh = BABYLON.MeshBuilder.CreateBox(
    "supportFloor",
    { width: size, height: size, depth: supportFloorThickness },
    scene,
  );
  supportFloorMesh.position = center;
  supportFloorMesh.material = createCollisionMaterial();
  supportFloorMesh.visibility = sceneVisible && collisionVisible ? 1 : 0;
  supportFloorMesh.isPickable = true;
  supportFloorMesh.metadata = { dimosCollisionMesh: true, dimosSupportFloor: true };
  registerCollisionMesh(supportFloorMesh);
  supportFloorAggregate = new BABYLON.PhysicsAggregate(
    supportFloorMesh,
    BABYLON.PhysicsShapeType.BOX,
    { mass: 0 },
    scene,
  );
}

function focusRobot() {
  if (!latestRootPosition) return;
  camera.setTarget(latestRootPosition);
  camera.radius = Math.max(4, camera.radius);
}

async function loadConfig() {
  const response = await fetch("/config.json", { cache: "no-store" });
  return await response.json();
}

async function loadSceneAsset(config) {
  if (sceneLoadStarted) return;
  sceneLoadStarted = true;
  if (!config.sceneFile) return;
  if (config.sceneBytes > maxAutoSceneBytes) {
    setStatus("scene exceeds browser load guard");
    return;
  }
  setStatus("loading scene");
  const root = new BABYLON.TransformNode("sceneRoot", scene);
  root.position = vec3(config.scenePosition);
  root.scaling = new BABYLON.Vector3(config.sceneScale, config.sceneScale, config.sceneScale);
  root.rotationQuaternion = quatWxyz(config.sceneWxyz);

  engine.stopRenderLoop(renderFrame);
  let result = null;
  try {
    result = await BABYLON.SceneLoader.ImportMeshAsync(null, "/assets/", config.sceneFile, scene);
  } finally {
    engine.runRenderLoop(renderFrame);
  }

  for (const light of result.lights || []) {
    light.dispose();
  }
  for (const camera of result.cameras || []) {
    camera.dispose();
  }
  for (const mesh of result.meshes) {
    if (mesh.parent === null) mesh.parent = root;
    mesh.isPickable = true;
    mesh.metadata = { dimosSceneMesh: true };
    if (mesh.getTotalVertices && mesh.getTotalVertices() > 0) registerSceneMesh(mesh);
    if (mesh.material) configureStaticSceneMaterial(mesh.material);
    if (mesh.getTotalVertices && mesh.getTotalVertices() > 0) {
      mesh.computeWorldMatrix(true);
      mesh.refreshBoundingInfo(true);
      mesh.freezeWorldMatrix();
      mesh.doNotSyncBoundingInfo = true;
    }
  }
  root.freezeWorldMatrix();
  setSceneDepthWrite(sceneDepthEnabled);
  setSceneWireframe(sceneWireEnabled);
  setForceVisible(forceVisibleEnabled);
  fitCameraToMeshes(sceneMeshes);
  await ensureSupportFloor(config);
}

async function loadCollisionAsset(config) {
  if (!config.collisionSceneFile) return;
  if (config.collisionSceneFile === config.sceneFile) return;
  if (config.collisionSceneBytes > maxAutoSceneBytes) {
    setStatus("collision exceeds browser load guard");
    return;
  }
  setStatus("loading collision");
  const root = new BABYLON.TransformNode("collisionRoot", scene);
  root.position = vec3(config.scenePosition);
  root.scaling = new BABYLON.Vector3(config.sceneScale, config.sceneScale, config.sceneScale);
  root.rotationQuaternion = quatWxyz(config.sceneWxyz);

  const result = await BABYLON.SceneLoader.ImportMeshAsync(
    null,
    "/assets/",
    config.collisionSceneFile,
    scene,
  );
  for (const light of result.lights || []) light.dispose();
  for (const camera of result.cameras || []) camera.dispose();
  for (const mesh of result.meshes) {
    if (mesh.parent === null) mesh.parent = root;
    if (!mesh.getTotalVertices || mesh.getTotalVertices() === 0) continue;
    mesh.isPickable = true;
    mesh.material = createCollisionMaterial();
    mesh.visibility = sceneVisible && collisionVisible ? 1 : 0;
    // Don't setEnabled(false) — Havok needs the mesh active to collide.
    // setCollisionVisibility() toggles only the visibility for display.
    mesh.metadata = { dimosCollisionMesh: true };
    registerCollisionMesh(mesh);
  }
  if (sceneMeshes.length === 0) fitCameraToMeshes(collisionMeshes);

  // Wrap each collision mesh with a static (mass=0) PhysicsAggregate so
  // entities + the character controller actually collide with the scene.
  if (await physicsReady) {
    for (const mesh of collisionMeshes) {
      if (mesh.physicsBody) continue; // idempotent
      try {
        new BABYLON.PhysicsAggregate(mesh, BABYLON.PhysicsShapeType.MESH, { mass: 0 }, scene);
      } catch (error) {
        console.warn(`collision physics aggregate failed for ${mesh.name}:`, error);
      }
    }
  }

  await ensureSupportFloor(config);
  snapBrowserPhysicsToGround(true);
  setStatus("live");
}

function pickScenePoint() {
  const ray = scene.createPickingRay(
    scene.pointerX,
    scene.pointerY,
    BABYLON.Matrix.Identity(),
    camera,
  );
  const useCollision = collisionMeshes.length > 0;
  const pick = scene.pickWithRay(
    ray,
    (mesh) => (useCollision ? collisionMeshSet.has(mesh) : sceneMeshSet.has(mesh)),
  );
  if (pick && pick.hit && pick.pickedPoint) return pick.pickedPoint;
  if (Math.abs(ray.direction.z) < 1e-6) return null;
  const distance = -ray.origin.z / ray.direction.z;
  if (distance <= 0) return null;
  return ray.origin.add(ray.direction.scale(distance));
}

async function loadRobot() {
  setStatus("loading robot");
  const response = await fetch("/robot.json", { cache: "no-store" });
  const payload = await response.json();
  bodyNodeList.length = 0;
  bodyNameList.length = 0;
  lastAppliedRobotPoses.length = 0;
  latestRobotPosePayload = null;
  robotRootBodyIndex = -1;
  for (const bodyName of payload.bodyNames) {
    bodyNameList.push(bodyName);
    bodyNodeList.push(ensureBodyNode(bodyName));
    if (robotRootBodyIndex < 0 && robotRootBodyNames.has(bodyName)) {
      robotRootBodyIndex = bodyNodeList.length - 1;
    }
  }
  if (!useRobotMesh) return;

  for (const geom of payload.geoms) {
    const mesh = new BABYLON.Mesh(`robot:${geom.id}`, scene);
    const normals = [];
    BABYLON.VertexData.ComputeNormals(geom.vertices, geom.indices, normals);
    const vertexData = new BABYLON.VertexData();
    vertexData.positions = geom.vertices;
    vertexData.indices = geom.indices;
    vertexData.normals = normals;
    vertexData.applyToMesh(mesh);

    const material = new BABYLON.StandardMaterial(`robotMat:${geom.id}`, scene);
    material.diffuseColor = new BABYLON.Color3(geom.rgba[0], geom.rgba[1], geom.rgba[2]);
    material.specularColor = new BABYLON.Color3(0.18, 0.18, 0.18);
    material.alpha = geom.rgba[3] > 0 ? geom.rgba[3] : 1;
    material.backFaceCulling = false;
    mesh.material = material;

    mesh.parent = bodyNodes.get(geom.body);
    mesh.position = vec3(geom.position);
    mesh.rotationQuaternion = quatWxyz(geom.wxyz);
    mesh.isPickable = false;
    robotMeshes.push(mesh);
  }
}

async function setupRobotCollider() {
  // Find the root body node (pelvis/torso/body_1) and wrap it with a
  // kinematic capsule so Havok-driven entities collide with the robot
  // as it's driven through the world by odom.
  if (!(await physicsReady)) return;
  if (browserPhysicsEnabled) {
    robotColliderMesh = BABYLON.MeshBuilder.CreateCapsule(
      "robotCollider",
      {
        height: ROBOT_COLLIDER_HEIGHT,
        radius: ROBOT_COLLIDER_RADIUS,
        tessellation: 12,
        subdivisions: 1,
      },
      scene,
    );
    robotColliderMesh.position = new BABYLON.Vector3(
      browserPhysicsPose.x,
      browserPhysicsPose.y,
      browserPhysicsPose.z,
    );
    robotColliderMesh.rotationQuaternion = capsuleQuaternion(browserPhysicsPose.yaw);
    robotColliderMesh.isVisible = false;
    robotColliderMesh.isPickable = false;
    try {
      robotColliderAggregate = new BABYLON.PhysicsAggregate(
        robotColliderMesh,
        BABYLON.PhysicsShapeType.CAPSULE,
        { mass: 0 },
        scene,
      );
      if (
        robotColliderAggregate.body
        && "disablePreStep" in robotColliderAggregate.body
      ) {
        robotColliderAggregate.body.disablePreStep = false;
      }
      console.log("browser physics robot collider initialized");
    } catch (error) {
      console.warn("browser robot collider aggregate failed:", error);
      robotColliderMesh.dispose();
      robotColliderMesh = null;
      robotColliderAggregate = null;
    }
    return;
  }
  let rootName = null;
  for (const name of bodyNameList) {
    if (robotRootBodyNames.has(name)) {
      rootName = name;
      break;
    }
  }
  if (rootName === null) {
    console.warn("robot collider: no root body found (looked for pelvis/torso/body_1)");
    return;
  }
  const rootNode = bodyNodes.get(rootName);
  if (!rootNode) return;

  // CreateCapsule is Y-axis by default; rotate to align with our Z-up scene.
  robotColliderMesh = BABYLON.MeshBuilder.CreateCapsule(
    "robotCollider",
    {
      height: ROBOT_COLLIDER_HEIGHT,
      radius: ROBOT_COLLIDER_RADIUS,
      tessellation: 12,
      subdivisions: 1,
    },
    scene,
  );
  robotColliderMesh.rotation = new BABYLON.Vector3(Math.PI / 2, 0, 0);
  robotColliderMesh.position = new BABYLON.Vector3(0, 0, ROBOT_COLLIDER_Z_OFFSET);
  robotColliderMesh.isVisible = false;
  robotColliderMesh.isPickable = false;
  robotColliderMesh.parent = rootNode;

  try {
    robotColliderAggregate = new BABYLON.PhysicsAggregate(
      robotColliderMesh,
      BABYLON.PhysicsShapeType.CAPSULE,
      { mass: 0 },
      scene,
    );
    // Kinematic bodies: prestep enabled so the parent-driven worldMatrix
    // is read into the physics body each tick. Entities then collide
    // against the capsule's current pose.
    if (
      robotColliderAggregate.body
      && "disablePreStep" in robotColliderAggregate.body
    ) {
      robotColliderAggregate.body.disablePreStep = false;
    }
    console.log(`robot collider attached to body "${rootName}"`);
  } catch (error) {
    console.warn("robot collider aggregate failed:", error);
    robotColliderMesh.dispose();
    robotColliderMesh = null;
    robotColliderAggregate = null;
  }
}

function proxyDiameter(bodyName) {
  if (bodyName === "world") return 0;
  if (bodyName.includes("pelvis") || bodyName.includes("torso")) return 0.22;
  if (bodyName.includes("hip") || bodyName.includes("shoulder")) return 0.14;
  if (bodyName.includes("knee") || bodyName.includes("elbow")) return 0.11;
  if (bodyName.includes("ankle") || bodyName.includes("wrist")) return 0.09;
  return 0.075;
}

function ensureBodyNode(bodyName) {
  let node = bodyNodes.get(bodyName);
  if (node) return node;
  node = new BABYLON.TransformNode(`body:${bodyName}`, scene);
  node.rotationQuaternion = BABYLON.Quaternion.Identity();
  bodyNodes.set(bodyName, node);
  if (!useRobotMesh) {
    const diameter = proxyDiameter(bodyName);
    if (diameter > 0) {
      if (!proxyMaterial) {
        proxyMaterial = new BABYLON.StandardMaterial("robotProxyMat", scene);
        proxyMaterial.diffuseColor = new BABYLON.Color3(0.95, 0.62, 0.24);
        proxyMaterial.specularColor = new BABYLON.Color3(0.22, 0.22, 0.22);
      }
      const marker = BABYLON.MeshBuilder.CreateSphere(
        `robotProxy:${bodyName}`,
        { diameter, segments: 8 },
        scene,
      );
      marker.material = proxyMaterial;
      marker.parent = node;
      marker.isPickable = false;
      robotMeshes.push(marker);
    }
  }
  return node;
}

function poseChanged(poses, offset, previous) {
  if (!previous) return true;
  return (
    Math.abs(poses[offset] - previous[0]) > posePositionEpsilon ||
    Math.abs(poses[offset + 1] - previous[1]) > posePositionEpsilon ||
    Math.abs(poses[offset + 2] - previous[2]) > posePositionEpsilon ||
    Math.abs(poses[offset + 3] - previous[3]) > poseQuaternionEpsilon ||
    Math.abs(poses[offset + 4] - previous[4]) > poseQuaternionEpsilon ||
    Math.abs(poses[offset + 5] - previous[5]) > poseQuaternionEpsilon ||
    Math.abs(poses[offset + 6] - previous[6]) > poseQuaternionEpsilon
  );
}

function rememberAppliedPose(bodyIndex, poses, offset) {
  let previous = lastAppliedRobotPoses[bodyIndex];
  if (!previous) {
    previous = new Float32Array(7);
    lastAppliedRobotPoses[bodyIndex] = previous;
  }
  previous[0] = poses[offset];
  previous[1] = poses[offset + 1];
  previous[2] = poses[offset + 2];
  previous[3] = poses[offset + 3];
  previous[4] = poses[offset + 4];
  previous[5] = poses[offset + 5];
  previous[6] = poses[offset + 6];
}

function packedPoseToMatrixToRef(poses, offset, result) {
  robotPosePositionScratch.copyFromFloats(
    poses[offset],
    poses[offset + 1],
    poses[offset + 2],
  );
  robotPoseQuaternionScratch.copyFromFloats(
    poses[offset + 4],
    poses[offset + 5],
    poses[offset + 6],
    poses[offset + 3],
  );
  BABYLON.Matrix.ComposeToRef(
    robotPoseComposeScale,
    robotPoseQuaternionScratch,
    robotPosePositionScratch,
    result,
  );
}

function copyPackedPoseToNode(node, poses, offset) {
  node.position.copyFromFloats(poses[offset], poses[offset + 1], poses[offset + 2]);
  if (!node.rotationQuaternion) {
    node.rotationQuaternion = BABYLON.Quaternion.Identity();
  }
  node.rotationQuaternion.copyFromFloats(
    poses[offset + 4],
    poses[offset + 5],
    poses[offset + 6],
    poses[offset + 3],
  );
}

function copyMatrixPoseToNode(node, matrix) {
  if (!node.rotationQuaternion) {
    node.rotationQuaternion = BABYLON.Quaternion.Identity();
  }
  matrix.decompose(robotPoseDecomposeScale, node.rotationQuaternion, node.position);
}

function browserPhysicsRootMatrixToRef(result) {
  robotPosePositionScratch.copyFromFloats(
    browserPhysicsPose.x,
    browserPhysicsPose.y,
    browserPhysicsPose.z,
  );
  const q = yawQuaternion(browserPhysicsPose.yaw);
  robotPoseQuaternionScratch.copyFromFloats(q.x, q.y, q.z, q.w);
  BABYLON.Matrix.ComposeToRef(
    robotPoseComposeScale,
    robotPoseQuaternionScratch,
    robotPosePositionScratch,
    result,
  );
}

function updateLatestRootPosition(position) {
  if (!latestRootPosition) latestRootPosition = BABYLON.Vector3.Zero();
  latestRootPosition.copyFrom(position);
}

function updatePath(path, pathVersion) {
  const hasVersion = Number.isFinite(pathVersion);
  if (hasVersion && pathVersion === lastPathVersion) return;
  if (hasVersion) lastPathVersion = pathVersion;

  if (pathMesh) {
    pathMesh.dispose();
    pathMesh = null;
  }
  if (!path || path.length <= 1) return;

  pathMesh = BABYLON.MeshBuilder.CreateLines(
    "navPath",
    { points: path.map((point) => vec3([point[0], point[1], point[2] + 0.08])) },
    scene,
  );
  pathMesh.color = new BABYLON.Color3(0.15, 0.95, 0.68);
  pathMesh.isPickable = false;
}

function applyRobotPose(payload) {
  const count = Math.min(payload.count, bodyNodeList.length);
  const rebaseToBrowserPhysics =
    browserPhysicsEnabled &&
    robotRootBodyIndex >= 0 &&
    robotRootBodyIndex < count;

  if (rebaseToBrowserPhysics) {
    const rootOffset = robotRootBodyIndex * 7;
    packedPoseToMatrixToRef(payload.poses, rootOffset, robotPoseRootMatrixScratch);
    robotPoseRootMatrixScratch.invertToRef(robotPoseRootInverseMatrixScratch);
    browserPhysicsRootMatrixToRef(robotPoseBrowserRootMatrixScratch);
    robotPoseRootInverseMatrixScratch.multiplyToRef(
      robotPoseBrowserRootMatrixScratch,
      robotPoseRebaseMatrixScratch,
    );
  }

  let updated = 0;
  let skipped = 0;
  for (let bodyIndex = 0; bodyIndex < count; bodyIndex += 1) {
    const node = bodyNodeList[bodyIndex];
    const bodyName = bodyNameList[bodyIndex];
    const offset = bodyIndex * 7;

    if (rebaseToBrowserPhysics) {
      packedPoseToMatrixToRef(payload.poses, offset, robotPoseMatrixScratch);
      robotPoseMatrixScratch.multiplyToRef(
        robotPoseRebaseMatrixScratch,
        robotPoseRebasedMatrixScratch,
      );
      copyMatrixPoseToNode(node, robotPoseRebasedMatrixScratch);
      rememberAppliedPose(bodyIndex, payload.poses, offset);
      updated += 1;
    } else if (poseChanged(payload.poses, offset, lastAppliedRobotPoses[bodyIndex])) {
      copyPackedPoseToNode(node, payload.poses, offset);
      rememberAppliedPose(bodyIndex, payload.poses, offset);
      updated += 1;
    } else {
      skipped += 1;
    }
    if (robotRootBodyNames.has(bodyName)) {
      updateLatestRootPosition(node.position);
    }
  }

  if (!latestRootPosition && count > 1) {
    updateLatestRootPosition(bodyNodeList[1].position);
  }
  perfCounters.poseFrames += 1;
  perfCounters.poseBodiesUpdated += updated;
  perfCounters.poseBodiesSkipped += skipped;
}

function applyLatestRobotPose() {
  if (!browserPhysicsEnabled || !latestRobotPosePayload) return;
  applyRobotPose(latestRobotPosePayload);
}

function queueRobotPose(payload) {
  const nowMs = performance.now();
  const sourceMs = Number.isFinite(payload.time) ? payload.time * 1000 : nowMs;
  stateTiming.jsMinusSourceMs = Math.min(
    nowMs - sourceMs,
    stateTiming.jsMinusSourceMs,
  );

  const previousSourceMs = stateTiming.previousSourceMs ?? sourceMs;
  const sourceDeltaMs = sourceMs - previousSourceMs;
  stateTiming.previousSourceMs = sourceMs;

  let targetMs = nowMs;
  const sourceLagMs = nowMs - stateTiming.jsMinusSourceMs - sourceMs;
  if (
    stateTiming.lastIdealJsMs === null ||
    sourceDeltaMs <= 0 ||
    sourceDeltaMs > stateImmediateDeltaMs ||
    sourceLagMs > stateMaxLagMs
  ) {
    stateTiming.lastIdealJsMs = nowMs;
  } else {
    const idealNextMs = stateTiming.lastIdealJsMs + sourceDeltaMs;
    const timeUntilIdealMs = idealNextMs - nowMs;
    if (timeUntilIdealMs > stateEarlyThresholdMs) {
      targetMs = nowMs + timeUntilIdealMs * statePacingDamping;
      stateTiming.lastIdealJsMs += sourceDeltaMs * statePacingDamping;
    } else {
      stateTiming.lastIdealJsMs = nowMs;
    }
  }

  queuedStateFrames.push({ targetMs, payload });
  while (queuedStateFrames.length > stateQueueMaxLength) {
    queuedStateFrames.shift();
  }
}

function applyQueuedState() {
  const nowMs = performance.now();
  let frame = null;
  while (queuedStateFrames.length > 0 && queuedStateFrames[0].targetMs <= nowMs) {
    frame = queuedStateFrames.shift();
  }
  if (!frame) return;
  latestRobotPosePayload = frame.payload;
  if (!browserPhysicsEnabled) applyRobotPose(frame.payload);
}

function createLidarMaterial() {
  if (lidarMaterial) return lidarMaterial;

  const shaders = BABYLON.Effect.ShadersStore;
  shaders.lidarPointVertexShader = `
    precision highp float;
    attribute vec3 position;
    attribute vec4 color;
    uniform mat4 worldViewProjection;
    uniform float pointSize;
    varying vec4 vColor;

    void main(void) {
      gl_Position = worldViewProjection * vec4(position, 1.0);
      gl_PointSize = pointSize;
      vColor = color;
    }
  `;
  shaders.lidarPointFragmentShader = `
    precision highp float;
    varying vec4 vColor;

    void main(void) {
      vec2 delta = gl_PointCoord - vec2(0.5);
      float radiusSquared = dot(delta, delta);
      if (radiusSquared > 0.25) discard;
      float edgeAlpha = smoothstep(0.25, 0.16, radiusSquared);
      gl_FragColor = vec4(vColor.rgb, vColor.a * edgeAlpha);
    }
  `;
  lidarMaterial = new BABYLON.ShaderMaterial(
    "lidarMaterial",
    scene,
    { vertex: "lidarPoint", fragment: "lidarPoint" },
    {
      attributes: ["position", "color"],
      uniforms: ["worldViewProjection", "pointSize"],
      needAlphaBlending: true,
    },
  );
  lidarMaterial.pointsCloud = true;
  lidarMaterial.setFloat("pointSize", lidarPointSizePx);
  lidarMaterial.backFaceCulling = false;
  return lidarMaterial;
}

function updatePointCloud(payload) {
  const count = payload.count || 0;
  if (count === 0 || !payload.positions || !payload.colors) return;
  perfCounters.pointcloudFrames += 1;

  const positions = payload.positions instanceof Float32Array
    ? payload.positions
    : Float32Array.from(payload.positions);
  const packedColors = payload.colors;
  const colors = new Float32Array(count * 4);
  for (let i = 0; i < count; i += 1) {
    colors[i * 4 + 0] = packedColors[i * 3 + 0] / 255;
    colors[i * 4 + 1] = packedColors[i * 3 + 1] / 255;
    colors[i * 4 + 2] = packedColors[i * 3 + 2] / 255;
    colors[i * 4 + 3] = 0.94;
  }

  if (lidarMesh && lidarPointCount === count) {
    lidarMesh.updateVerticesData(
      BABYLON.VertexBuffer.PositionKind,
      positions,
      true,
      false,
    );
    lidarMesh.updateVerticesData(
      BABYLON.VertexBuffer.ColorKind,
      colors,
      true,
      false,
    );
    lidarMesh.setEnabled(lidarVisible);
    perfCounters.pointcloudBufferUpdates += 1;
    return;
  }

  if (lidarMesh) {
    lidarMesh.dispose();
    lidarMesh = null;
    lidarPointCount = 0;
  }

  const nextMesh = new BABYLON.Mesh("lidarCloud", scene);
  const vertexData = new BABYLON.VertexData();
  vertexData.positions = positions;
  vertexData.colors = colors;
  vertexData.applyToMesh(nextMesh, true);
  nextMesh.hasVertexAlpha = true;
  nextMesh.alwaysSelectAsActiveMesh = true;
  nextMesh.isPickable = false;

  nextMesh.material = createLidarMaterial();
  nextMesh.setEnabled(lidarVisible);

  lidarMesh = nextMesh;
  lidarPointCount = count;
  perfCounters.pointcloudRecreates += 1;
}

// ─── Entity world ───────────────────────────────────────────────────────
//
// Python sends spawn/despawn/set_pose/apply_velocity over WS (JSON,
// dispatched in the worker handler below). State is broadcast back per
// Havok physics tick, throttled to ~30 Hz.

function physicsShapeFromHint(hint) {
  switch (hint) {
    case "box":
      return BABYLON.PhysicsShapeType.BOX;
    case "sphere":
      return BABYLON.PhysicsShapeType.SPHERE;
    case "cylinder":
      return BABYLON.PhysicsShapeType.CYLINDER;
    case "mesh":
    default:
      return BABYLON.PhysicsShapeType.MESH;
  }
}

function physicsShapeForEntity(descriptor, mass) {
  if (descriptor.shape_hint === "mesh" && descriptor.kind === "dynamic" && mass > 0) {
    // Havok/Babylon can use triangle meshes reliably for static scene
    // collision, but dynamic triangle-mesh rigid bodies are unstable for
    // authored furniture. Keep the exact mesh for rendering + lidar; use a
    // convex hull only for the live rigid-body solver.
    return BABYLON.PhysicsShapeType.CONVEX_HULL ?? BABYLON.PhysicsShapeType.MESH;
  }
  return physicsShapeFromHint(descriptor.shape_hint);
}

function buildPrimitiveMesh(descriptor) {
  const ext = descriptor.extents || [];
  switch (descriptor.shape_hint) {
    case "box": {
      const w = ext[0] || 1.0;
      const h = ext[1] || 1.0;
      const d = ext[2] || 1.0;
      return BABYLON.MeshBuilder.CreateBox(
        `entity:${descriptor.entity_id}`,
        { width: w, height: h, depth: d },
        scene,
      );
    }
    case "sphere": {
      const r = ext[0] || 0.5;
      return BABYLON.MeshBuilder.CreateSphere(
        `entity:${descriptor.entity_id}`,
        { diameter: 2 * r },
        scene,
      );
    }
    case "cylinder": {
      const r = ext[0] || 0.5;
      const h = ext[1] || 1.0;
      return BABYLON.MeshBuilder.CreateCylinder(
        `entity:${descriptor.entity_id}`,
        { diameterTop: 2 * r, diameterBottom: 2 * r, height: h },
        scene,
      );
    }
    default:
      return null;
  }
}

async function buildMeshEntity(descriptor) {
  if (!descriptor.mesh_ref) {
    console.warn(`entity_spawn ${descriptor.entity_id}: mesh shape requires mesh_ref`);
    return null;
  }
  let result;
  try {
    result = await BABYLON.SceneLoader.ImportMeshAsync(
      null,
      "/assets/",
      descriptor.mesh_ref,
      scene,
    );
  } catch (error) {
    console.warn(`entity_spawn ${descriptor.entity_id}: failed to import mesh_ref`, error);
    return null;
  }

  for (const light of result.lights || []) light.dispose();
  for (const camera of result.cameras || []) camera.dispose();

  const nodes = [...(result.meshes || []), ...(result.transformNodes || [])];
  const meshes = (result.meshes || []).filter(
    (mesh) => mesh.getTotalVertices && mesh.getTotalVertices() > 0,
  );
  if (meshes.length === 0) {
    console.warn(`entity_spawn ${descriptor.entity_id}: mesh_ref has no renderable meshes`);
    for (const node of nodes) node.dispose?.();
    return null;
  }

  for (const node of nodes) {
    node.computeWorldMatrix?.(true);
    if ("isPickable" in node) {
      node.isPickable = false;
    }
  }

  const collider = BABYLON.Mesh.MergeMeshes(
    meshes,
    false,
    true,
    undefined,
    false,
    true,
  );
  if (!collider) {
    console.warn(`entity_spawn ${descriptor.entity_id}: failed to merge mesh collider`);
    for (const node of nodes) node.dispose?.();
    return null;
  }
  collider.computeWorldMatrix(true);
  collider.bakeCurrentTransformIntoVertices();
  collider.name = `entity:${descriptor.entity_id}`;
  collider.position.set(0, 0, 0);
  collider.scaling.set(1, 1, 1);
  collider.rotationQuaternion = BABYLON.Quaternion.Identity();
  collider.isVisible = false;
  collider.isPickable = false;
  collider.metadata = { dimosEntityCollider: true, entityId: descriptor.entity_id };

  for (const node of nodes) {
    if (node === collider) continue;
    if (!node.parent) {
      node.parent = collider;
    }
  }
  return { mesh: collider, visualNodes: nodes };
}

async function buildEntityMesh(descriptor) {
  if (descriptor.shape_hint === "mesh") {
    return buildMeshEntity(descriptor);
  }
  const mesh = buildPrimitiveMesh(descriptor);
  if (!mesh) return null;
  return { mesh, visualNodes: [] };
}

function applyPoseWire(mesh, pose) {
  mesh.position.set(pose.x || 0, pose.y || 0, pose.z || 0);
  if (!mesh.rotationQuaternion) {
    mesh.rotationQuaternion = BABYLON.Quaternion.Identity();
  }
  mesh.rotationQuaternion.set(
    pose.qx || 0,
    pose.qy || 0,
    pose.qz || 0,
    pose.qw === undefined ? 1 : pose.qw,
  );
}

async function attachEntityVisual(descriptor, colliderMesh) {
  if (!descriptor.mesh_ref) return [];
  try {
    const result = await BABYLON.SceneLoader.ImportMeshAsync(
      null,
      "/assets/",
      descriptor.mesh_ref,
      scene,
    );
    const nodes = [...(result.meshes || []), ...(result.transformNodes || [])];
    for (const node of nodes) {
      if (node === colliderMesh) continue;
      if (!node.parent) {
        node.parent = colliderMesh;
      }
      if ("isPickable" in node) {
        node.isPickable = false;
      }
    }
    if (nodes.length > 0) {
      colliderMesh.isVisible = false;
    }
    return nodes;
  } catch (error) {
    console.warn(`entity ${descriptor.entity_id}: failed to load mesh_ref`, error);
    return [];
  }
}

async function handleEntitySpawn(payload) {
  const desc = payload.descriptor || {};
  const id = desc.entity_id;
  if (!id) return;
  if (entities.has(id)) {
    // Idempotent re-spawn (e.g. reconnect replay): just apply the pose.
    handleEntitySetPose({ entity_id: id, pose: payload.pose || {} });
    return;
  }
  if (!(await physicsReady)) {
    console.warn(`entity_spawn ${id}: physics not ready, dropping`);
    return;
  }
  const built = await buildEntityMesh(desc);
  if (!built) {
    console.warn(
      `entity_spawn ${id}: cannot build shape_hint=${desc.shape_hint}`,
    );
    return;
  }
  const mesh = built.mesh;
  applyPoseWire(mesh, payload.pose || {});
  mesh.computeWorldMatrix(true);
  mesh.refreshBoundingInfo(true);

  // mass=0 → kinematic (program-driven via set_entity_pose). Matches the
  // semantics in entity.py: dynamic with mass>0 actually gets simulated.
  const wantsKinematic = desc.kind !== "dynamic" || (desc.mass || 0) === 0;
  const mass = wantsKinematic ? 0 : desc.mass;
  let aggregate;
  try {
    aggregate = new BABYLON.PhysicsAggregate(
      mesh,
      physicsShapeForEntity(desc, mass),
      { mass },
      scene,
    );
  } catch (error) {
    console.warn(`entity_spawn ${id}: aggregate failed:`, error);
    mesh.dispose();
    return;
  }
  // Kinematic bodies: keep prestep enabled so set_entity_pose teleports
  // are picked up before the next integration.
  if (wantsKinematic && aggregate.body && "disablePreStep" in aggregate.body) {
    aggregate.body.disablePreStep = false;
  }
  let visualNodes = built.visualNodes;
  if (desc.shape_hint !== "mesh") {
    visualNodes = await attachEntityVisual(desc, mesh);
  }
  const entry = {
    mesh,
    aggregate,
    visualNodes,
    descriptor: desc,
    kinematic: wantsKinematic,
  };
  entities.set(id, entry);
  setEntityVisualsVisible(entry, sceneVisible);
  if (ui.setEntityStatus) ui.setEntityStatus(`${entities.size} active`);
  broadcastEntityStates(true);
}

function handleEntityDespawn(payload) {
  const entry = entities.get(payload.entity_id);
  if (!entry) return;
  try {
    entry.aggregate.dispose();
  } catch (_) {
    // best-effort
  }
  try {
    entry.mesh.dispose();
  } catch (_) {
    // best-effort
  }
  entities.delete(payload.entity_id);
  if (ui.setEntityStatus) ui.setEntityStatus(`${entities.size} active`);
  broadcastEntityStates(true);
}

function handleEntitySetPose(payload) {
  const entry = entities.get(payload.entity_id);
  if (!entry) return;
  applyPoseWire(entry.mesh, payload.pose || {});
  // For dynamic bodies, zero velocities so the teleport doesn't carry
  // stale momentum into the next step.
  if (!entry.kinematic && entry.aggregate.body) {
    try {
      entry.aggregate.body.setLinearVelocity(BABYLON.Vector3.Zero());
      entry.aggregate.body.setAngularVelocity(BABYLON.Vector3.Zero());
    } catch (_) {
      // ignore
    }
  }
}

function handleEntityApplyVelocity(payload) {
  const entry = entities.get(payload.entity_id);
  if (!entry || entry.kinematic) return;
  const t = payload.twist || {};
  try {
    entry.aggregate.body.setLinearVelocity(
      new BABYLON.Vector3(t.lx || 0, t.ly || 0, t.lz || 0),
    );
    entry.aggregate.body.setAngularVelocity(
      new BABYLON.Vector3(t.ax || 0, t.ay || 0, t.az || 0),
    );
  } catch (error) {
    console.warn(`apply_velocity ${payload.entity_id} failed:`, error);
  }
}

function roundedStateValue(value, scale) {
  return Math.round(finiteNumber(value, 0) * scale);
}

function entityStateSignature(states) {
  return JSON.stringify(states.map((state) => [
    state.entity_id,
    roundedStateValue(state.pose.x, 100),
    roundedStateValue(state.pose.y, 100),
    roundedStateValue(state.pose.z, 100),
    roundedStateValue(state.pose.qw, 1000),
    roundedStateValue(state.pose.qx, 1000),
    roundedStateValue(state.pose.qy, 1000),
    roundedStateValue(state.pose.qz, 1000),
  ]));
}

function broadcastEntityStates(force = false) {
  if (entities.size === 0 && !force) return;
  const now = performance.now();
  if (!force && now - lastEntityBroadcast < entityBroadcastIntervalMs) return;
  const ts = Date.now() / 1000.0;
  const states = [];
  for (const [id, entry] of entities) {
    const p = entry.mesh.position;
    const q = entry.mesh.rotationQuaternion || BABYLON.Quaternion.Identity();
    states.push({
      entity_id: id,
      frame_id: "world",
      ts,
      pose: { x: p.x, y: p.y, z: p.z, qw: q.w, qx: q.x, qy: q.y, qz: q.z },
    });
  }
  const signature = entityStateSignature(states);
  if (
    !force
    && signature === lastEntityBroadcastSignature
    && now - lastEntityBroadcastKeepalive < entityBroadcastKeepaliveMs
  ) {
    lastEntityBroadcast = now;
    return;
  }
  lastEntityBroadcast = now;
  lastEntityBroadcastSignature = signature;
  lastEntityBroadcastKeepalive = now;
  sendSocketPayload({ type: "entity_states", states });
}

physicsReady.then((ok) => {
  if (ok) scene.onAfterPhysicsObservable.add(() => broadcastEntityStates(false));
});

// `window.__pimsimReady` becomes true once physics has loaded, the WS to
// the Python module is up, and (if requested) the scene assets have
// settled. Headless test harnesses (Playwright) poll this to know when
// it's safe to send respawn/cmd_vel/spawn_entity messages.
let __pimsimSceneReady = false;
async function evaluatePimsimReady() {
  if (!(await physicsReady)) return;
  if (!streamRef.ready) return;
  if (!__pimsimSceneReady) return;
  window.__pimsimReady = true;
}
function markPimsimSceneReady() {
  __pimsimSceneReady = true;
  evaluatePimsimReady();
}

// ─── LCM <-> WS bridge subscriptions ────────────────────────────────────
//
// pimsim's BabylonSceneViewerModule subscribes to every LCM channel and
// forwards raw packets to /lcm-ws. @dimos/msgs decodes them in the
// lcm_client.js module worker. Here we just dispatch decoded messages
// into the existing Babylon renderers — same hooks the old binary frames
// fed into, no rendering changes.

function whenLcmReady(callback) {
  if (window.dimosLcm && window.dimosMsgs) {
    callback();
    return;
  }
  setTimeout(() => whenLcmReady(callback), 50);
}

// Mike Bostock's polynomial approximation of Google's "turbo" colormap.
function turboColor(t) {
  const x = Math.max(0, Math.min(1, t));
  const r = 34.61 + x * (1172.33 - x * (10793.56 - x * (33300.12 - x * (38394.49 - x * 14825.05))));
  const g = 23.31 + x * (557.33 + x * (1225.33 - x * (3574.96 - x * (1073.77 + x * 707.56))));
  const b = 27.2 + x * (3211.1 - x * (15327.97 - x * (27814 - x * (22569.18 - x * 6838.66))));
  return [
    Math.max(0, Math.min(255, r)),
    Math.max(0, Math.min(255, g)),
    Math.max(0, Math.min(255, b)),
  ];
}

function findPointCloudField(fields, name) {
  for (const f of fields) if (f.name === name) return f;
  return null;
}

// Decode a sensor_msgs.PointCloud2 (xyz + optional packed-float32 rgb) into
// the {count, positions, colors} shape `updatePointCloud` already expects.
function pointCloud2ToRenderPayload(msg) {
  const fx = findPointCloudField(msg.fields, "x");
  const fy = findPointCloudField(msg.fields, "y");
  const fz = findPointCloudField(msg.fields, "z");
  if (!fx || !fy || !fz) return null;

  const step = msg.point_step;
  if (!step || !msg.data || !msg.data.byteLength) return null;
  const count = Math.floor(msg.data.byteLength / step);
  if (count === 0) return null;

  const view = new DataView(
    msg.data.buffer,
    msg.data.byteOffset,
    msg.data.byteLength,
  );
  const littleEndian = !msg.is_bigendian;
  const positions = new Float32Array(count * 3);
  const frgb = findPointCloudField(msg.fields, "rgb");
  const colorBuf = new Uint8Array(count * 3);

  // First pass: positions + z range for fallback colormap.
  let zMin = Infinity;
  let zMax = -Infinity;
  for (let i = 0; i < count; i += 1) {
    const o = i * step;
    const x = view.getFloat32(o + fx.offset, littleEndian);
    const y = view.getFloat32(o + fy.offset, littleEndian);
    const z = view.getFloat32(o + fz.offset, littleEndian);
    if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) {
      positions[i * 3] = 0;
      positions[i * 3 + 1] = 0;
      positions[i * 3 + 2] = 0;
      continue;
    }
    positions[i * 3] = x;
    positions[i * 3 + 1] = y;
    positions[i * 3 + 2] = z;
    if (z < zMin) zMin = z;
    if (z > zMax) zMax = z;
  }

  if (frgb) {
    // ROS convention: 4 packed bytes (B,G,R,A) reinterpreted as float32.
    for (let i = 0; i < count; i += 1) {
      const o = i * step + frgb.offset;
      colorBuf[i * 3 + 0] = view.getUint8(o + 2); // R
      colorBuf[i * 3 + 1] = view.getUint8(o + 1); // G
      colorBuf[i * 3 + 2] = view.getUint8(o + 0); // B
    }
  } else {
    const denom = (zMax - zMin) || 1.0;
    for (let i = 0; i < count; i += 1) {
      const t = (positions[i * 3 + 2] - zMin) / denom;
      const [r, g, b] = turboColor(t);
      colorBuf[i * 3 + 0] = r;
      colorBuf[i * 3 + 1] = g;
      colorBuf[i * 3 + 2] = b;
    }
  }

  return { count, positions, colors: colorBuf };
}

function subscribeLcmTopics() {
  const M = window.dimosMsgs;
  const lcm = window.dimosLcm;
  if (!M || !lcm) return;

  let pathVersion = 0;

  lcm.subscribe("/global_map", M.sensor_msgs.PointCloud2, (msg) => {
    const payload = pointCloud2ToRenderPayload(msg);
    if (payload) updatePointCloud(payload);
  });

  lcm.subscribe("/nav_path", M.nav_msgs.Path, (msg) => {
    pathVersion += 1;
    const pts = (msg.poses || []).map((ps) => [
      ps.pose.position.x,
      ps.pose.position.y,
      ps.pose.position.z,
    ]);
    updatePath(pts, pathVersion);
  });

  lcm.subscribe("/coordinator/joint_state", M.sensor_msgs.JointState, (msg) => {
    if (!msg.name || !msg.position) return;
    const joints = {};
    for (let i = 0; i < msg.name.length && i < msg.position.length; i += 1) {
      // Match the canonicalisation pimsim's old _make_state_payload did so
      // the slider HUD keys still line up.
      let k = msg.name[i];
      const slash = k.indexOf("/");
      if (slash >= 0) k = k.slice(slash + 1);
      if (k.endsWith("_joint")) k = k.slice(0, -"_joint".length);
      joints[k] = Number(msg.position[i]);
    }
    _updateSlidersFromState(joints);
  });

  lcm.subscribe("/nav_cmd_vel", M.geometry_msgs.Twist, (msg) => {
    if (!browserPhysicsEnabled) return;
    setBrowserPhysicsCommand({
      linear: [msg.linear.x, msg.linear.y, msg.linear.z],
      angular: [msg.angular.x, msg.angular.y, msg.angular.z],
    });
  });
}

function connectStreamWorker() {
  const worker = new Worker("/static/stream_worker.js");
  streamRef.worker = worker;
  streamRef.ready = false;
  worker.onmessage = (event) => {
    const message = event.data || {};
    switch (message.type) {
      case "status":
        streamRef.ready = Boolean(message.ready);
        setStatus(message.status);
        if (streamRef.ready && browserPhysicsEnabled) publishBrowserSimOdom(true);
        if (streamRef.ready) evaluatePimsimReady();
        break;
      case "state":
        // Only JSON-only multi-tab entity replay + sim respawn ride here now.
        // Joint state, nav path, pointcloud, and planner cmd_vel all arrive
        // via /lcm-ws and are dispatched in subscribeLcmTopics() below.
        if (message.payload.type === "entity_spawn") {
          handleEntitySpawn(message.payload);
        }
        if (message.payload.type === "entity_despawn") {
          handleEntityDespawn(message.payload);
        }
        if (message.payload.type === "entity_set_pose") {
          handleEntitySetPose(message.payload);
        }
        if (message.payload.type === "entity_apply_velocity") {
          handleEntityApplyVelocity(message.payload);
        }
        if (message.payload.type === "sim_respawn") {
          setBrowserPhysicsPose(message.payload.pose || {}, true);
        }
        break;
      case "robot_pose":
        queueRobotPose({
          count: message.count,
          time: message.time,
          poses: new Float32Array(
            message.buffer,
            robotPoseHeaderBytes,
            message.count * 7,
          ),
        });
        break;
      case "camera":
        updateCameraFrame(message.cameraName, message.buffer, message.jpegOffset);
        break;
      case "error":
        console.warn(message.message);
        break;
      default:
        break;
    }
  };
  worker.onerror = (event) => {
    console.error(event);
    streamRef.ready = false;
    setStatus("stream worker error");
  };
  worker.postMessage({ type: "connect" });
  return worker;
}

function updateCameraFrame(cameraName, buffer, jpegOffset) {
  if (ui.updateCameraFrame) {
    ui.updateCameraFrame(cameraName, buffer, jpegOffset);
    return;
  }
  console.warn("camera frame dropped: PimSimUI.updateCameraFrame unavailable", cameraName);
}

function installClickPublisher() {
  scene.onPointerObservable.add((pointerInfo) => {
    if (pointerInfo.type !== BABYLON.PointerEventTypes.POINTERPICK) return;
    const event = pointerInfo.event;
    if (event.target !== canvas) return;

    const publishNav = clickMode === "nav" || event.shiftKey;
    const publishPoint = clickMode === "point" || event.altKey;
    const publishSpawn = clickMode === "spawn";
    if (!publishNav && !publishPoint && !publishSpawn) return;

    if (!streamRef.ready) return;

    if (publishNav || publishSpawn) {
      const point = pickScenePoint();
      if (!point) return;
      if (publishSpawn) {
        spawnMarker = placeMarker(
          spawnMarker,
          "spawnMarker",
          new BABYLON.Vector3(point.x, point.y, point.z + 0.12),
          spawnMarkerMaterial,
          0.28,
        );
        sendSocketPayload({
          type: "respawn_at",
          point: [point.x, point.y, point.z],
        });
        setClickMode(null);
        setStatus("spawn requested");
        return;
      }
      navGoalMarker = placeMarker(
        navGoalMarker,
        "navGoalMarker",
        new BABYLON.Vector3(point.x, point.y, point.z + 0.08),
        navMarkerMaterial,
        0.22,
      );
      publishLcmPointStamped("/clicked_point", point);
      setClickMode(null);
      setStatus("nav target sent");
      return;
    }

    const pick = pointerInfo.pickInfo;
    let point = null;
    if (pick && pick.hit && pick.pickedPoint) {
      point = pick.pickedPoint;
    } else {
      const ray = scene.createPickingRay(
        scene.pointerX,
        scene.pointerY,
        BABYLON.Matrix.Identity(),
        camera,
      );
      if (Math.abs(ray.direction.z) < 1e-6) return;
      const distance = (1.0 - ray.origin.z) / ray.direction.z;
      if (distance <= 0) return;
      point = ray.origin.add(ray.direction.scale(distance));
    }
    pointGoalMarker = placeMarker(
      pointGoalMarker,
      "pointGoalMarker",
      point,
      pointMarkerMaterial,
      0.16,
    );
    publishLcmPointStamped("/point_goal", point);
    setClickMode(null);
    setStatus("point target sent");
  });
}

function publishLcmPointStamped(topic, point) {
  const M = window.dimosMsgs;
  const lcm = window.dimosLcm;
  if (!M || !lcm) return;
  try {
    const ts = Date.now() / 1000;
    const sec = Math.floor(ts);
    const nanosec = Math.floor((ts - sec) * 1e9);
    const stamped = new M.geometry_msgs.PointStamped({
      header: new M.std_msgs.Header({
        stamp: new M.builtin_interfaces.Time({ sec, nanosec }),
        frame_id: "map",
      }),
      point: new M.geometry_msgs.Point({ x: point.x, y: point.y, z: point.z }),
    });
    lcm.publish(topic, stamped);
  } catch (err) {
    console.warn(`[lcm] ${topic} publish failed`, err);
  }
}

document.getElementById("toggleScene").onclick = () => {
  const visible = document.getElementById("toggleScene").dataset.active !== "true";
  setSceneVisibility(visible);
};
document.getElementById("toggleCollision").onclick = () => {
  setCollisionVisibility(!collisionVisible);
};
document.getElementById("toggleRobot").onclick = () => {
  const visible = document.getElementById("toggleRobot").dataset.active !== "true";
  setRobotVisibility(visible);
};
document.getElementById("toggleDrive").onclick = () => setDriveEnabled(!driveEnabled);
document.getElementById("respawnRobot").onclick = () => {
  sendDriveCommand(true);
  sendSocketPayload({ type: "respawn" });
  setStatus("respawn requested");
};
document.getElementById("toggleLidar").onclick = () => setLidarVisibility(!lidarVisible);
const _toggleSplat = document.getElementById("toggleSplat");
if (_toggleSplat) _toggleSplat.onclick = () => setSplatVisibility(!splatVisible);
document.getElementById("toggleCamera").onclick = () => {
  const btn = document.getElementById("toggleCamera");
  const active = btn.dataset.active !== "true";
  btn.dataset.active = active ? "true" : "false";
  if (ui.setPanelActive) ui.setPanelActive("cameraPanel", active);
  else document.getElementById("cameraPanel").dataset.active = active ? "true" : "false";
};
document.getElementById("navClick").onclick = () => setClickMode("nav");
document.getElementById("pointClick").onclick = () => setClickMode("point");
document.getElementById("spawnClick").onclick = () => setClickMode("spawn");
document.getElementById("toggleDepth").onclick = () => setSceneDepthWrite(!sceneDepthEnabled);
document.getElementById("toggleWire").onclick = () => setSceneWireframe(!sceneWireEnabled);
document.getElementById("forceVisible").onclick = () => setForceVisible(!forceVisibleEnabled);
document.getElementById("focusRobot").onclick = focusRobot;
document.getElementById("loadScene").onclick = () => {
  if (!sceneConfig) return;
  (async () => {
    try {
      await loadSceneAsset(sceneConfig);
      await loadCollisionAsset(sceneConfig);
    } catch (error) {
      console.error(error);
      setStatus("scene load failed");
    }
  })();
};

document.getElementById("entityAdd").onclick = () => {
  // Drop the box 2 m above the camera target so Havok gravity catches it
  // on the way down — proves both the scene collision aggregate and the
  // entity body are wired correctly.
  const t = camera.getTarget();
  sendSocketPayload({
    type: "entity_test_add",
    point: [t.x, t.y, t.z + 2.0],
  });
};

document.getElementById("entityClear").onclick = () => {
  sendSocketPayload({ type: "entity_clear" });
};

// --- Policy arm / dry-run toggles ---
// Initial dataset.active reflects the coordinator's defaults for the
// typical real-hardware blueprint (unarmed, dry-run on). If the
// blueprint configured different defaults the button is still a plain
// toggle — click it once to sync.
document.getElementById("policyArm").onclick = () => {
  const btn = document.getElementById("policyArm");
  const engaged = btn.dataset.active !== "true";
  if (!sendSocketPayload({ type: "set_activated", engaged })) return;
  btn.dataset.active = engaged ? "true" : "false";
  setStatus(engaged ? "policy armed" : "policy disarmed");
};
document.getElementById("policyDryRun").onclick = () => {
  const btn = document.getElementById("policyDryRun");
  const enabled = btn.dataset.active !== "true";
  if (!sendSocketPayload({ type: "set_dry_run", enabled })) return;
  btn.dataset.active = enabled ? "true" : "false";
  setStatus(enabled ? "dry-run on" : "dry-run off (live)");
};

// --- Arm slider panel ---
// Toggle visibility
document.getElementById("armsToggle").onclick = () => {
  const btn = document.getElementById("armsToggle");
  const active = btn.dataset.active !== "true";
  btn.dataset.active = active ? "true" : "false";
  if (ui.setPanelActive) ui.setPanelActive("armsPanel", active);
  else document.getElementById("armsPanel").dataset.active = active ? "true" : "false";
};

// Release: stop publishing arm commands (hand control back to MC)
document.getElementById("armsRelease").onclick = () => {
  if (sendSocketPayload({ type: "release_arms" })) {
    setStatus("arms released");
  }
};

// Build the slider list from /arms.json. Each slider sends an
// {type: arm_joint, name, position} message on input (throttled).
function _humanLabel(name) {
  // strip "left_"/"right_" prefix for column-internal display
  return name
    .replace(/^left_/, "")
    .replace(/^right_/, "")
    .replace(/_/g, " ");
}
// Track which slider the user is currently dragging so we don't
// overwrite its value from incoming joint-state updates.
const _armSliders = {};       // joint_name -> {slider, val, dragging}
let _armSendThrottle = {};
function _throttledSendJoint(name, position) {
  const now = performance.now();
  const last = _armSendThrottle[name] || 0;
  if (now - last < 30) return; // ~33 Hz max per slider
  _armSendThrottle[name] = now;
  sendSocketPayload({ type: "arm_joint", name, position });
}
function _buildSlider(joint) {
  const row = document.createElement("div");
  row.className = "arm-slider-row";

  const labelTop = document.createElement("div");
  labelTop.className = "joint-name";
  labelTop.textContent = _humanLabel(joint.name);
  row.appendChild(labelTop);

  const val = document.createElement("div");
  val.className = "joint-val";
  val.textContent = "  …  ";
  row.appendChild(val);

  const slider = document.createElement("input");
  slider.type = "range";
  slider.min = String(joint.min);
  slider.max = String(joint.max);
  slider.step = "0.005";
  // Default to midpoint until we get a real reading — visually obvious
  // it's not real yet (the value cell shows "..." until first update).
  slider.value = String((joint.min + joint.max) / 2);
  slider.disabled = true;  // enabled once a joint_state arrives

  const entry = { slider, val, dragging: false, ready: false };
  _armSliders[joint.name] = entry;

  slider.addEventListener("pointerdown", () => { entry.dragging = true; });
  slider.addEventListener("pointerup",   () => { entry.dragging = false; });
  slider.addEventListener("pointercancel", () => { entry.dragging = false; });

  slider.oninput = () => {
    const pos = parseFloat(slider.value);
    val.textContent = pos.toFixed(3);
    _throttledSendJoint(joint.name, pos);
  };
  slider.onchange = () => {
    // Final exact value on release (in case the throttle dropped it)
    const pos = parseFloat(slider.value);
    sendSocketPayload({ type: "arm_joint", name: joint.name, position: pos });
  };
  row.appendChild(slider);

  const range = document.createElement("div");
  range.className = "joint-range";
  const lo = document.createElement("span");
  lo.textContent = joint.min.toFixed(2);
  const hi = document.createElement("span");
  hi.textContent = joint.max.toFixed(2);
  range.appendChild(lo);
  range.appendChild(hi);
  row.appendChild(range);

  return row;
}

function _updateSlidersFromState(joints) {
  if (!joints) return;
  for (const [name, value] of Object.entries(joints)) {
    const entry = _armSliders[name];
    if (!entry) continue;
    if (!entry.ready) {
      // First sample → enable the slider, set it to actual position.
      entry.ready = true;
      entry.slider.disabled = false;
    }
    if (entry.dragging) continue;  // don't fight the user
    // Only set value when the slider is idle, so the user sees ground truth.
    entry.slider.value = String(value);
    entry.val.textContent = Number(value).toFixed(3);
  }
}

(async () => {
  try {
    const resp = await fetch("/arms.json");
    const data = await resp.json();
    const leftCol = document.getElementById("leftArmSliders");
    const rightCol = document.getElementById("rightArmSliders");
    for (const j of data.joints || []) {
      const row = _buildSlider(j);
      if (j.name.startsWith("left_")) leftCol.appendChild(row);
      else if (j.name.startsWith("right_")) rightCol.appendChild(row);
    }
  } catch (e) {
    console.error("Failed to load /arms.json:", e);
  }
})();

(async () => {
  try {
    const config = await loadConfig();
    sceneConfig = config;
    configureBrowserPhysics(config);
    await loadRobot();
    setupRobotCollider();   // fire-and-forget; awaits physicsReady internally
    await ensureSupportFloor(config);
    connectStreamWorker();
    whenLcmReady(subscribeLcmTopics);
    installClickPublisher();
    setStatus("live");
    if (sceneMode !== "0" && sceneMode !== "manual") {
      window.setTimeout(async () => {
        try {
          await loadSceneAsset(config);
          await loadCollisionAsset(config);
          markPimsimSceneReady();
        } catch (error) {
          console.error(error);
          setStatus("scene load failed");
        }
      }, 0);
    } else {
      // No scene to load (manual / empty) — ready as soon as WS + physics are up.
      markPimsimSceneReady();
    }
  } catch (error) {
    console.error(error);
    setStatus("load failed");
  }
})();

window.addEventListener("keydown", (event) => {
  const key = event.key.toLowerCase();
  if (key === "shift") pressedKeys.add("shift");
  if (key === " ") {
    if (driveEnabled) sendDriveCommand(true);
    event.preventDefault();
    return;
  }
  if (!["w", "a", "s", "d", "q", "e"].includes(key)) return;
  pressedKeys.add(key);
  event.preventDefault();
});

window.addEventListener("keyup", (event) => {
  const key = event.key.toLowerCase();
  if (key === "shift") pressedKeys.delete("shift");
  pressedKeys.delete(key);
});

window.addEventListener("blur", () => {
  // Clear any keys the user was holding when the window lost focus
  // so we don't keep walking on alt-tab. Only publish a stop if drive
  // was actually engaged — otherwise we'd send a spurious zero Twist
  // on every page refresh, which clobbers whatever cmd_vel source
  // is currently driving the robot (Quest joysticks, an agent, etc.)
  // for one tick and shows up as the robot going briefly damp.
  pressedKeys.clear();
  if (driveEnabled) sendDriveCommand(true);
});

function renderFrame() {
  stepBrowserPhysics();
  applyQueuedState();
  applyLatestRobotPose();
  updateKeyboardCamera();
  sendDriveCommand(false);
  scene.render();
  updatePerfCounters();
}

engine.runRenderLoop(renderFrame);
window.addEventListener("resize", () => engine.resize());
