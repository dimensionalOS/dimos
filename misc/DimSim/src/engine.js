import "./style.css";

import * as THREE from "three";
import { PointerLockControls } from "three/examples/jsm/controls/PointerLockControls.js";
import { AiAvatar } from "./AiAvatar.js";
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader.js";
import { RoomEnvironment } from "three/examples/jsm/environments/RoomEnvironment.js";
import { RoundedBoxGeometry } from "three/examples/jsm/geometries/RoundedBoxGeometry.js";

let RAPIER = null;
let _rapierInitPromise = null;
let rapierWorld = null;
let worldBody = null;
let playerBody = null;
let playerCollider = null;
let flyMode = true;
let ghostMode = true;
let characterController = null;
let _rapierStepFaultCount = 0;
let walkVerticalVel = 0;
let aiAgents = [];

// Track asset collider handles for cleanup
const _assetColliderHandles = new Map();

// Player dimensions (tuned smaller so you can fit inside tighter splat/glb interiors).
const PLAYER_RADIUS = 0.12;
const PLAYER_HALF_HEIGHT = 0.25;
const PLAYER_EYE_HEIGHT = PLAYER_HALF_HEIGHT + PLAYER_RADIUS + 0.2; // camera above body origin
const LIDAR_MOUNT_HEIGHT = 0.35; // Go2 lidar mount height above ground
// Real Go2 front camera height above ground in its home/operating crouch pose.
// Used for agent-POV captures so the rendered view matches what the hardware
// camera would see (low, not eye-level). ~0.30 m matches the MuJoCo go2.xml
// home keyframe (thigh 0.9, calf -1.8) which the Go2 actually stands in.
const GO2_CAMERA_HEIGHT = 0.30;
// Go2 front RGB-D camera is mounted on the front of the head, forward of the
// body center. Offsetting places the camera origin outside the robot mesh so
// POV captures don't render the inside of the body.
const GO2_CAMERA_FORWARD = 0.18;

const canvas = document.getElementById("c");
const statusEl = document.getElementById("status");
const overlayEl = document.getElementById("overlay");
const simPanelCollapseBtn = document.getElementById("sim-panel-collapse");
const simPanelOpenBtn = document.getElementById("sim-panel-open");
const statusSimEl = document.getElementById("status-sim");
const agentPanelEl = document.getElementById("agent-panel");
const agentLastEl = document.getElementById("agent-last");
const agentShotImgEl = document.getElementById("agent-shot-img");
const agentReqMetaEl = document.getElementById("agent-req-meta");
const agentReqPromptEl = document.getElementById("agent-req-prompt");
const agentReqContextEl = document.getElementById("agent-req-context");
const agentRespRawEl = document.getElementById("agent-resp-raw");
const agentLogEl = document.getElementById("agent-log");
const agentTaskStatusEl = document.getElementById("agent-task-status");
const agentTaskInputEl = document.getElementById("agent-task-input");
const agentTaskStartBtn = document.getElementById("agent-task-start");
const agentTaskEndBtn = document.getElementById("agent-task-end");
const simCameraModeToggleBtn = document.getElementById("sim-camera-toggle");
const simViewRgbdBtn = document.getElementById("sim-view-rgbd");
const simViewLidarBtn = document.getElementById("sim-view-lidar");
const simViewCompareBtn = document.getElementById("sim-view-compare");
const simRgbdGrayBtn = document.getElementById("sim-rgbd-gray");
const simRgbdColormapBtn = document.getElementById("sim-rgbd-colormap");
const simRgbdAutoRangeBtn = document.getElementById("sim-rgbd-auto-range");
const simRgbdNoiseBtn = document.getElementById("sim-rgbd-noise");
const simRgbdSpeckleBtn = document.getElementById("sim-rgbd-speckle");
const simRgbdMinEl = document.getElementById("sim-rgbd-min");
const simRgbdMaxEl = document.getElementById("sim-rgbd-max");
const simRgbdMinValEl = document.getElementById("sim-rgbd-min-val");
const simRgbdMaxValEl = document.getElementById("sim-rgbd-max-val");
const simLidarColorRangeBtn = document.getElementById("sim-lidar-color-range");
const simLidarOrderedDebugBtn = document.getElementById("sim-lidar-ordered-debug");
const simLidarNoiseBtn = document.getElementById("sim-lidar-noise");
const simLidarMultiReturnBtn = document.getElementById("sim-lidar-multireturn");

// ── dimos integration mode ──────────────────────────────────────────────────
// Activated via ?dimos=1 URL param or window.__dimosMode (injected by Deno bridge server).
// When active: internal VLM loop disabled, agent pose driven by external /odom,
// sensor data (RGB, depth, LiDAR) published as LCM packets via WebSocket bridge.
const _dimosParams = new URLSearchParams(window.location.search);
const dimosMode = _dimosParams.get("dimos") === "1" || window.__dimosMode === true;
if (dimosMode) document.body.classList.add("dimos-mode");
const dimosScene = _dimosParams.get("scene") || window.__dimosScene || null;
let simSensorViewMode = "rgb"; // "rgb" | "rgbd" | "lidar"
let simCompareView = false; // show RGB + RGB-D + LiDAR side-by-side
let simPanelCollapsed = false;
let simUserCameraMode = localStorage.getItem("sparkWorldSimCameraMode") === "agent" ? "agent" : "user";
let rgbdVizMode = "colormap"; // "colormap" | "gray"
let rgbdAutoRange = true;
let rgbdRangeMinM = 0.2;
let rgbdRangeMaxM = 12.0;
let rgbdNoiseEnabled = false;
let rgbdSpeckleEnabled = false;
let lidarColorByRange = false; // false = intensity grayscale (realistic default)
let lidarOrderedDebugView = false; // false=unordered 3D cloud, true=ordered rings debug
let lidarNoiseEnabled = false; // deterministic range noise + dropouts
let lidarMultiReturnMode = "strongest"; // "strongest" | "last"
let worldKey = localStorage.getItem("sparkWorldLastWorldKey") ?? "default";

function clampNum(v, min, max) {
  const n = Number(v);
  if (!Number.isFinite(n)) return min;
  return Math.min(max, Math.max(min, n));
}

function normalizeHexColor(value, fallback) {
  try {
    return `#${new THREE.Color(value).getHexString()}`;
  } catch {
    return fallback;
  }
}

function createDefaultSceneSettings() {
  return {
    sky: {
      enabled: false,
      topColor: "#7aa9ff",
      horizonColor: "#cfe5ff",
      bottomColor: "#f4f8ff",
      brightness: 1.0,
      softness: 1.35,
      sunStrength: 0.18,
      sunHeight: 0.45,
    },
  };
}

function normalizeSceneSettings(raw) {
  const defaults = createDefaultSceneSettings();
  const src = raw && typeof raw === "object" ? raw : {};
  const srcSky = src.sky && typeof src.sky === "object" ? src.sky : {};
  return {
    sky: {
      enabled: !!srcSky.enabled,
      topColor: normalizeHexColor(srcSky.topColor, defaults.sky.topColor),
      horizonColor: normalizeHexColor(srcSky.horizonColor, defaults.sky.horizonColor),
      bottomColor: normalizeHexColor(srcSky.bottomColor, defaults.sky.bottomColor),
      brightness: clampNum(srcSky.brightness, 0.2, 2.0),
      softness: clampNum(srcSky.softness, 0.2, 3.0),
      sunStrength: clampNum(srcSky.sunStrength, 0.0, 1.0),
      sunHeight: clampNum(srcSky.sunHeight, -0.2, 1.0),
    },
  };
}

function serializeSceneSettings() {
  return normalizeSceneSettings(sceneSettings);
}

let sceneSettings = createDefaultSceneSettings();
let tags = [];
let selectedTagId = null;
let draftTag = null; // tag being edited/created
const tagsGroup = new THREE.Group();
tagsGroup.name = "tagsGroup";

// Assets (Edit mode)
let assets = []; // [{id,title,notes,states:[{id,name,glbName,dataBase64,interactions:[{id,label,to}]}],currentStateId,actions:[{id,label,from,to}],transform:{...}, _colliderHandle?}]
const assetsGroup = new THREE.Group();
assetsGroup.name = "assetsGroup";
const gltfLoader = new GLTFLoader();

// =============================================================================
// BLOB SHADOW – lightweight planar shadow for GLB assets (no shadow maps needed)
// =============================================================================
// Procedural radial-gradient texture (created once, shared by all blob shadows)
let _blobShadowTexture = null;
let _blobShadowGeometry = null;

function getBlobShadowTexture() {
  if (_blobShadowTexture) return _blobShadowTexture;
  const size = 128;
  const canvas = document.createElement("canvas");
  canvas.width = size;
  canvas.height = size;
  // Use a GRAYSCALE gradient: white = opaque shadow, black = transparent.
  // This texture will be used as an alphaMap (only the luminance/R channel matters).
  const ctx = canvas.getContext("2d");
  const gradient = ctx.createRadialGradient(size / 2, size / 2, 0, size / 2, size / 2, size / 2);
  gradient.addColorStop(0, "#ffffff");     // center: fully opaque
  gradient.addColorStop(0.35, "#cccccc");
  gradient.addColorStop(0.65, "#444444");
  gradient.addColorStop(1, "#000000");      // edge: fully transparent
  ctx.fillStyle = gradient;
  ctx.fillRect(0, 0, size, size);
  _blobShadowTexture = new THREE.CanvasTexture(canvas);
  _blobShadowTexture.needsUpdate = true;
  return _blobShadowTexture;
}

function getBlobShadowGeometry() {
  if (_blobShadowGeometry) return _blobShadowGeometry;
  _blobShadowGeometry = new THREE.PlaneGeometry(1, 1);
  // Rotate so the plane lies flat on the XZ ground plane (face up)
  _blobShadowGeometry.rotateX(-Math.PI / 2);
  return _blobShadowGeometry;
}

// Create a blob shadow mesh sized to an asset's footprint.
// Returns a Mesh that should be added as a child of the asset root.
// `opts` = { opacity, scale, stretch, rotationDeg, offsetX, offsetY, offsetZ }
function createBlobShadow(assetId, footprintX, footprintZ, localGroundY, opts) {
  const o = opts || {};
  const userScale = o.scale ?? 1.0;
  const userOpacity = o.opacity ?? 0.5;
  const stretch = o.stretch ?? 1.0;     // >1 elongates X, <1 elongates Z
  const rotDeg = o.rotationDeg ?? 0;    // rotation around Y in degrees
  const offsetX = o.offsetX ?? 0;
  const offsetY = o.offsetY ?? 0;
  const offsetZ = o.offsetZ ?? 0;

  // Base diameter from asset footprint, then apply user scale
  const baseDiameter = Math.max(footprintX, footprintZ) * 1.1;
  const d = baseDiameter * userScale;
  if (d < 0.04) return null;

  const mat = new THREE.MeshBasicMaterial({
    color: 0x000000,
    alphaMap: getBlobShadowTexture(),
    transparent: true,
    depthWrite: false,
    depthTest: true,
    opacity: userOpacity,
    side: THREE.DoubleSide,
    // Use ONLY constant depth bias. Slope-based factor causes the blob to
    // appear to slide as the camera angle changes while moving.
    polygonOffset: true,
    polygonOffsetFactor: 0,
    polygonOffsetUnits: -300,
  });
  const mesh = new THREE.Mesh(getBlobShadowGeometry(), mat);
  // stretch > 1 makes the X axis wider; Z axis is inversely narrower to
  // keep the overall area roughly constant.
  const sx = d * stretch;
  const sz = d / stretch;
  mesh.scale.set(sx, 1, sz);
  // Raise slightly so it stays on/just above floor.
  mesh.position.set(offsetX, localGroundY + 0.08 + offsetY, offsetZ);
  // The shared geometry is already rotated to lie on XZ. An additional Y
  // rotation spins the ellipse around the vertical axis.
  mesh.rotation.y = (rotDeg * Math.PI) / 180;
  mesh.renderOrder = 1000;
  mesh.castShadow = false;
  mesh.receiveShadow = false;
  mesh.name = `blobShadow:${assetId}`;
  mesh.userData.isBlobShadow = true;
  mesh.userData._baseDiameter = baseDiameter;
  mesh.userData._baseLocalY = localGroundY + 0.08;
  return mesh;
}

// =============================================================================
// PRIMITIVES (Level Editor) – lightweight parametric shapes
// =============================================================================
let primitives = []; // [{id, type, name, dimensions:{...}, transform:{position,rotation,scale}, material:{color,roughness,metalness,textureDataUrl}, physics:bool, _colliderHandle?}]
const _assetBumpVelocities = new Map(); // assetId -> THREE.Vector3
const _playerPosPrevForBump = new THREE.Vector3();
let _playerPosPrevForBumpValid = false;
const _agentPosPrevForBump = new Map(); // agentId -> THREE.Vector3
let _lastBumpSaveAt = 0;
let _lastBumpColliderSyncAt = 0;
const primitivesGroup = new THREE.Group();
primitivesGroup.name = "primitivesGroup";

const PRIMITIVE_DEFAULTS = {
  box: {
    width: 1,
    height: 1,
    depth: 1,
    edgeRadius: 0,
    edgeSegments: 4,
    widthSegments: 1,
    heightSegments: 1,
    depthSegments: 1,
  },
  sphere: {
    radius: 0.5,
    widthSegments: 32,
    heightSegments: 16,
    phiStartDeg: 0,
    phiLengthDeg: 360,
    thetaStartDeg: 0,
    thetaLengthDeg: 180,
  },
  cylinder: { radiusTop: 0.5, radiusBottom: 0.5, height: 1, radialSegments: 32, heightSegments: 1, openEnded: 0 },
  cone: { radius: 0.5, height: 1, radialSegments: 32, heightSegments: 1, openEnded: 0 },
  torus: { radius: 0.5, tube: 0.15, radialSegments: 16, tubularSegments: 48, arcDeg: 360 },
  plane: { width: 2, height: 2, widthSegments: 1, heightSegments: 1 },
};

const PRIMITIVE_DIM_CONFIG = {
  width: { min: 0.05, max: 50, step: 0.05 },
  height: { min: 0.05, max: 50, step: 0.05 },
  depth: { min: 0.05, max: 50, step: 0.05 },
  radius: { min: 0.01, max: 20, step: 0.01 },
  radiusTop: { min: 0.01, max: 20, step: 0.01 },
  radiusBottom: { min: 0.01, max: 20, step: 0.01 },
  tube: { min: 0.01, max: 10, step: 0.01 },
  edgeRadius: { min: 0, max: 2.5, step: 0.01 },
  edgeSegments: { min: 1, max: 12, step: 1, integer: true },
  widthSegments: { min: 1, max: 128, step: 1, integer: true },
  heightSegments: { min: 1, max: 128, step: 1, integer: true },
  depthSegments: { min: 1, max: 128, step: 1, integer: true },
  radialSegments: { min: 3, max: 128, step: 1, integer: true },
  tubularSegments: { min: 3, max: 256, step: 1, integer: true },
  phiStartDeg: { min: 0, max: 360, step: 1 },
  phiLengthDeg: { min: 1, max: 360, step: 1 },
  thetaStartDeg: { min: 0, max: 180, step: 1 },
  thetaLengthDeg: { min: 1, max: 180, step: 1 },
  arcDeg: { min: 1, max: 360, step: 1 },
  openEnded: { min: 0, max: 1, step: 1, integer: true },
};


const PRIMITIVE_DIM_LABELS = {
  edgeRadius: "Roundness",
  edgeSegments: "Round Detail",
  widthSegments: "Detail X",
  heightSegments: "Detail Y",
  depthSegments: "Detail Z",
  radialSegments: "Circle Detail",
  tubularSegments: "Ring Detail",
  phiStartDeg: "Horizontal Cut Start",
  phiLengthDeg: "Horizontal Fill",
  thetaStartDeg: "Vertical Cut Start",
  thetaLengthDeg: "Vertical Fill",
  arcDeg: "Ring Opening",
  openEnded: "Open Ends",
  radiusTop: "Top Radius",
  radiusBottom: "Bottom Radius",
};


// =============================================================================
// EDITOR LIGHTS – user-placed lights with full control
// =============================================================================
let editorLights = []; // [{id, type, name, color, intensity, position:{x,y,z}, target:{x,y,z}, distance, angle, penumbra, castShadow, _lightObj?, _helperObj?}]
let groups = []; // [{id, name, children:[primId,...], pickable?}]
const lightsGroup = new THREE.Group();
lightsGroup.name = "lightsGroup";
const _assetRaycaster = new THREE.Raycaster();
const _agentAssetRaycaster = new THREE.Raycaster();
const _tmpV1 = new THREE.Vector3();
const _tmpV2 = new THREE.Vector3();
const _tmpV3 = new THREE.Vector3();

// Agent camera follow mode (first-person POV)
let agentCameraFollow = false;
let _agentFollowInitialized = false;

// Agent task state — per-agent tasks for parallel execution.
let agentTask = {
  active: false,
  instruction: "",
  startedAt: 0,
  finishedAt: 0,
  finishedReason: "",
  lastSummary: "",
};
const _agentTasks = new Map(); // agentId -> { active, instruction, startedAt, finishedAt, finishedReason, lastSummary }


function _setAgentTask(agentId, task) {
  _agentTasks.set(agentId, task);
  // Keep global agentTask in sync with the most recent active task (for UI compat)
  if (task.active) {
    agentTask = { ...task };
  }
}
let selectedAgentInspectorId = null;
const agentInspectorStateById = new Map(); // id -> { shot, request, response }
let agentCameraFollowId = null;
let agentUiSelectedLabelEl = null;
let agentUiSpawnBtn = null;
let agentUiFollowBtn = null;
let agentUiStopBtn = null;
let agentUiRemoveBtn = null;
let agentUiTaskInputEl = null;
let agentUiTaskRunBtn = null;
let agentTaskTargetId = null;
let agentBadgeLayerEl = null;
const agentBadgeElsById = new Map();
const MAX_AGENT_COUNT = 4;

// =============================================================================
// WORLD MANIFEST & LOADING
// =============================================================================

// Helper to normalize asset schema (backward compat)
// This function ensures all asset properties are properly loaded including states, interactions, and actions

const renderer = new THREE.WebGLRenderer({
  canvas,
  antialias: false,
  powerPreference: "high-performance",
  // Required for reading pixels from the canvas (agent POV capture)
  preserveDrawingBuffer: true,
});
renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
renderer.setSize(window.innerWidth, window.innerHeight, false);
renderer.outputColorSpace = THREE.SRGBColorSpace;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 1.1;

// Shadows: OFF by default. Enabled dynamically only when a light actually casts shadows.
// BasicShadowMap is fully deterministic (no PCF/stochastic filtering).
renderer.shadowMap.enabled = false;
renderer.shadowMap.type = THREE.BasicShadowMap;
renderer.shadowMap.autoUpdate = false; // we control when shadow maps update

const clock = new THREE.Clock();
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x06070a);

// Image-based lighting for PBR GLBs. This dramatically improves "too dark" assets.
try {
  const pmrem = new THREE.PMREMGenerator(renderer);
  scene.environment = pmrem.fromScene(new RoomEnvironment(), 0.04).texture;
  pmrem.dispose();
} catch {
  // ignore
}

const camera = new THREE.PerspectiveCamera(
  65,
  window.innerWidth / window.innerHeight,
  0.05,
  2000
);
camera.position.set(0, 1.7, 4);

// Lighting for non-splat geometry (assets/avatars).
// Splats are mostly self-lit visually; GLB assets need strong, stable fill to avoid looking black.
// Tagged dimsimDefault so clearDefaultLights() can drop them.
const ambientLight = new THREE.AmbientLight(0xffffff, 0.65);
ambientLight.userData.dimsimDefault = true;
scene.add(ambientLight);

const hemi = new THREE.HemisphereLight(0xffffff, 0x223344, 0.85);
hemi.position.set(0, 10, 0);
hemi.userData.dimsimDefault = true;
scene.add(hemi);

const dir = new THREE.DirectionalLight(0xffffff, 1.6);
dir.userData.dimsimDefault = true;
dir.position.set(8, 14, 6);
dir.castShadow = false; // off by default; user enables via Scene Lighting panel
dir.shadow.mapSize.width = 512;
dir.shadow.mapSize.height = 512;
dir.shadow.camera.near = 0.5;
dir.shadow.camera.far = 40;
dir.shadow.camera.left = -15;
dir.shadow.camera.right = 15;
dir.shadow.camera.top = 15;
dir.shadow.camera.bottom = -15;
dir.shadow.bias = -0.003;
scene.add(dir);

// Headlamp-style light attached to the camera so assets are visible wherever they are placed.
const headLamp = new THREE.PointLight(0xffffff, 1.4, 26, 1.5);
headLamp.position.set(0, 1.0, 0.6);
camera.add(headLamp);

// Lightweight procedural sky dome (single draw call). This is intentionally
// simple so it remains cheap for scale/headless workloads.
const skyUniforms = {
  uTop: { value: new THREE.Color("#7aa9ff") },
  uHorizon: { value: new THREE.Color("#cfe5ff") },
  uBottom: { value: new THREE.Color("#f4f8ff") },
  uBrightness: { value: 1.0 },
  uSoftness: { value: 1.35 },
  uSunStrength: { value: 0.18 },
  uSunDir: { value: new THREE.Vector3(0, 0.45, -1).normalize() },
};
const skyDome = new THREE.Mesh(
  new THREE.SphereGeometry(220, 24, 16),
  new THREE.ShaderMaterial({
    uniforms: skyUniforms,
    side: THREE.BackSide,
    depthWrite: false,
    vertexShader: `
      varying vec3 vWorldDir;
      void main() {
        vec4 worldPos = modelMatrix * vec4(position, 1.0);
        vWorldDir = normalize(worldPos.xyz - cameraPosition);
        gl_Position = projectionMatrix * viewMatrix * worldPos;
      }
    `,
    fragmentShader: `
      varying vec3 vWorldDir;
      uniform vec3 uTop;
      uniform vec3 uHorizon;
      uniform vec3 uBottom;
      uniform float uBrightness;
      uniform float uSoftness;
      uniform float uSunStrength;
      uniform vec3 uSunDir;
      void main() {
        float h = clamp(vWorldDir.y * 0.5 + 0.5, 0.0, 1.0);
        float shaped = pow(h, max(0.15, uSoftness));
        vec3 col = mix(uBottom, uHorizon, smoothstep(0.0, 0.55, shaped));
        col = mix(col, uTop, smoothstep(0.45, 1.0, shaped));
        float sun = pow(max(dot(normalize(vWorldDir), normalize(uSunDir)), 0.0), 220.0);
        col += vec3(1.0, 0.92, 0.78) * sun * uSunStrength;
        gl_FragColor = vec4(col * uBrightness, 1.0);
      }
    `,
  })
);
skyDome.frustumCulled = false;
skyDome.renderOrder = -1000;
skyDome.visible = false;
skyDome.userData.engineInternal = true;
scene.add(skyDome);


// Registry of built-in scene lights so the editor can expose them
const sceneLights = [
  { id: "_ambient",  label: "Ambient",     obj: ambientLight, type: "ambient" },
  { id: "_hemi",     label: "Hemisphere",   obj: hemi,         type: "hemisphere" },
  { id: "_dir",      label: "Directional",  obj: dir,          type: "directional" },
  { id: "_headlamp", label: "Head Lamp",    obj: headLamp,     type: "point" },
  { id: "_sky",      label: "Sky",          obj: skyDome,      type: "sky" },
];
scene.add(camera);

// Avatar: simple capsule that follows the first-person camera.
const avatar = new THREE.Mesh(
  new THREE.CapsuleGeometry(PLAYER_RADIUS * 0.8, PLAYER_HALF_HEIGHT * 2.0, 6, 12),
  new THREE.MeshStandardMaterial({ color: 0x7cc4ff, roughness: 0.5 })
);
avatar.castShadow = false;
avatar.receiveShadow = false;
avatar.visible = false; // always hidden; physics capsule handles collision
avatar.userData.engineInternal = true;
tagsGroup.userData.engineInternal = true;
assetsGroup.userData.engineInternal = true;
primitivesGroup.userData.engineInternal = true;
lightsGroup.userData.engineInternal = true;
scene.add(avatar);
scene.add(tagsGroup);
scene.add(assetsGroup);
scene.add(primitivesGroup);
scene.add(lightsGroup);


// -----------------------------------------------------------------------------
// Sim sensor view modes (deterministic + lightweight)
// -----------------------------------------------------------------------------
const DEFAULT_SCENE_BG = new THREE.Color(0x06070a);
const RGBD_BG = new THREE.Color(0x000000);
function applySceneSkySettings() {
  const s = normalizeSceneSettings(sceneSettings).sky;
  sceneSettings.sky = s;
  skyUniforms.uTop.value.set(s.topColor);
  skyUniforms.uHorizon.value.set(s.horizonColor);
  skyUniforms.uBottom.value.set(s.bottomColor);
  skyUniforms.uBrightness.value = s.brightness;
  skyUniforms.uSoftness.value = s.softness;
  skyUniforms.uSunStrength.value = s.sunStrength;
  skyUniforms.uSunDir.value.set(0, s.sunHeight, -1).normalize();
}
function applySceneRgbBackground() {
  if (sceneSettings.sky.enabled) {
    skyDome.visible = true;
    scene.background = null;
  } else {
    skyDome.visible = false;
    scene.background = DEFAULT_SCENE_BG;
  }
}
applySceneSkySettings();
// RGB-D visualization range tuned for indoor robotics scenes (meters).
const RGBD_MIN_DEPTH_M = 0.2;
const RGBD_MAX_DEPTH_M = 12.0;
const RGBD_AUTO_PERCENTILE_LOW = 0.05;
const RGBD_AUTO_PERCENTILE_HIGH = 0.95;
const RGBD_AUTO_RANGE_UPDATE_MS = 250;
const RGBD_AUTO_RANGE_SMOOTH = 0.2;
const RGBD_CLEAR_ALPHA = 1.0;
rgbdRangeMinM = RGBD_MIN_DEPTH_M;
rgbdRangeMaxM = RGBD_MAX_DEPTH_M;
const _rgbdSize = new THREE.Vector2(
  Math.max(1, Math.floor(window.innerWidth * renderer.getPixelRatio())),
  Math.max(1, Math.floor(window.innerHeight * renderer.getPixelRatio()))
);
const rgbdDepthTarget = new THREE.WebGLRenderTarget(_rgbdSize.x, _rgbdSize.y, {
  minFilter: THREE.NearestFilter,
  magFilter: THREE.NearestFilter,
  format: THREE.RGBAFormat,
  type: THREE.UnsignedByteType,
  depthBuffer: true,
  stencilBuffer: false,
});
rgbdDepthTarget.texture.generateMipmaps = false;
rgbdDepthTarget.depthTexture = new THREE.DepthTexture(_rgbdSize.x, _rgbdSize.y, THREE.UnsignedIntType);
rgbdDepthTarget.depthTexture.minFilter = THREE.NearestFilter;
rgbdDepthTarget.depthTexture.magFilter = THREE.NearestFilter;
rgbdDepthTarget.depthTexture.generateMipmaps = false;

// RGB-D debug material (planar forward-axis depth from view-space z).
// Kept only for debugging and no longer used as default RGB-D output.
const rgbdPlanarDepthDebugMaterial = new THREE.ShaderMaterial({
  uniforms: {
    uMinDepth: { value: RGBD_MIN_DEPTH_M },
    uMaxDepth: { value: RGBD_MAX_DEPTH_M },
  },
  vertexShader: `
    varying float vLinearDepth;
    void main() {
      vec4 mv = modelViewMatrix * vec4(position, 1.0);
      vLinearDepth = -mv.z;
      gl_Position = projectionMatrix * mv;
    }
  `,
  fragmentShader: `
    varying float vLinearDepth;
    uniform float uMinDepth;
    uniform float uMaxDepth;
    void main() {
      // Blend linear + inverse depth for strong near-range sensitivity while
      // preserving metric ordering (deterministic, no auto-exposure).
      float d = clamp(vLinearDepth, uMinDepth, uMaxDepth);
      float lin = (d - uMinDepth) / max(0.0001, (uMaxDepth - uMinDepth)); // 0 near, 1 far
      float inv = (1.0 / d - 1.0 / uMaxDepth) / max(0.0001, (1.0 / uMinDepth - 1.0 / uMaxDepth)); // 1 near, 0 far
      float t = clamp(0.35 * (1.0 - lin) + 0.65 * inv, 0.0, 1.0); // near -> 1, far -> 0

      // High-contrast pseudo-color ramp (near cyan/green, far orange/red)
      vec3 nearC = vec3(0.05, 0.98, 0.98);
      vec3 midC  = vec3(0.40, 0.95, 0.10);
      vec3 farC  = vec3(0.98, 0.15, 0.05);
      vec3 c = (t > 0.5) ? mix(midC, nearC, (t - 0.5) * 2.0) : mix(farC, midC, t * 2.0);
      gl_FragColor = vec4(c, 1.0);
    }
  `,
});
rgbdPlanarDepthDebugMaterial.toneMapped = false;

// Fullscreen passes:
// 1) reconstruct metric camera-space Z into a float render target
// 2) visualize that metric depth for display
const rgbdPostCamera = new THREE.OrthographicCamera(-1, 1, 1, -1, 0, 1);
const rgbdMetricUsesR32F = renderer.capabilities.isWebGL2 && !!renderer.extensions.get("EXT_color_buffer_float");
const rgbdMetricTargetType = rgbdMetricUsesR32F ? THREE.FloatType : THREE.HalfFloatType;
const rgbdMetricTarget = new THREE.WebGLRenderTarget(_rgbdSize.x, _rgbdSize.y, {
  minFilter: THREE.NearestFilter,
  magFilter: THREE.NearestFilter,
  format: rgbdMetricUsesR32F ? THREE.RedFormat : THREE.RGBAFormat,
  type: rgbdMetricTargetType,
  depthBuffer: false,
  stencilBuffer: false,
});
if (rgbdMetricUsesR32F) rgbdMetricTarget.texture.internalFormat = "R32F";
rgbdMetricTarget.texture.generateMipmaps = false;

const rgbdMetricScene = new THREE.Scene();
const rgbdMetricMaterial = new THREE.ShaderMaterial({
  uniforms: {
    uDepthTex: { value: rgbdDepthTarget.depthTexture },
    uNear: { value: camera.near },
    uFar: { value: camera.far },
    uMinDepth: { value: rgbdRangeMinM },
    uMaxDepth: { value: rgbdRangeMaxM },
    uNoiseEnabled: { value: 0.0 },
    uSpeckleEnabled: { value: 0.0 },
  },
  vertexShader: `
    varying vec2 vUv;
    void main() {
      vUv = uv;
      gl_Position = vec4(position.xy, 0.0, 1.0);
    }
  `,
  fragmentShader: `
    varying vec2 vUv;
    uniform sampler2D uDepthTex;
    uniform float uNear;
    uniform float uFar;
    uniform float uMinDepth;
    uniform float uMaxDepth;
    uniform float uNoiseEnabled;
    uniform float uSpeckleEnabled;

    // Perspective depth [0,1] -> view-space z (negative in front of camera).
    float perspectiveDepthToViewZ(const in float depth, const in float near, const in float far) {
      return (near * far) / ((far - near) * depth - far);
    }

    float hash12(vec2 p) {
      vec3 p3 = fract(vec3(p.xyx) * 0.1031);
      p3 += dot(p3, p3.yzx + 33.33);
      return fract((p3.x + p3.y) * p3.z);
    }

    void main() {
      float depth01 = texture2D(uDepthTex, vUv).x;
      // No geometry hit: treat as max range.
      if (depth01 >= 0.999999) {
        gl_FragColor = vec4(uMaxDepth, uMaxDepth, uMaxDepth, 1.0);
        return;
      }

      float viewZ = perspectiveDepthToViewZ(depth01, uNear, uFar);
      float zMetric = -viewZ; // camera-space Z in meters (robotics back-projection convention)
      float d = clamp(zMetric, uMinDepth, uMaxDepth);

      if (uNoiseEnabled > 0.5) {
        float span = max(0.0001, uMaxDepth - uMinDepth);
        float t = clamp((d - uMinDepth) / span, 0.0, 1.0);
        // Quantization: ~1mm near, up to ~8mm far (indoors).
        float q = mix(0.001, 0.008, t * t);
        d = floor(d / q + 0.5) * q;

        // Dropouts: more likely on edges and farther range.
        float edge = clamp(length(vec2(dFdx(depth01), dFdy(depth01))) * 250.0, 0.0, 1.0);
        float pDrop = 0.01 + 0.08 * t * t + 0.18 * edge;
        float u = hash12(vUv * vec2(4096.0, 4096.0));
        if (u < pDrop) {
          gl_FragColor = vec4(uMaxDepth, uMaxDepth, uMaxDepth, 1.0);
          return;
        }

        // Optional speckle noise (small multiplicative perturbation).
        if (uSpeckleEnabled > 0.5) {
          float n = hash12(vUv * vec2(8192.0, 8192.0) + vec2(17.3, 9.1)) - 0.5;
          float amp = 0.002 + 0.01 * t; // 2mm near -> 12mm far
          d = clamp(d + n * amp, uMinDepth, uMaxDepth);
        }
      }

      gl_FragColor = vec4(d, d, d, 1.0);
    }
  `,
  depthTest: false,
  depthWrite: false,
});
rgbdMetricMaterial.toneMapped = false;
const rgbdMetricQuad = new THREE.Mesh(new THREE.PlaneGeometry(2, 2), rgbdMetricMaterial);
rgbdMetricScene.add(rgbdMetricQuad);

const rgbdVizScene = new THREE.Scene();
const rgbdVizMaterial = new THREE.ShaderMaterial({
  uniforms: {
    uMetricDepthTex: { value: rgbdMetricTarget.texture },
    uMinDepth: { value: rgbdRangeMinM },
    uMaxDepth: { value: rgbdRangeMaxM },
    uGrayMode: { value: 0.0 },
  },
  vertexShader: `
    varying vec2 vUv;
    void main() {
      vUv = uv;
      gl_Position = vec4(position.xy, 0.0, 1.0);
    }
  `,
  fragmentShader: `
    varying vec2 vUv;
    uniform sampler2D uMetricDepthTex;
    uniform float uMinDepth;
    uniform float uMaxDepth;
    uniform float uGrayMode;
    void main() {
      float d = texture2D(uMetricDepthTex, vUv).r;
      d = clamp(d, uMinDepth, uMaxDepth);
      float lin = (d - uMinDepth) / max(0.0001, (uMaxDepth - uMinDepth)); // 0 near, 1 far
      if (uGrayMode > 0.5) {
        float g = 1.0 - lin;
        gl_FragColor = vec4(g, g, g, 1.0);
        return;
      }
      float inv = (1.0 / d - 1.0 / uMaxDepth) / max(0.0001, (1.0 / uMinDepth - 1.0 / uMaxDepth)); // 1 near, 0 far
      float t = clamp(0.35 * (1.0 - lin) + 0.65 * inv, 0.0, 1.0); // near -> 1, far -> 0
      vec3 nearC = vec3(0.05, 0.98, 0.98);
      vec3 midC  = vec3(0.40, 0.95, 0.10);
      vec3 farC  = vec3(0.98, 0.15, 0.05);
      vec3 c = (t > 0.5) ? mix(midC, nearC, (t - 0.5) * 2.0) : mix(farC, midC, t * 2.0);
      gl_FragColor = vec4(c, 1.0);
    }
  `,
  depthTest: false,
  depthWrite: false,
});
rgbdVizMaterial.toneMapped = false;
const rgbdVizQuad = new THREE.Mesh(new THREE.PlaneGeometry(2, 2), rgbdVizMaterial);
rgbdVizScene.add(rgbdVizQuad);
let _savedOverrideMaterial = null;


let _rgbdNearFarAsserted = false;
let _rgbdLastAutoRangeMs = 0;

function updateRgbdRangeLabels() {
  if (simRgbdMinValEl) simRgbdMinValEl.textContent = `${rgbdRangeMinM.toFixed(1)}m`;
  if (simRgbdMaxValEl) simRgbdMaxValEl.textContent = `${rgbdRangeMaxM.toFixed(1)}m`;
}

function setRgbdRange(minD, maxD) {
  const lo = Math.max(0.05, Math.min(minD, maxD - 0.05));
  const hi = Math.max(lo + 0.05, maxD);
  rgbdRangeMinM = lo;
  rgbdRangeMaxM = hi;
  rgbdMetricMaterial.uniforms.uMinDepth.value = lo;
  rgbdMetricMaterial.uniforms.uMaxDepth.value = hi;
  rgbdVizMaterial.uniforms.uMinDepth.value = lo;
  rgbdVizMaterial.uniforms.uMaxDepth.value = hi;
  if (simRgbdMinEl) simRgbdMinEl.value = lo.toFixed(1);
  if (simRgbdMaxEl) simRgbdMaxEl.value = hi.toFixed(1);
  updateRgbdRangeLabels();
}

setRgbdRange(RGBD_MIN_DEPTH_M, RGBD_MAX_DEPTH_M);

function percentileFromSorted(sorted, p) {
  if (!sorted.length) return 0;
  const idx = Math.min(sorted.length - 1, Math.max(0, Math.floor(p * (sorted.length - 1))));
  return sorted[idx];
}

function updateRgbdAutoRangeFromMetricTarget() {
  const now = performance.now();
  if (now - _rgbdLastAutoRangeMs < RGBD_AUTO_RANGE_UPDATE_MS) return;
  _rgbdLastAutoRangeMs = now;
  const depth = readRgbdMetricDepthFrameMeters();
  if (!depth || depth.length === 0) return;
  const samples = [];
  const stride = Math.max(1, Math.floor(depth.length / 5000));
  for (let i = 0; i < depth.length; i += stride) {
    const d = depth[i];
    if (!Number.isFinite(d)) continue;
    if (d <= RGBD_MIN_DEPTH_M || d >= RGBD_MAX_DEPTH_M) continue;
    samples.push(d);
  }
  if (samples.length < 32) return;
  samples.sort((a, b) => a - b);
  const p05 = percentileFromSorted(samples, RGBD_AUTO_PERCENTILE_LOW);
  const p95 = percentileFromSorted(samples, RGBD_AUTO_PERCENTILE_HIGH);
  const targetMin = Math.max(RGBD_MIN_DEPTH_M, Math.min(p05, p95 - 0.1));
  const targetMax = Math.min(RGBD_MAX_DEPTH_M, Math.max(p95, targetMin + 0.1));
  const smoothMin = rgbdRangeMinM + (targetMin - rgbdRangeMinM) * RGBD_AUTO_RANGE_SMOOTH;
  const smoothMax = rgbdRangeMaxM + (targetMax - rgbdRangeMaxM) * RGBD_AUTO_RANGE_SMOOTH;
  setRgbdRange(smoothMin, smoothMax);
}

function renderRgbdView(enableAutoRange = true) {
  renderRgbdMetricPassOffscreen();

  if (enableAutoRange && rgbdAutoRange) updateRgbdAutoRangeFromMetricTarget();
  rgbdVizMaterial.uniforms.uGrayMode.value = rgbdVizMode === "gray" ? 1.0 : 0.0;

  // Pass 3: visualize metric depth target.
  renderer.setRenderTarget(null);
  renderer.setClearColor(RGBD_BG, RGBD_CLEAR_ALPHA);
  renderer.clear(true, true, true);
  renderer.render(rgbdVizScene, rgbdPostCamera);
}

function renderRgbdMetricPassOffscreen(overrideCamera) {
  const cam = overrideCamera || camera;
  rgbdMetricMaterial.uniforms.uNear.value = cam.near;
  rgbdMetricMaterial.uniforms.uFar.value = cam.far;
  rgbdMetricMaterial.uniforms.uNoiseEnabled.value = rgbdNoiseEnabled ? 1.0 : 0.0;
  rgbdMetricMaterial.uniforms.uSpeckleEnabled.value = rgbdSpeckleEnabled ? 1.0 : 0.0;
  if (!_rgbdNearFarAsserted && !overrideCamera) {
    console.assert(
      Math.abs(rgbdMetricMaterial.uniforms.uNear.value - camera.near) < 1e-9 &&
      Math.abs(rgbdMetricMaterial.uniforms.uFar.value - camera.far) < 1e-9,
      "[RGB-D] Reconstruction near/far must match active camera near/far."
    );
    _rgbdNearFarAsserted = true;
  }

  // Ensure depth pass sees scene geometry, not lidar/overlay debug points.
  const savedOverride = scene.overrideMaterial;
  const savedAssets = assetsGroup.visible;
  const savedPrims = primitivesGroup.visible;
  const savedLights = lightsGroup.visible;
  const savedTags = tagsGroup.visible;
  const savedLidarViz = lidarVizGroup.visible;

  scene.overrideMaterial = null;
  assetsGroup.visible = true;
  primitivesGroup.visible = true;
  lightsGroup.visible = true;
  tagsGroup.visible = false;
  lidarVizGroup.visible = false;

  renderer.setRenderTarget(rgbdDepthTarget);
  renderer.setClearColor(0x000000, RGBD_CLEAR_ALPHA);
  renderer.clear(true, true, true);
  renderer.render(scene, cam);

  renderer.setRenderTarget(rgbdMetricTarget);
  renderer.setClearColor(0x000000, RGBD_CLEAR_ALPHA);
  renderer.clear(true, true, true);
  renderer.render(rgbdMetricScene, rgbdPostCamera);

  scene.overrideMaterial = savedOverride;
  assetsGroup.visible = savedAssets;
  primitivesGroup.visible = savedPrims;
  lightsGroup.visible = savedLights;
  tagsGroup.visible = savedTags;
  lidarVizGroup.visible = savedLidarViz;
}

function halfToFloat(h) {
  const s = (h & 0x8000) >> 15;
  const e = (h & 0x7c00) >> 10;
  const f = h & 0x03ff;
  if (e === 0) return (s ? -1 : 1) * Math.pow(2, -14) * (f / 1024);
  if (e === 31) return f ? NaN : ((s ? -1 : 1) * Infinity);
  return (s ? -1 : 1) * Math.pow(2, e - 15) * (1 + f / 1024);
}

function readRgbdMetricDepthFrameMeters() {
  const w = rgbdMetricTarget.width;
  const h = rgbdMetricTarget.height;
  if (!w || !h) return null;

  if (rgbdMetricUsesR32F) {
    const depth = new Float32Array(w * h);
    renderer.readRenderTargetPixels(rgbdMetricTarget, 0, 0, w, h, depth);
    return depth;
  }

  if (rgbdMetricTarget.texture.type === THREE.FloatType) {
    const raw = new Float32Array(w * h * 4);
    renderer.readRenderTargetPixels(rgbdMetricTarget, 0, 0, w, h, raw);
    const depth = new Float32Array(w * h);
    for (let i = 0; i < w * h; i++) depth[i] = raw[i * 4 + 0];
    return depth;
  }

  // Half-float fallback (WebGL1 / constrained platforms)
  const raw = new Uint16Array(w * h * 4);
  renderer.readRenderTargetPixels(rgbdMetricTarget, 0, 0, w, h, raw);
  const depth = new Float32Array(w * h);
  for (let i = 0; i < w * h; i++) depth[i] = halfToFloat(raw[i * 4 + 0]);
  return depth;
}


// -----------------------------------------------------------------------------
// RoboVal standardized LiDAR schema + sensor model
// -----------------------------------------------------------------------------
// We use lidar->world pose convention for pose_T_world_lidar (T_w_l).
// i.e. p_world = T_w_l * p_lidar
// Livox Mid-360 sensor model (non-repetitive Fibonacci scan pattern)
const LIDAR_SCAN_DURATION_S = 0.1; // 10 Hz scan rate
const LIDAR_NUM_POINTS = 10000;    // points per scan
const LIDAR_MAX_POINTS = LIDAR_NUM_POINTS;
const LIDAR_MIN_RANGE_M = 0.1;     // Mid-360: 0.1m min
const LIDAR_MAX_RANGE_M = 5;
const LIDAR_V_MIN_RAD = THREE.MathUtils.degToRad(-30);  // sees ground ~0.6m from robot
const LIDAR_V_MAX_RAD = THREE.MathUtils.degToRad(15);   // 15° up avoids ceiling, focuses rays on walls/ground
// Legacy constants kept for browser UI range image (not used by dimos path)
const LIDAR_NUM_RINGS = 1;
const LIDAR_RANGE_IMAGE_W = 1;
let _lidarScanCount = 0;

// Pre-compute Fibonacci sphere ray directions (uniform sampling on spherical cap)
const _fibLidarDirs = (() => {
  const golden = (1 + Math.sqrt(5)) / 2;
  const zMin = Math.sin(LIDAR_V_MIN_RAD); // sin(-7°) ≈ -0.122
  const zMax = Math.sin(LIDAR_V_MAX_RAD); // sin(52°) ≈ 0.788
  const dirs = new Float32Array(LIDAR_NUM_POINTS * 3);
  for (let i = 0; i < LIDAR_NUM_POINTS; i++) {
    const z = zMin + (zMax - zMin) * (i + 0.5) / LIDAR_NUM_POINTS;
    const r = Math.sqrt(1 - z * z);
    const phi = 2 * Math.PI * i / golden;
    dirs[i * 3 + 0] = r * Math.cos(phi); // x (forward in FLU)
    dirs[i * 3 + 1] = r * Math.sin(phi); // y (left in FLU)
    dirs[i * 3 + 2] = z;                  // z (up in FLU)
  }
  return dirs;
})();
const LIDAR_ACCUM_FRAMES = 50;
const LIDAR_STATS_INTERVAL_MS = 1500;
const LIDAR_ACCUM_MIN_TRANSLATION_M = 0.08;
const LIDAR_ACCUM_MIN_ROT_DEG = 1.5;
const LIDAR_ACCUM_REFRESH_S = 2.0;

// Lidar frame uses FLU convention:
// x=forward, y=left, z=up (right-handed). Camera local is x=right, y=up, z=back.
const _lidarToCamQuat = (() => {
  const m = new THREE.Matrix4().set(
    0, -1, 0, 0,
    0, 0, 1, 0,
    -1, 0, 0, 0,
    0, 0, 0, 1
  );
  return new THREE.Quaternion().setFromRotationMatrix(m);
})();

// Pose history for deskew (camera used as lidar pose proxy)
const _lidarPoseHistory = []; // [{stampNs, pos:Vector3, quat:Quaternion}]
const LIDAR_POSE_HISTORY_NS = 2_000_000_000; // keep ~2s history
let _lidarLastStatsMs = 0;
let _lidarUseKnownGoodDebugCloud = false;

function nowNs() {
  // Use unix epoch in ns consistently (browser clock based).
  return Math.floor(performance.timeOrigin * 1e6 + performance.now() * 1e6);
}

function pushLidarPoseSample(stampNs = nowNs()) {
  let pos, quat;
  const dimosAgent = dimosMode && window.__dimosAgent;
  if (dimosAgent) {
    // In dimos mode, sample from the agent's body position + orientation.
    // getPosition() returns capsule center (~0.37m above ground), so subtract
    // capsule half-extent to get ground level, then add mount height.
    const [ax, ay, az] = dimosAgent.getPosition?.() || [0, 0, 0];
    const groundY = ay - (PLAYER_HALF_HEIGHT + PLAYER_RADIUS);
    const lidarY = groundY + LIDAR_MOUNT_HEIGHT;
    pos = new THREE.Vector3(ax, lidarY, az);
    const yaw = window.__dimosYaw ?? dimosAgent.group?.rotation?.y ?? 0;
    const agentQuat = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), yaw);
    quat = agentQuat.multiply(_lidarToCamQuat);
  } else {
    pos = camera.getWorldPosition(new THREE.Vector3());
    const camQuat = camera.getWorldQuaternion(new THREE.Quaternion());
    quat = camQuat.clone().multiply(_lidarToCamQuat);
  }
  _lidarPoseHistory.push({ stampNs, pos, quat });
  const minNs = stampNs - LIDAR_POSE_HISTORY_NS;
  while (_lidarPoseHistory.length > 2 && _lidarPoseHistory[0].stampNs < minNs) {
    _lidarPoseHistory.shift();
  }
}

function getLidarPoseAtNs(stampNs) {
  if (_lidarPoseHistory.length === 0) {
    const camQuat = camera.getWorldQuaternion(new THREE.Quaternion());
    return {
      pos: camera.getWorldPosition(new THREE.Vector3()),
      quat: camQuat.multiply(_lidarToCamQuat),
    };
  }
  if (_lidarPoseHistory.length === 1) {
    return {
      pos: _lidarPoseHistory[0].pos.clone(),
      quat: _lidarPoseHistory[0].quat.clone(),
    };
  }
  // Find bounding samples
  let i1 = 0;
  while (i1 < _lidarPoseHistory.length && _lidarPoseHistory[i1].stampNs < stampNs) i1++;
  if (i1 <= 0) {
    return {
      pos: _lidarPoseHistory[0].pos.clone(),
      quat: _lidarPoseHistory[0].quat.clone(),
    };
  }
  if (i1 >= _lidarPoseHistory.length) {
    const last = _lidarPoseHistory[_lidarPoseHistory.length - 1];
    return { pos: last.pos.clone(), quat: last.quat.clone() };
  }
  const a = _lidarPoseHistory[i1 - 1];
  const b = _lidarPoseHistory[i1];
  const alpha = (stampNs - a.stampNs) / Math.max(1, b.stampNs - a.stampNs);
  const pos = a.pos.clone().lerp(b.pos, alpha);
  const quat = a.quat.clone().slerp(b.quat, alpha);
  return { pos, quat };
}

function composeTwlFlat64(pos, quat) {
  const m = new THREE.Matrix4().compose(pos, quat, new THREE.Vector3(1, 1, 1));
  const e = m.elements;
  // Return row-major 4x4 flattened float64 (explicitly for stable downstream use)
  return new Float64Array([
    e[0], e[4], e[8], e[12],
    e[1], e[5], e[9], e[13],
    e[2], e[6], e[10], e[14],
    e[3], e[7], e[11], e[15],
  ]);
}

function twlInverseMatrix(pos, quat) {
  const twl = new THREE.Matrix4().compose(pos, quat, new THREE.Vector3(1, 1, 1));
  return twl.clone().invert();
}


function makeRoboValLidarFrame({
  frameId,
  stampNs,
  points,
  intensity,
  ring,
  t,
  hasRing,
  hasPerPointTime,
  scanDurationS,
  poseTWorldLidar,
}) {
  // RoboValLidarFrame schema (used across sim/export/eval)
  return {
    frame_id: frameId,
    stamp_ns: stampNs,
    points, // Float32Array length N*3 (xyz meters, lidar frame)
    intensity, // Float32Array length N
    ring, // Uint16Array length N
    t, // Float32Array length N (seconds from start of scan)
    has_ring: hasRing,
    has_per_point_time: hasPerPointTime,
    scan_duration_s: scanDurationS,
    pose_T_world_lidar: poseTWorldLidar, // Float64Array length 16, row-major
  };
}

// ROS2 PointField datatype constants:
// INT8=1, UINT8=2, INT16=3, UINT16=4, INT32=5, UINT32=6, FLOAT32=7, FLOAT64=8
function to_pointcloud2(frame) {
  const n = Math.floor((frame.points?.length || 0) / 3);
  const pointStep = 22; // x,y,z,float32(12) + intensity,float32(4) + ring,uint16(2) + t,float32(4)
  const data = new Uint8Array(n * pointStep);
  const dv = new DataView(data.buffer);
  for (let i = 0; i < n; i++) {
    const o = i * pointStep;
    dv.setFloat32(o + 0, frame.points[i * 3 + 0], true);
    dv.setFloat32(o + 4, frame.points[i * 3 + 1], true);
    dv.setFloat32(o + 8, frame.points[i * 3 + 2], true);
    dv.setFloat32(o + 12, frame.intensity[i] ?? 0, true);
    dv.setUint16(o + 16, frame.ring[i] ?? 0, true);
    dv.setFloat32(o + 18, frame.t[i] ?? 0, true);
  }
  return {
    header: {
      frame_id: frame.frame_id,
      stamp: {
        sec: Math.floor(frame.stamp_ns / 1e9),
        nanosec: Math.floor(frame.stamp_ns % 1e9),
      },
    },
    height: 1,
    width: n,
    fields: [
      { name: "x", offset: 0, datatype: 7, count: 1 },
      { name: "y", offset: 4, datatype: 7, count: 1 },
      { name: "z", offset: 8, datatype: 7, count: 1 },
      { name: "intensity", offset: 12, datatype: 7, count: 1 },
      { name: "ring", offset: 16, datatype: 4, count: 1 },
      { name: "t", offset: 18, datatype: 7, count: 1 },
    ],
    is_bigendian: false,
    point_step: pointStep,
    row_step: pointStep * n,
    data,
    is_dense: true,
  };
}

function toNpyBytes(typedArray, shape, descr) {
  // NPY v1.0
  const magic = new Uint8Array([0x93, 0x4e, 0x55, 0x4d, 0x50, 0x59, 0x01, 0x00]);
  const shapeStr = `(${shape.join(", ")}${shape.length === 1 ? "," : ""})`;
  let header = `{'descr': '${descr}', 'fortran_order': False, 'shape': ${shapeStr}, }`;
  // Pad so (magic+2-byte-len+header+\n) % 16 == 0
  const preamble = 10;
  const base = preamble + header.length + 1;
  const pad = (16 - (base % 16)) % 16;
  header = header + " ".repeat(pad) + "\n";
  const headerBytes = new TextEncoder().encode(header);
  const out = new Uint8Array(magic.length + 2 + headerBytes.length + typedArray.byteLength);
  out.set(magic, 0);
  const dv = new DataView(out.buffer);
  dv.setUint16(magic.length, headerBytes.length, true);
  out.set(headerBytes, magic.length + 2);
  out.set(new Uint8Array(typedArray.buffer, typedArray.byteOffset, typedArray.byteLength), magic.length + 2 + headerBytes.length);
  return out;
}

function makeZipStore(entries) {
  // Uncompressed ZIP (store) writer for deterministic byte output ordering.
  const enc = new TextEncoder();
  const localParts = [];
  const centralParts = [];
  let offset = 0;
  const files = [];
  const crcTable = (() => {
    const t = new Uint32Array(256);
    for (let i = 0; i < 256; i++) {
      let c = i;
      for (let k = 0; k < 8; k++) c = (c & 1) ? (0xedb88320 ^ (c >>> 1)) : (c >>> 1);
      t[i] = c >>> 0;
    }
    return t;
  })();
  const crc32 = (u8) => {
    let c = 0xffffffff;
    for (let i = 0; i < u8.length; i++) c = crcTable[(c ^ u8[i]) & 0xff] ^ (c >>> 8);
    return (c ^ 0xffffffff) >>> 0;
  };

  for (const e of entries) {
    const nameBytes = enc.encode(e.name);
    const data = e.data;
    const crc = crc32(data);
    const lfh = new Uint8Array(30 + nameBytes.length);
    const dv = new DataView(lfh.buffer);
    dv.setUint32(0, 0x04034b50, true);
    dv.setUint16(4, 20, true);
    dv.setUint16(6, 0, true);
    dv.setUint16(8, 0, true); // store
    dv.setUint16(10, 0, true);
    dv.setUint16(12, 0, true);
    dv.setUint32(14, crc, true);
    dv.setUint32(18, data.length, true);
    dv.setUint32(22, data.length, true);
    dv.setUint16(26, nameBytes.length, true);
    dv.setUint16(28, 0, true);
    lfh.set(nameBytes, 30);
    localParts.push(lfh, data);
    files.push({ nameBytes, crc, size: data.length, offset });
    offset += lfh.length + data.length;
  }

  let centralSize = 0;
  for (const f of files) {
    const cfh = new Uint8Array(46 + f.nameBytes.length);
    const dv = new DataView(cfh.buffer);
    dv.setUint32(0, 0x02014b50, true);
    dv.setUint16(4, 20, true);
    dv.setUint16(6, 20, true);
    dv.setUint16(8, 0, true);
    dv.setUint16(10, 0, true);
    dv.setUint16(12, 0, true);
    dv.setUint16(14, 0, true);
    dv.setUint32(16, f.crc, true);
    dv.setUint32(20, f.size, true);
    dv.setUint32(24, f.size, true);
    dv.setUint16(28, f.nameBytes.length, true);
    dv.setUint16(30, 0, true);
    dv.setUint16(32, 0, true);
    dv.setUint16(34, 0, true);
    dv.setUint16(36, 0, true);
    dv.setUint32(38, 0, true);
    dv.setUint32(42, f.offset, true);
    cfh.set(f.nameBytes, 46);
    centralParts.push(cfh);
    centralSize += cfh.length;
  }

  const eocd = new Uint8Array(22);
  const dvE = new DataView(eocd.buffer);
  dvE.setUint32(0, 0x06054b50, true);
  dvE.setUint16(4, 0, true);
  dvE.setUint16(6, 0, true);
  dvE.setUint16(8, files.length, true);
  dvE.setUint16(10, files.length, true);
  dvE.setUint32(12, centralSize, true);
  dvE.setUint32(16, offset, true);
  dvE.setUint16(20, 0, true);

  return new Blob([...localParts, ...centralParts, eocd], { type: "application/zip" });
}

function frameToNpzBlob(frame, rangeImage = null) {
  const n = Math.floor((frame.points?.length || 0) / 3);
  const xyz = toNpyBytes(frame.points, [n, 3], "<f4");
  const intensity = toNpyBytes(frame.intensity, [n], "<f4");
  const ring = toNpyBytes(frame.ring, [n], "<u2");
  const t = toNpyBytes(frame.t, [n], "<f4");
  const metadata = {
    frame_id: frame.frame_id,
    stamp_ns: frame.stamp_ns,
    scan_duration_s: frame.scan_duration_s,
    pose_T_world_lidar: Array.from(frame.pose_T_world_lidar),
    has_ring: frame.has_ring,
    has_per_point_time: frame.has_per_point_time,
  };
  const entries = [
    { name: "xyz.npy", data: xyz },
    { name: "intensity.npy", data: intensity },
    { name: "ring.npy", data: ring },
    { name: "t.npy", data: t },
    { name: "metadata.json", data: new TextEncoder().encode(JSON.stringify(metadata, null, 2)) },
  ];
  if (rangeImage) {
    entries.push(
      { name: "range.npy", data: toNpyBytes(rangeImage.range, [rangeImage.H, rangeImage.W], "<f4") },
      { name: "intensity_img.npy", data: toNpyBytes(rangeImage.intensity, [rangeImage.H, rangeImage.W], "<f4") },
      { name: "ring_index.npy", data: toNpyBytes(rangeImage.ring_index, [rangeImage.H, rangeImage.W], "<u2") },
      { name: "range_metadata.json", data: new TextEncoder().encode(JSON.stringify(rangeImage.metadata, null, 2)) },
    );
  }
  return makeZipStore(entries);
}

let _lidarLatestRawFrame = null;
let _lidarLatestDeskewedFrame = null;
let _lidarLatestRangeImage = null;
let _lidarAutoExport = false;
let _lidarFrameSeq = 0;

async function writeLidarFrameFiles(rawFrame, deskewedFrame, rangeImage = null) {
  // Browser-safe export path: deterministic filenames with sequence + stamp.
  const seq = _lidarFrameSeq++;
  const base = `lidar_${String(seq).padStart(6, "0")}_${deskewedFrame.stamp_ns}`;
  const rawBlob = frameToNpzBlob(rawFrame, null);
  const deskBlob = frameToNpzBlob(deskewedFrame, null);
  const a1 = document.createElement("a");
  a1.href = URL.createObjectURL(rawBlob);
  a1.download = `${base}_lidar_raw.npz`;
  document.body.appendChild(a1);
  a1.click();
  a1.remove();
  setTimeout(() => URL.revokeObjectURL(a1.href), 500);

  const a2 = document.createElement("a");
  a2.href = URL.createObjectURL(deskBlob);
  a2.download = `${base}_lidar_deskewed.npz`;
  document.body.appendChild(a2);
  a2.click();
  a2.remove();
  setTimeout(() => URL.revokeObjectURL(a2.href), 500);

  if (rangeImage) {
    const rBlob = makeZipStore([
      { name: "range.npy", data: toNpyBytes(rangeImage.range, [rangeImage.H, rangeImage.W], "<f4") },
      { name: "intensity.npy", data: toNpyBytes(rangeImage.intensity, [rangeImage.H, rangeImage.W], "<f4") },
      { name: "ring_index.npy", data: toNpyBytes(rangeImage.ring_index, [rangeImage.H, rangeImage.W], "<u2") },
      { name: "metadata.json", data: new TextEncoder().encode(JSON.stringify(rangeImage.metadata, null, 2)) },
    ]);
    const a3 = document.createElement("a");
    a3.href = URL.createObjectURL(rBlob);
    a3.download = `${base}_lidar_range_image.npz`;
    document.body.appendChild(a3);
    a3.click();
    a3.remove();
    setTimeout(() => URL.revokeObjectURL(a3.href), 500);
  }
}

const lidarVizGroup = new THREE.Group();
lidarVizGroup.name = "lidarVizGroup";
lidarVizGroup.visible = false;
const LIDAR_VIZ_MAX_POINTS = LIDAR_MAX_POINTS * LIDAR_ACCUM_FRAMES;
const _lidarPosArray = new Float32Array(LIDAR_VIZ_MAX_POINTS * 3);
const _lidarColArray = new Float32Array(LIDAR_VIZ_MAX_POINTS * 3);
const _lidarAccumFrames = []; // [{pos: Float32Array, col: Float32Array}]
let _lidarLastAccumPose = null; // {pos:Vector3, quat:Quaternion, stampNs:number}
const _lidarGeom = new THREE.BufferGeometry();
_lidarGeom.setAttribute("position", new THREE.BufferAttribute(_lidarPosArray, 3));
_lidarGeom.setAttribute("color", new THREE.BufferAttribute(_lidarColArray, 3));
_lidarGeom.setDrawRange(0, 0);
const _lidarMat = new THREE.PointsMaterial({
  color: 0xffffff,
  vertexColors: true,
  size: 0.03,
  sizeAttenuation: true,
  depthTest: true,
  transparent: false,
});
const _lidarPoints = new THREE.Points(_lidarGeom, _lidarMat);
_lidarPoints.frustumCulled = false; // point cloud covers entire scene; never cull
console.assert(_lidarPoints.isPoints === true, "[LiDAR] Visualization must use THREE.Points");
lidarVizGroup.add(_lidarPoints);
scene.add(lidarVizGroup);
let _lidarLastNonZeroDrawCount = 0;

let _lidarScanState = null; // incremental scan state (processed across frames)

function updateSimSensorButtons() {
  if (simViewCompareBtn) simViewCompareBtn.classList.toggle("active", simCompareView);
  if (simViewRgbdBtn) simViewRgbdBtn.classList.toggle("active", simSensorViewMode === "rgbd" && !simCompareView);
  if (simRgbdGrayBtn) simRgbdGrayBtn.classList.toggle("active", rgbdVizMode === "gray");
  if (simRgbdColormapBtn) simRgbdColormapBtn.classList.toggle("active", rgbdVizMode === "colormap");
  if (simRgbdAutoRangeBtn) simRgbdAutoRangeBtn.classList.toggle("active", rgbdAutoRange);
  if (simRgbdNoiseBtn) simRgbdNoiseBtn.classList.toggle("active", rgbdNoiseEnabled);
  if (simRgbdSpeckleBtn) simRgbdSpeckleBtn.classList.toggle("active", rgbdSpeckleEnabled);
  if (simRgbdMinEl) simRgbdMinEl.disabled = rgbdAutoRange;
  if (simRgbdMaxEl) simRgbdMaxEl.disabled = rgbdAutoRange;
  if (simViewLidarBtn) simViewLidarBtn.classList.toggle("active", simSensorViewMode === "lidar" && !lidarOrderedDebugView && !simCompareView);
  if (simLidarColorRangeBtn) simLidarColorRangeBtn.classList.toggle("active", lidarColorByRange);
  if (simLidarOrderedDebugBtn) simLidarOrderedDebugBtn.classList.toggle("active", lidarOrderedDebugView);
  if (simLidarNoiseBtn) simLidarNoiseBtn.classList.toggle("active", lidarNoiseEnabled);
  if (simLidarMultiReturnBtn) {
    simLidarMultiReturnBtn.classList.toggle("active", lidarMultiReturnMode === "last");
    simLidarMultiReturnBtn.textContent = lidarMultiReturnMode === "last" ? "LiDAR: Last Return" : "LiDAR: Strongest";
  }
  updateRgbdRangeLabels();
}

function applySimPanelCollapsedState() {
  if (!overlayEl || !agentPanelEl) return;
  const shouldCollapse = simPanelCollapsed;
  overlayEl.classList.toggle("sim-panel-collapsed", shouldCollapse);
  agentPanelEl.classList.toggle("hidden", shouldCollapse);
  simPanelOpenBtn?.classList.toggle("hidden", !shouldCollapse);
}

function lidarRangeColor01(t) {
  // Deterministic near->far gradient: cyan -> green -> yellow -> red
  const x = Math.min(1, Math.max(0, t));
  if (x < 0.33) {
    const u = x / 0.33;
    return [0.05 + 0.35 * u, 0.98, 0.98 - 0.88 * u];
  }
  if (x < 0.66) {
    const u = (x - 0.33) / 0.33;
    return [0.40 + 0.58 * u, 0.95 - 0.15 * u, 0.10 * (1.0 - u)];
  }
  const u = (x - 0.66) / 0.34;
  return [0.98, 0.80 - 0.65 * u, 0.02 + 0.03 * (1.0 - u)];
}

function lidarHash01(seed) {
  let x = seed | 0;
  x ^= x >>> 16;
  x = Math.imul(x, 0x7feb352d);
  x ^= x >>> 15;
  x = Math.imul(x, 0x846ca68b);
  x ^= x >>> 16;
  return (x >>> 0) / 4294967296;
}

function lidarGaussianNoise(seedBase) {
  // Deterministic approx N(0,1) from 6 uniforms (CLT).
  let s = 0;
  for (let i = 0; i < 6; i++) {
    s += lidarHash01(seedBase + i * 2654435761);
  }
  return s - 3.0;
}

function applyLidarRealityModel(toi, incidence, scanSeed, vi, hi) {
  let outRange = toi;
  let dropped = false;

  if (lidarNoiseEnabled) {
    // Indoor-friendly deterministic noise profile (meters).
    const sigma = 0.004 + 0.0015 * Math.max(0, toi); // ~4mm near, grows with range
    const n = lidarGaussianNoise(scanSeed ^ (vi * 73856093) ^ (hi * 19349663));
    outRange = Math.max(LIDAR_MIN_RANGE_M, Math.min(LIDAR_MAX_RANGE_M, outRange + sigma * n));

    const tr = Math.min(1, Math.max(0, toi / LIDAR_MAX_RANGE_M));
    const dropoutP = 0.005 + 0.04 * tr * tr; // deterministic, stronger at longer range
    const u = lidarHash01(scanSeed ^ (vi * 83492791) ^ (hi * 2654435761));
    if (u < dropoutP) dropped = true;
  }

  // Multi-return knob for future lidar profiles.
  // With a single physics hit, "last" is approximated as a slight farther-biased return.
  if (!dropped && lidarMultiReturnMode === "last") {
    const weakSurface = 1.0 - Math.max(0, Math.min(1, incidence));
    const tail = 0.015 * weakSurface; // up to 1.5 cm
    outRange = Math.min(LIDAR_MAX_RANGE_M, outRange + tail);
  }

  return { range: outRange, dropped };
}

function buildKnownGoodDebugCloud() {
  // Deterministic 1m cube grid centered 2m in front of camera.
  const center = new THREE.Vector3(0, 0, -2).applyMatrix4(camera.matrixWorld);
  const step = 0.1; // 11^3 ~= 1331 points
  const points = [];
  const colors = [];
  for (let x = -0.5; x <= 0.5001; x += step) {
    for (let y = -0.5; y <= 0.5001; y += step) {
      for (let z = -0.5; z <= 0.5001; z += step) {
        points.push(center.x + x, center.y + y, center.z + z);
        colors.push(0.15 + (x + 0.5) * 0.7, 0.25 + (y + 0.5) * 0.6, 0.95 - (z + 0.5) * 0.5);
      }
    }
  }
  return {
    pos: new Float32Array(points),
    col: new Float32Array(colors),
  };
}

function logLidarFrameStats(points, n, ring) {
  const now = performance.now();
  if (now - _lidarLastStatsMs < LIDAR_STATS_INTERVAL_MS) return;
  _lidarLastStatsMs = now;
  if (!n) {
    console.info("[LiDAR stats]", { n_points: 0, nan_inf_pct: 0 });
    return;
  }
  let minX = Infinity, minY = Infinity, minZ = Infinity;
  let maxX = -Infinity, maxY = -Infinity, maxZ = -Infinity;
  let ringMin = Infinity;
  let ringMax = -Infinity;
  let bad = 0;
  const yQuant = new Set();
  for (let i = 0; i < n; i++) {
    const x = points[i * 3 + 0];
    const y = points[i * 3 + 1];
    const z = points[i * 3 + 2];
    if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) {
      bad++;
      continue;
    }
    if (x < minX) minX = x;
    if (x > maxX) maxX = x;
    if (y < minY) minY = y;
    if (y > maxY) maxY = y;
    if (z < minZ) minZ = z;
    if (z > maxZ) maxZ = z;
    yQuant.add(Math.round(y * 1000));
    const rr = ring[i];
    if (rr < ringMin) ringMin = rr;
    if (rr > ringMax) ringMax = rr;
  }
  console.info("[LiDAR stats]", {
    n_points: n,
    min: { x: minX, y: minY, z: minZ },
    max: { x: maxX, y: maxY, z: maxZ },
    nan_inf_pct: (100 * bad) / n,
    unique_y_mm: yQuant.size,
    rings_configured: LIDAR_NUM_RINGS,
    ring_min: Number.isFinite(ringMin) ? ringMin : 0,
    ring_max: Number.isFinite(ringMax) ? ringMax : 0,
  });
}

function shouldAppendAccumFrame(refPose, stampNs) {
  if (!_lidarLastAccumPose) return true;
  const dtS = (stampNs - _lidarLastAccumPose.stampNs) / 1e9;
  if (dtS >= LIDAR_ACCUM_REFRESH_S) return true;
  const dp = refPose.pos.distanceTo(_lidarLastAccumPose.pos);
  if (dp >= LIDAR_ACCUM_MIN_TRANSLATION_M) return true;
  const ang = THREE.MathUtils.radToDeg(refPose.quat.angleTo(_lidarLastAccumPose.quat));
  if (ang >= LIDAR_ACCUM_MIN_ROT_DEG) return true;
  return false;
}

function resetLidarScanState() {
  _lidarScanState = null;
}

function updateLidarPointCloud() {
  if (!rapierWorld || !RAPIER || (simSensorViewMode !== "lidar" && !dimosMode)) return;

  if (_lidarUseKnownGoodDebugCloud) {
    resetLidarScanState();
    const dbg = buildKnownGoodDebugCloud();
    const nDbg = Math.min(LIDAR_VIZ_MAX_POINTS, Math.floor(dbg.pos.length / 3));
    _lidarPosArray.set(dbg.pos.subarray(0, nDbg * 3), 0);
    _lidarColArray.set(dbg.col.subarray(0, nDbg * 3), 0);
    _lidarGeom.setDrawRange(0, nDbg);
    _lidarGeom.attributes.position.needsUpdate = true;
    _lidarGeom.attributes.color.needsUpdate = true;
    lidarVizGroup.position.set(0, 0, 0);
    lidarVizGroup.quaternion.identity();
    lidarVizGroup.scale.set(1, 1, 1);
    return;
  }

  // Build set of collider handles to exclude from lidar raycasts.
  // Excludes player collider and ALL AI agent colliders (lidar origin is inside them).
  // In dimos mode, also explicitly exclude the active dimos agent body/colliders.
  const _lidarExcludeHandles = new Set();
  const _lidarHostAgent = dimosMode ? window.__dimosAgent : null;
  const _lidarExcludeRigidBodyHandle = _lidarHostAgent?.body?.handle;
  if (playerCollider) _lidarExcludeHandles.add(playerCollider.handle);
  if (_lidarHostAgent?.collider?.handle != null) _lidarExcludeHandles.add(_lidarHostAgent.collider.handle);
  if (_lidarHostAgent?.spineCollider?.handle != null) _lidarExcludeHandles.add(_lidarHostAgent.spineCollider.handle);
  for (const a of aiAgents) {
    if (a?.collider) _lidarExcludeHandles.add(a.collider.handle);
    if (a?.spineCollider) _lidarExcludeHandles.add(a.spineCollider.handle);
  }

  // Livox Mid-360 style: Fibonacci sphere sampling, incremental over ~0.1s wall-clock.
  const N = LIDAR_NUM_POINTS;
  const scanDurationS = LIDAR_SCAN_DURATION_S;
  const scanDurationNs = Math.floor(scanDurationS * 1e9);
  if (!_lidarScanState) {
    const scanStartNs = nowNs();
    _lidarScanCount++;
    const jitterAngle = _lidarScanCount * 2.399963; // golden angle rotation per scan
    const rangeImg = new Float32Array(LIDAR_NUM_RINGS * LIDAR_RANGE_IMAGE_W);
    const intenImg = new Float32Array(LIDAR_NUM_RINGS * LIDAR_RANGE_IMAGE_W);
    const ringIdxImg = new Uint16Array(LIDAR_NUM_RINGS * LIDAR_RANGE_IMAGE_W);
    _lidarScanState = {
      scanStartNs,
      scanDurationS,
      scanDurationNs,
      scanSeed: (scanStartNs / 1e6) | 0,
      cosJitter: Math.cos(jitterAngle),
      sinJitter: Math.sin(jitterAngle),
      nextIdx: 0,
      n: 0,
      rawPts: new Float32Array(LIDAR_MAX_POINTS * 3),
      deskPts: new Float32Array(LIDAR_MAX_POINTS * 3),
      intensity: new Float32Array(LIDAR_MAX_POINTS),
      ring: new Uint16Array(LIDAR_MAX_POINTS),
      tArr: new Float32Array(LIDAR_MAX_POINTS),
      worldPts: new Float32Array(LIDAR_MAX_POINTS * 3),
      colArray: new Float32Array(LIDAR_MAX_POINTS * 3),
      rangeImg,
      intenImg,
      ringIdxImg,
    };
  }
  const st = _lidarScanState;
  const dirLocal = new THREE.Vector3();
  const dirWorld = new THREE.Vector3();
  const pWorld = new THREE.Vector3();
  const pRawLocal = new THREE.Vector3();
  const elapsedNs = Math.max(0, nowNs() - st.scanStartNs);
  const progress = Math.min(1, elapsedNs / Math.max(1, st.scanDurationNs));
  let targetIdx = Math.floor(progress * N);
  targetIdx = Math.max(targetIdx, Math.min(N, st.nextIdx + 1));
  if (elapsedNs >= st.scanDurationNs) targetIdx = N;

  const cosJ = st.cosJitter, sinJ = st.sinJitter;

  for (let i = st.nextIdx; i < targetIdx; i++) {
    {
      if (st.n >= LIDAR_MAX_POINTS) break;
      // Fibonacci direction with per-scan golden-angle rotation around Z (non-repetitive)
      const fx = _fibLidarDirs[i * 3 + 0], fy = _fibLidarDirs[i * 3 + 1], fz = _fibLidarDirs[i * 3 + 2];
      const tSec = (i / Math.max(1, N - 1)) * scanDurationS;
      const stampNs = st.scanStartNs + Math.floor(tSec * 1e9);
      const pose = getLidarPoseAtNs(stampNs);
      const w2lNow = twlInverseMatrix(pose.pos, pose.quat);
      const origin = pose.pos;

      // Fibonacci direction rotated by per-scan golden angle (FLU frame)
      dirLocal.set(fx * cosJ - fy * sinJ, fx * sinJ + fy * cosJ, fz);
      dirWorld.copy(dirLocal).applyQuaternion(pose.quat).normalize();
      const ray = new RAPIER.Ray(
        { x: origin.x, y: origin.y, z: origin.z },
        { x: dirWorld.x, y: dirWorld.y, z: dirWorld.z }
      );
      let hit = null;
      let singleExcludeHandle = undefined;
      // Defensive retry: if a self-collider slips through, recast while excluding it.
      // Keeps scans alive even if exclusion bookkeeping is briefly stale.
      for (let castAttempt = 0; castAttempt < 4; castAttempt++) {
        hit = rapierWorld.queryPipeline.castRayAndGetNormal(
          rapierWorld.bodies,
          rapierWorld.colliders,
          ray,
          LIDAR_MAX_RANGE_M,
          false,
          RAPIER.QueryFilterFlags.EXCLUDE_SENSORS,
          undefined,
          singleExcludeHandle,
          _lidarExcludeRigidBodyHandle,
          (h) => !_lidarExcludeHandles.has(h)
        );
        const hitHandle = hit?.colliderHandle;
        if (!hit || hitHandle == null || !_lidarExcludeHandles.has(hitHandle)) break;
        singleExcludeHandle = hitHandle;
      }
      let toi = hit ? (hit.toi ?? hit.timeOfImpact ?? 0) : Infinity;
      const hitNormal = hit?.normal || null;

      // Ground-truth-style lidar: no-return beams are omitted.
      if (!Number.isFinite(toi) || toi > LIDAR_MAX_RANGE_M || toi < LIDAR_MIN_RANGE_M) continue;

      const nx = hitNormal?.x ?? 0;
      const ny = hitNormal?.y ?? 0;
      const nz = hitNormal?.z ?? 1;
      const incidence = hitNormal ? Math.max(0, -(dirWorld.x * nx + dirWorld.y * ny + dirWorld.z * nz)) : 0.7;
      const reality = applyLidarRealityModel(toi, incidence, st.scanSeed, i & 0xff, i >> 8);
      if (reality.dropped) continue;
      toi = reality.range;

      pWorld.set(
        origin.x + dirWorld.x * toi,
        origin.y + dirWorld.y * toi,
        origin.z + dirWorld.z * toi
      );
      pRawLocal.copy(pWorld).applyMatrix4(w2lNow);

      st.rawPts[st.n * 3 + 0] = pRawLocal.x;
      st.rawPts[st.n * 3 + 1] = pRawLocal.y;
      st.rawPts[st.n * 3 + 2] = pRawLocal.z;
      st.worldPts[st.n * 3 + 0] = pWorld.x;
      st.worldPts[st.n * 3 + 1] = pWorld.y;
      st.worldPts[st.n * 3 + 2] = pWorld.z;

      st.ring[st.n] = 0;
      st.tArr[st.n] = tSec;

      const atten = 1.0 / (1.0 + 0.02 * toi * toi);
      const I = Math.max(0.06, Math.min(1.0, incidence * atten));
      st.intensity[st.n] = I;
      const tr = Math.min(1, Math.max(0, toi / LIDAR_MAX_RANGE_M));
      const depthShade = 1.0 - 0.35 * tr; // cheap EDL-like darkening by depth/range

      if (lidarColorByRange) {
        const [r, g, b] = lidarRangeColor01(tr);
        st.colArray[st.n * 3 + 0] = r * depthShade;
        st.colArray[st.n * 3 + 1] = g * depthShade;
        st.colArray[st.n * 3 + 2] = b * depthShade;
      } else {
        // Intensity-like grayscale (closer to raw LiDAR semantics)
        const g = I * depthShade;
        st.colArray[st.n * 3 + 0] = g;
        st.colArray[st.n * 3 + 1] = g;
        st.colArray[st.n * 3 + 2] = g;
      }
      st.n++;
    }
  }
  st.nextIdx = targetIdx;
  if (st.nextIdx < N) {
    // Keep LiDAR visible while a scan is still being built.
    // If we don't have accumulated frames yet, show the partial current scan.
    if (!lidarOrderedDebugView && _lidarAccumFrames.length === 0 && st.n > 0) {
      _lidarPosArray.set(st.worldPts.subarray(0, st.n * 3), 0);
      _lidarColArray.set(st.colArray.subarray(0, st.n * 3), 0);
      _lidarGeom.setDrawRange(0, st.n);
      if (st.n > 0) _lidarLastNonZeroDrawCount = st.n;
      _lidarGeom.attributes.position.needsUpdate = true;
      _lidarGeom.attributes.color.needsUpdate = true;
      lidarVizGroup.position.set(0, 0, 0);
      lidarVizGroup.quaternion.identity();
      lidarVizGroup.scale.set(1, 1, 1);
    }
    return; // scan still in progress
  }

  const scanEndNs = st.scanStartNs + st.scanDurationNs;
  const refPose = getLidarPoseAtNs(scanEndNs);
  const refTwlFlat = composeTwlFlat64(refPose.pos, refPose.quat);
  const refW2L = twlInverseMatrix(refPose.pos, refPose.quat);
  const pDeskLocal = new THREE.Vector3();
  for (let i = 0; i < st.n; i++) {
    pDeskLocal.set(
      st.worldPts[i * 3 + 0],
      st.worldPts[i * 3 + 1],
      st.worldPts[i * 3 + 2]
    ).applyMatrix4(refW2L);
    st.deskPts[i * 3 + 0] = pDeskLocal.x;
    st.deskPts[i * 3 + 1] = pDeskLocal.y;
    st.deskPts[i * 3 + 2] = pDeskLocal.z;
  }

  logLidarFrameStats(st.worldPts, st.n, st.ring);

  const rawFrame = makeRoboValLidarFrame({
    frameId: "lidar",
    stampNs: scanEndNs,
    points: st.rawPts.subarray(0, st.n * 3),
    intensity: st.intensity.subarray(0, st.n),
    ring: st.ring.subarray(0, st.n),
    t: st.tArr.subarray(0, st.n),
    hasRing: true,
    hasPerPointTime: true,
    scanDurationS,
    poseTWorldLidar: refTwlFlat,
  });
  const deskewedFrame = makeRoboValLidarFrame({
    frameId: "lidar",
    stampNs: scanEndNs,
    points: st.deskPts.subarray(0, st.n * 3),
    intensity: st.intensity.subarray(0, st.n),
    ring: st.ring.subarray(0, st.n),
    t: st.tArr.subarray(0, st.n),
    hasRing: true,
    hasPerPointTime: true,
    scanDurationS,
    poseTWorldLidar: refTwlFlat,
  });
  const sensorModelMeta = {
    range_min_m: LIDAR_MIN_RANGE_M,
    range_max_m: LIDAR_MAX_RANGE_M,
    noise_enabled: lidarNoiseEnabled,
    multi_return_mode: lidarMultiReturnMode,
    ordered_render_debug: lidarOrderedDebugView,
    deskewed: true,
  };
  rawFrame.sensor_model = sensorModelMeta;
  deskewedFrame.sensor_model = sensorModelMeta;
  const rangeImage = {
    H: LIDAR_NUM_RINGS,
    W: LIDAR_RANGE_IMAGE_W,
    range: st.rangeImg,
    intensity: st.intenImg,
    ring_index: st.ringIdxImg,
    metadata: {
      azimuth_convention: "col increases with azimuth in lidar FLU frame",
      binning: "uniform azimuth bins",
      num_rings: LIDAR_NUM_RINGS,
      num_azimuth_bins: LIDAR_RANGE_IMAGE_W,
      sensor_model: sensorModelMeta,
      visualization_mode: lidarOrderedDebugView ? "single_sweep_ordered" : "accumulated_unordered",
      accumulation: {
        max_frames: LIDAR_ACCUM_FRAMES,
        min_translation_m: LIDAR_ACCUM_MIN_TRANSLATION_M,
        min_rotation_deg: LIDAR_ACCUM_MIN_ROT_DEG,
        refresh_s: LIDAR_ACCUM_REFRESH_S,
      },
    },
  };
  _lidarLatestRawFrame = rawFrame;
  _lidarLatestDeskewedFrame = deskewedFrame;
  _lidarLatestRangeImage = rangeImage;
  // Default visualization: accumulated world-space point cloud (depth-tested).
  if (!lidarOrderedDebugView) {
    if (shouldAppendAccumFrame(refPose, scanEndNs)) {
      const framePos = new Float32Array(st.n * 3);
      const frameCol = new Float32Array(st.n * 3);
      framePos.set(st.worldPts.subarray(0, st.n * 3));
      frameCol.set(st.colArray.subarray(0, st.n * 3));
      _lidarAccumFrames.push({ pos: framePos, col: frameCol });
      while (_lidarAccumFrames.length > LIDAR_ACCUM_FRAMES) _lidarAccumFrames.shift();
      _lidarLastAccumPose = {
        pos: refPose.pos.clone(),
        quat: refPose.quat.clone(),
        stampNs: scanEndNs,
      };
    }

    let out = 0;
    const len = _lidarAccumFrames.length;
    for (let fi = 0; fi < len && out < LIDAR_VIZ_MAX_POINTS; fi++) {
      const f = _lidarAccumFrames[fi];
      const age01 = len <= 1 ? 0 : (len - 1 - fi) / (len - 1); // 1 old -> 0 newest
      const fade = 1.0 - 0.7 * age01;
      const fn = Math.floor(f.pos.length / 3);
      for (let i = 0; i < fn && out < LIDAR_VIZ_MAX_POINTS; i++, out++) {
        _lidarPosArray[out * 3 + 0] = f.pos[i * 3 + 0];
        _lidarPosArray[out * 3 + 1] = f.pos[i * 3 + 1];
        _lidarPosArray[out * 3 + 2] = f.pos[i * 3 + 2];
        _lidarColArray[out * 3 + 0] = Math.max(0, Math.min(1, f.col[i * 3 + 0] * fade));
        _lidarColArray[out * 3 + 1] = Math.max(0, Math.min(1, f.col[i * 3 + 1] * fade));
        _lidarColArray[out * 3 + 2] = Math.max(0, Math.min(1, f.col[i * 3 + 2] * fade));
      }
    }
    if (out > 0) {
      _lidarGeom.setDrawRange(0, out);
      _lidarLastNonZeroDrawCount = out;
    }
    lidarVizGroup.position.set(0, 0, 0);
    lidarVizGroup.quaternion.identity();
    lidarVizGroup.scale.set(1, 1, 1);
  } else {
    // Debug visualization: ordered current-frame cloud in deskewed lidar frame.
    _lidarAccumFrames.length = 0;
    _lidarPosArray.set(st.deskPts.subarray(0, st.n * 3), 0);
    _lidarGeom.setDrawRange(0, st.n);
    if (st.n > 0) _lidarLastNonZeroDrawCount = st.n;
    lidarVizGroup.position.copy(refPose.pos);
    lidarVizGroup.quaternion.copy(refPose.quat);
    lidarVizGroup.scale.set(1, 1, 1);
  }
  _lidarGeom.attributes.position.needsUpdate = true;
  _lidarGeom.attributes.color.needsUpdate = true;
  // Guard against intermittent empty frames causing visible flicker.
  if (_lidarGeom.drawRange.count <= 0 && _lidarLastNonZeroDrawCount > 0) {
    _lidarGeom.setDrawRange(0, _lidarLastNonZeroDrawCount);
  }
  if (_lidarAutoExport) {
    writeLidarFrameFiles(rawFrame, deskewedFrame, rangeImage);
  }
  resetLidarScanState();
}

function applySimSensorViewMode() {
  if (simSensorViewMode === "rgb") {
    // Restore default rendering.
    scene.overrideMaterial = _savedOverrideMaterial;
    assetsGroup.visible = true;
    primitivesGroup.visible = true;
    lightsGroup.visible = true;
    tagsGroup.visible = false;
    lidarVizGroup.visible = false;
    _lidarAccumFrames.length = 0;
    _lidarLastAccumPose = null;
    resetLidarScanState();
    applySceneRgbBackground();
  } else if (simSensorViewMode === "rgbd") {
    // RGB-D mode: render scene depth to offscreen target, then post-process to
    // metric camera-space Z visualization. Do not override scene materials.
    _savedOverrideMaterial = null;
    scene.overrideMaterial = null;
    assetsGroup.visible = true;
    primitivesGroup.visible = true;
    lightsGroup.visible = true;
    tagsGroup.visible = false;
    lidarVizGroup.visible = false;
    _lidarAccumFrames.length = 0;
    _lidarLastAccumPose = null;
    resetLidarScanState();
    skyDome.visible = false;
    scene.background = RGBD_BG;
  } else {
    // LiDAR mode: hide scene visuals and render deterministic point cloud only.
    _savedOverrideMaterial = null;
    scene.overrideMaterial = null;
    assetsGroup.visible = false;
    primitivesGroup.visible = false;
    lightsGroup.visible = false;
    tagsGroup.visible = false;
    lidarVizGroup.visible = true;
    skyDome.visible = false;
    scene.background = RGBD_BG;
  }
  updateSimSensorButtons();
}

function setSimSensorViewMode(mode) {
  const next = mode === "rgbd" || mode === "lidar" ? mode : "rgb";
  // Toggle behavior: clicking an already-active sensor mode returns to RGB.
  simSensorViewMode = (simSensorViewMode === next && next !== "rgb") ? "rgb" : next;
  applySimSensorViewMode();
  if (simSensorViewMode === "rgb") {
    setStatus("RGB view");
  } else if (simSensorViewMode === "rgbd") {
    setStatus(`RGB-D ${rgbdVizMode === "gray" ? "grayscale" : "colormap"} (${rgbdRangeMinM.toFixed(1)}-${rgbdRangeMaxM.toFixed(1)}m)`);
  } else {
    setStatus(lidarOrderedDebugView ? "LiDAR single sweep view" : "LiDAR accumulated 3D point cloud");
  }
}

// Controls: pointer-lock look + WASD move.
const controls = new PointerLockControls(camera, document.body);
scene.add(controls.object);


const keys = {
  forward: false,
  backward: false,
  left: false,
  right: false,
  up: false,
  down: false,
};

function setStatus(msg) {
  if (statusEl) statusEl.textContent = msg || "";
  if (statusSimEl) statusSimEl.textContent = msg || "";
}

function randId() {
  return Math.random().toString(16).slice(2) + "-" + Date.now().toString(16);
}

function escapeHtml(s) {
  return String(s)
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;")
    .replaceAll("'", "&#039;");
}


function saveTagsForWorld() {
  try {
    let rawState = localStorage.getItem("sparkWorldStateByWorld");
    let byWorld = {};

    try {
      byWorld = rawState ? JSON.parse(rawState) : {};
    } catch {
      // Corrupted data, start fresh
      console.warn("[SAVE] Corrupted localStorage data, clearing...");
      byWorld = {};
    }

    console.log(`[SAVE] Saving ${assets.length} assets for world: ${worldKey}`);

    // Only save lightweight metadata - NOT the full dataBase64 model data
    // Only save state changes (currentStateId, transform)
    const lightweightAssets = assets.map(a => {
      // Regular assets: only save delta/metadata, not model data
      return {
        id: a.id,
        currentStateId: a.currentStateId || a.currentState,
        transform: a.transform,
        pickable: a.pickable,
        castShadow: a.castShadow ?? false,
        receiveShadow: a.receiveShadow ?? false,
        blobShadow: a.blobShadow || null,
        _deltaOnly: true,
      };
    });

    // Save primitives — strip collider handles and large texture data URLs
    // (textures are preserved in Export but too big for localStorage)
    const savePrimitives = primitives.map((p) => {
      const { _colliderHandle, ...rest } = p;
      if (rest.material?.textureDataUrl) {
        rest.material = { ...rest.material, textureDataUrl: null };
      }
      return rest;
    });

    // Save lights (strip runtime objects)
    const saveLights = editorLights.map((l) => {
      const { _lightObj, _helperObj, _proxyObj, ...rest } = l;
      return rest;
    });

    byWorld[worldKey] = {
      tags,
      assets: lightweightAssets,
      primitives: savePrimitives,
      lights: saveLights,
      groups,
      sceneSettings: serializeSceneSettings(),
    };
    const dataStr = JSON.stringify(byWorld);

    // Check size before saving (localStorage limit is typically 5MB)
    const sizeKB = (dataStr.length * 2) / 1024; // Rough estimate (UTF-16)
    console.log(`[SAVE] Data size: ${sizeKB.toFixed(1)}KB`);

    localStorage.setItem("sparkWorldStateByWorld", dataStr);
    localStorage.setItem("sparkWorldLastWorldKey", worldKey);
  } catch (e) {
    console.error("[SAVE] Failed to save world state:", e);

    // If quota exceeded, try clearing old data and retry
    if (e.name === "QuotaExceededError") {
      console.warn("[SAVE] Quota exceeded, clearing old world data...");
      try {
        localStorage.removeItem("sparkWorldStateByWorld");
        // Retry with just current world and minimal data
        const freshData = {};
        freshData[worldKey] = {
          tags,
          assets: [],
          sceneSettings: serializeSceneSettings()
        };
        localStorage.setItem("sparkWorldStateByWorld", JSON.stringify(freshData));
        console.log("[SAVE] Saved minimal data after clearing old data");
      } catch (e2) {
        console.error("[SAVE] Still failed after clearing:", e2);
      }
    }
  }
}

// Clear all localStorage data for this app (useful for debugging)
function clearWorldStorage() {
  localStorage.removeItem("sparkWorldStateByWorld");
  localStorage.removeItem("sparkWorldLastWorldKey");
  console.log("[STORAGE] Cleared all world storage");
}
// Expose for debugging: window.clearWorldStorage = clearWorldStorage;


const markerGeom = new THREE.SphereGeometry(0.08, 12, 12);
const markerMat = new THREE.MeshBasicMaterial({ color: 0x7cc4ff });
const markerMatActive = new THREE.MeshBasicMaterial({ color: 0xffd36e });
const radiusGeom = new THREE.SphereGeometry(1, 20, 14);
const radiusMat = new THREE.MeshBasicMaterial({
  color: 0x7cc4ff,
  transparent: true,
  opacity: 0.08,
  depthWrite: false,
});

function agentUiPush(event) {
  const logs = [
    agentLogEl,
  ].filter(Boolean);
  for (const log of logs) {
    const el = document.createElement("div");
    el.className = "agent-log-item";
    el.textContent = event;
    log.prepend(el);
    // cap
    while (log.children.length > 10) log.removeChild(log.lastChild);
  }
}


function agentUiSetShot(base64) {
  if (!base64) return;
  const src = `data:image/jpeg;base64,${base64}`;
  if (agentShotImgEl) agentShotImgEl.src = src;
}


function agentUiSetRequest({ endpoint, model, prompt, context, imageBytes, messages }) {
  const metaText = `endpoint: ${endpoint}\nmodel: ${model}\nimageBytes: ${imageBytes ?? "?"}\nworld: ${worldKey}`;
  if (agentReqMetaEl) agentReqMetaEl.textContent = metaText;
  if (agentReqPromptEl) agentReqPromptEl.textContent = prompt || "";

  // Format messages for display (only assistant and user messages, not system)
  let contextText = "";
  if (messages && messages.length > 0) {
    // Filter out system messages - only show assistant and user
    const conversationMessages = messages.filter(msg => msg.role !== "system");
    if (conversationMessages.length > 0) {
      contextText = conversationMessages.map((msg) => {
        const role = msg.role.toUpperCase();
        let content = "";
        if (typeof msg.content === "string") {
          content = msg.content;
        } else if (Array.isArray(msg.content)) {
          // Handle multimodal content (text + image)
          content = msg.content.map(part => {
            if (part.type === "text") return part.text;
            if (part.type === "image_url") return "[IMAGE]";
            return JSON.stringify(part);
          }).join("\n");
        } else {
          content = JSON.stringify(msg.content, null, 2);
        }
        return `═══ ${role} ═══\n${content}`;
      }).join("\n\n");
    } else {
      contextText = "(No conversation history yet)";
    }
  } else {
    contextText = JSON.stringify(context ?? {}, null, 2);
  }
  if (agentReqContextEl) agentReqContextEl.textContent = contextText;
}

function agentUiSetResponse({ raw, parsed }) {
  if (agentRespRawEl) agentRespRawEl.textContent = raw || "";
  if (agentLastEl) agentLastEl.textContent = JSON.stringify(parsed ?? {}, null, 2);
}

function clearAgentInspectorViews() {
  if (agentShotImgEl) agentShotImgEl.removeAttribute("src");
  if (agentReqMetaEl) agentReqMetaEl.textContent = "No request yet";
  if (agentReqPromptEl) agentReqPromptEl.textContent = "";
  if (agentReqContextEl) agentReqContextEl.textContent = "";
  if (agentRespRawEl) agentRespRawEl.textContent = "";
  if (agentLastEl) agentLastEl.textContent = "Waiting...";
}


function getAgentById(id) {
  const key = String(id || "");
  if (!key) return null;
  return aiAgents.find((a) => a?.id === key) || null;
}

function ensureAgentControlStrip() {
  // Restrict spawned-agent controls to the right-panel "Spawned Agents" tab only.
  const panelContent = document.getElementById("vibe-tab-agents-pane");
  if (!panelContent) return;

  let strip = document.getElementById("agent-control-strip");

  // Re-parent strip if it ended up in the wrong panel after mode switch.
  if (strip && strip.parentElement !== panelContent) {
    strip.remove();
    strip = null;
    agentUiSelectedLabelEl = null;
    agentUiSpawnBtn = null;
    agentUiFollowBtn = null;
    agentUiStopBtn = null;
    agentUiRemoveBtn = null;
    agentUiTaskInputEl = null;
    agentUiTaskRunBtn = null;
  }

  if (agentUiSelectedLabelEl && agentUiFollowBtn && agentUiStopBtn && agentUiRemoveBtn) return;

  if (!strip) {
    strip = document.createElement("div");
    strip.id = "agent-control-strip";
    strip.className = "agent-control-strip";
    strip.innerHTML = `
      <div class="agent-control-selected" id="agent-selected-label">Selected: none</div>
      <div class="agent-control-actions">
        <button id="agent-selected-spawn" type="button" class="tb-btn tb-primary">+ Spawn</button>
        <button id="agent-selected-follow" type="button" class="tb-btn tb-muted">Follow POV</button>
        <button id="agent-selected-stop" type="button" class="tb-btn">Stop</button>
        <button id="agent-selected-remove" type="button" class="tb-btn tb-danger">Remove</button>
      </div>
      <div class="agent-control-task-row">
        <input id="agent-selected-task-input" class="agent-control-task-input" type="text" placeholder="Task for selected agent..." />
        <button id="agent-selected-task-run" type="button" class="tb-btn tb-primary">Run</button>
      </div>
    `;
    panelContent.insertBefore(strip, panelContent.firstChild || null);
  }

  agentUiSelectedLabelEl = document.getElementById("agent-selected-label");
  agentUiSpawnBtn = document.getElementById("agent-selected-spawn");
  agentUiFollowBtn = document.getElementById("agent-selected-follow");
  agentUiStopBtn = document.getElementById("agent-selected-stop");
  agentUiRemoveBtn = document.getElementById("agent-selected-remove");
  agentUiTaskInputEl = document.getElementById("agent-selected-task-input");
  agentUiTaskRunBtn = document.getElementById("agent-selected-task-run");

  agentUiSpawnBtn?.addEventListener("click", () => {
    void spawnOrMoveAiAtAim({ createNew: true, silent: false, ephemeral: false }).then(() => {
      const newest = aiAgents[aiAgents.length - 1];
      if (newest?.id) selectAgentInspector(newest.id);
      renderSelectedAgentControls();
    });
  });
  agentUiFollowBtn?.addEventListener("click", () => {
    const a = getAgentById(selectedAgentInspectorId);
    if (!a) return;
    if (agentCameraFollow && agentCameraFollowId === a.id) {
      disableAgentCameraFollow();
    } else {
      enableAgentCameraFollow(a.id);
    }
    renderSelectedAgentControls();
  });
  agentUiStopBtn?.addEventListener("click", () => {
    const a = getAgentById(selectedAgentInspectorId);
    if (!a) return;
    stopAiAgent(a, "ui-stop");
    setStatus(`Stopped ${a.id}.`);
    renderSelectedAgentControls();
  });
  agentUiRemoveBtn?.addEventListener("click", () => {
    const a = getAgentById(selectedAgentInspectorId);
    if (!a) return;
    removeAiAgent(a, "ui-remove");
    setStatus(`Removed ${a.id}.`);
    if (agentTask.active && aiAgents.length === 0) endAgentTask("all-agents-removed");
    renderSelectedAgentControls();
  });
  const runSelectedTask = () => {
    const a = getAgentById(selectedAgentInspectorId);
    if (!a) return;
    const text = String(agentUiTaskInputEl?.value || "").trim();
    if (!text) return;
    if (agentTask.active) endAgentTask("replace-task");
    void startAgentTask(text, { autoPool: false, targetAgentId: a.id });
    if (agentUiTaskInputEl) agentUiTaskInputEl.value = "";
    setStatus(`Running task on ${a.id}.`);
  };
  agentUiTaskRunBtn?.addEventListener("click", runSelectedTask);
  agentUiTaskInputEl?.addEventListener("keydown", (e) => {
    e.stopPropagation();
    if (e.key === "Enter") runSelectedTask();
  });
}

function renderSelectedAgentControls() {
  ensureAgentControlStrip();
  if (!agentUiSelectedLabelEl || !agentUiFollowBtn || !agentUiStopBtn || !agentUiRemoveBtn) return;
  const a = getAgentById(selectedAgentInspectorId);
  const has = !!a;
  agentUiSelectedLabelEl.textContent = has ? `Selected: ${a.id}` : "Selected: none";
  agentUiFollowBtn.disabled = !has;
  agentUiStopBtn.disabled = !has;
  agentUiRemoveBtn.disabled = !has;
  agentUiFollowBtn.textContent = has && agentCameraFollow && agentCameraFollowId === a.id ? "Unfollow POV" : "Follow POV";
}

function getOrCreateAgentInspectorState(agentId) {
  const id = String(agentId || "");
  if (!id) return { shot: "", request: null, response: null };
  if (!agentInspectorStateById.has(id)) {
    agentInspectorStateById.set(id, { shot: "", request: null, response: null });
  }
  return agentInspectorStateById.get(id);
}

function renderAgentInspector(agentId = selectedAgentInspectorId) {
  const id = String(agentId || "");
  if (!id) return;
  const s = getOrCreateAgentInspectorState(id);
  if (agentReqMetaEl) {
    const base = s.request || { endpoint: "-", model: "-", prompt: "", context: {}, imageBytes: null, messages: [] };
    agentUiSetRequest(base);
    agentReqMetaEl.textContent = `${agentReqMetaEl.textContent}\nagent: ${id}`;
  }
  if (s.shot) agentUiSetShot(s.shot);
  if (s.response) agentUiSetResponse(s.response);
}

function selectAgentInspector(agentId) {
  const id = String(agentId || "");
  if (!id) return;
  selectedAgentInspectorId = id;
  // Force strip into correct panel on selection.
  ensureAgentControlStrip();
  renderAgentInspector(id);
  renderSelectedAgentControls();
  // Visual flash feedback.
  const strip = document.getElementById("agent-control-strip");
  if (strip) {
    strip.style.outline = "2px solid var(--accent-primary)";
    setTimeout(() => { strip.style.outline = ""; }, 600);
  }
}

function renderAgentTaskUi() {
  ensureAgentControlStrip();
  const bar = document.getElementById("agent-command-bar");
  const hasAgent = aiAgents.length > 0;

  if (bar) bar.style.display = "";

  if (!agentTaskStatusEl || !agentTaskInputEl || !agentTaskStartBtn || !agentTaskEndBtn) return;

  if (!agentTask.active) {
    agentTaskStatusEl.textContent = "";
    agentTaskInputEl.disabled = false;
    agentTaskStartBtn.disabled = !hasAgent;
    agentTaskEndBtn.disabled = true;
    if (bar) bar.classList.remove("active");
  } else {
    agentTaskStatusEl.textContent = "Running";
    agentTaskInputEl.disabled = true;
    agentTaskStartBtn.disabled = true;
    agentTaskEndBtn.disabled = false;
    if (bar) bar.classList.add("active");
  }
  updateSimCameraModeToggleUi();
  renderSelectedAgentControls();
}

function updateSimCameraModeToggleUi() {
  if (!simCameraModeToggleBtn) return;
  const isUserCam = simUserCameraMode === "user";
  simCameraModeToggleBtn.textContent = isUserCam ? "Camera: User" : "Camera: Agent";
  simCameraModeToggleBtn.classList.toggle("active", isUserCam);
  simCameraModeToggleBtn.classList.toggle("tb-muted", !isUserCam);
  simCameraModeToggleBtn.title = isUserCam
    ? "Keep your user camera while the agent runs"
    : "Follow the active agent while the task runs";
}

function enableAgentCameraFollow(agentId = selectedAgentInspectorId) {
  if (aiAgents.length === 0) return;
  const target = getAgentById(agentId) || aiAgents[0];
  if (!target) return;
  agentCameraFollow = true;
  agentCameraFollowId = target.id;
  _agentFollowInitialized = false;

  // Unlock player controls so camera isn't fighting with pointer lock
  controls?.unlock?.();

  // Hide the player avatar
  avatar.visible = false;

  // Hide crosshair and interaction hints during follow mode
  const crosshair = document.getElementById("crosshair");
  if (crosshair) crosshair.style.display = "none";
  const hint = document.getElementById("interaction-hint");
  if (hint) hint.style.display = "none";

  console.log("[AGENT CAM] Following agent");
  renderSelectedAgentControls();
}

function disableAgentCameraFollow() {
  agentCameraFollow = false;
  agentCameraFollowId = null;

  // Show all agent meshes again
  for (const a of aiAgents) {
    if (a?.group) a.group.visible = true;
  }

  // Avatar mesh stays hidden (physics capsule still active)

  // Restore crosshair and interaction hints
  const crosshair = document.getElementById("crosshair");
  if (crosshair) crosshair.style.display = "";
  const hint = document.getElementById("interaction-hint");
  if (hint) hint.style.display = "";

  console.log("[AGENT CAM] Returning to player");
  renderSelectedAgentControls();
}

function updateAgentCameraFollow(dt) {
  if (!agentCameraFollow || aiAgents.length === 0) return;

  const agent = getAgentById(agentCameraFollowId) || aiAgents[0];
  if (!agent) return;
  const [ax, ay, az] = agent.getPosition?.() || [0, 0, 0];
  const yaw = agent.group?.rotation?.y ?? 0;
  const pitch = typeof agent.pitch === "number" ? agent.pitch : 0;

  // Place camera at the real Go2 front-camera mount: GO2_CAMERA_HEIGHT above
  // the ground and GO2_CAMERA_FORWARD along the agent's heading so the origin
  // sits outside the body mesh (Go2's head-mounted RGB-D, not body-center).
  const feetY = ay - ((agent.halfHeight || 0.25) + (agent.radius || 0.12));
  const eyeY = feetY + GO2_CAMERA_HEIGHT;
  const eyeX = ax + Math.sin(yaw) * GO2_CAMERA_FORWARD;
  const eyeZ = az + Math.cos(yaw) * GO2_CAMERA_FORWARD;
  camera.position.set(eyeX, eyeY, eyeZ);

  // Compute forward direction exactly like visionCapture.js does
  const cp = Math.cos(pitch);
  const sp = Math.sin(pitch);
  const fx = Math.sin(yaw) * cp;
  const fy = sp;
  const fz = Math.cos(yaw) * cp;

  // Use lookAt to match the VLM capture camera
  camera.lookAt(eyeX + fx, eyeY + fy, eyeZ + fz);

  // Hide the agent's own mesh so it doesn't block the view
  if (agent.group) agent.group.visible = false;
}

async function startAgentTask(instruction, { autoPool = true, targetAgentId = null } = {}) {
  const text = String(instruction || "").trim();
  if (!text) return;

  const now = Date.now();
  const taskState = {
    active: true,
    instruction: text,
    startedAt: now,
    finishedAt: 0,
    finishedReason: "",
    lastSummary: "",
  };

  // Determine which agents get this task
  const target = targetAgentId ? getAgentById(targetAgentId) : null;
  agentTaskTargetId = target?.id || null;
  const targets = target ? [target] : aiAgents;

  for (const a of targets) {
    _setAgentTask(a.id, { ...taskState });
    a._taskStartedAt = now;
  }

  agentUiPush(`${new Date().toLocaleTimeString()}\nTASK START\n${text}${target ? ` [${target.id}]` : ` [${targets.length} agents]`}`);
  renderAgentTaskUi();

  if (simUserCameraMode === "agent") enableAgentCameraFollow();
}

function endAgentTask(reason = "manual", agentId = null) {
  if (agentId) {
    // End task for a specific agent
    const task = _agentTasks.get(agentId);
    if (task?.active) {
      task.active = false;
      task.finishedAt = Date.now();
      task.finishedReason = reason;
      _agentTasks.set(agentId, task);
    }
    agentUiPush(`${new Date().toLocaleTimeString()}\nTASK END (${reason}) [${agentId}]`);
  } else {
    // End all tasks
    for (const [id, task] of _agentTasks) {
      if (task.active) {
        task.active = false;
        task.finishedAt = Date.now();
        task.finishedReason = reason;
      }
    }
    agentTask.active = false;
    agentTask.finishedAt = Date.now();
    agentTask.finishedReason = reason;
    agentUiPush(`${new Date().toLocaleTimeString()}\nTASK END ALL (${reason})`);
  }
  agentTaskTargetId = null;

  // Check if any agent still has an active task
  const anyActive = [..._agentTasks.values()].some((t) => t.active);
  if (!anyActive) {
    agentTask.active = false;
    disableAgentCameraFollow();
  }

  renderAgentTaskUi();

}

function rebuildTagMarkers() {
  while (tagsGroup.children.length) tagsGroup.remove(tagsGroup.children[0]);

  for (const t of tags) {
    if (!t.position) continue;
    const m = new THREE.Mesh(markerGeom, t.id === selectedTagId ? markerMatActive : markerMat);
    m.position.set(t.position.x, t.position.y, t.position.z);
    m.userData.tagId = t.id;
    m.renderOrder = 1000;
    tagsGroup.add(m);

    const r = Number(t.radius ?? 1.5);
    const shell = new THREE.Mesh(radiusGeom, radiusMat);
    shell.position.copy(m.position);
    shell.scale.setScalar(Math.max(0.01, r));
    shell.userData.tagId = t.id;
    shell.userData.isRadius = true;
    tagsGroup.add(shell);
  }

  updateMarkerMaterials();
}

function updateMarkerMaterials() {
  for (const child of tagsGroup.children) {
    if (!child.isMesh) continue;
    if (child.userData?.isRadius) continue;
    child.material = child.userData.tagId === selectedTagId ? markerMatActive : markerMat;
  }
}


function arrayBufferFromBase64(base64) {
  const bin = atob(base64);
  const len = bin.length;
  const bytes = new Uint8Array(len);
  for (let i = 0; i < len; i++) bytes[i] = bin.charCodeAt(i);
  return bytes.buffer;
}


function normalizeShapeStateScene(sceneLike) {
  const raw = sceneLike || { tags: [], primitives: [], lights: [], groups: [] };
  return {
    tags: Array.isArray(raw.tags) ? raw.tags : [],
    primitives: Array.isArray(raw.primitives) ? raw.primitives : [],
    lights: Array.isArray(raw.lights) ? raw.lights : [],
    groups: Array.isArray(raw.groups) ? raw.groups : [],
  };
}

function buildShapeStateRoot(state, assetId, fixedPivotCenter = null) {
  const sceneState = normalizeShapeStateScene(state?.scene || state?.shapeScene);
  const root = new THREE.Group();
  const primMap = new Map();
  for (const p of sceneState.primitives) {
    const geom = createPrimitiveGeometry(p.type, p.dimensions || {});
    const mat = createPrimitiveMaterial(p.material || {});
    const mesh = new THREE.Mesh(geom, mat);
    applyPrimitiveCutoutShader(mesh, p);
    mesh.name = `assetPrim:${assetId}:${p.id || randId()}`;
    mesh.userData.assetId = assetId;
    mesh.userData.isAssetPrimitive = true;
    mesh.castShadow = p.castShadow !== false;
    mesh.receiveShadow = p.receiveShadow !== false;
    const tr = p.transform || {};
    if (tr.position) mesh.position.set(tr.position.x || 0, tr.position.y || 0, tr.position.z || 0);
    if (tr.rotation) mesh.rotation.set(tr.rotation.x || 0, tr.rotation.y || 0, tr.rotation.z || 0);
    if (tr.scale) mesh.scale.set(tr.scale.x ?? 1, tr.scale.y ?? 1, tr.scale.z ?? 1);
    root.add(mesh);
    if (p.id) primMap.set(p.id, mesh);
  }
  for (const g of sceneState.groups || []) {
    if (!Array.isArray(g.children) || g.children.length === 0) continue;
    const subgroup = new THREE.Group();
    subgroup.name = `assetGroup:${assetId}:${g.id || randId()}`;
    root.add(subgroup);
    for (const cid of g.children) {
      const child = primMap.get(cid);
      if (!child) continue;
      subgroup.add(child);
    }
  }

  // Re-center: move the pivot to the bounding-box center so the transform
  // gizmo appears on the asset rather than at an arbitrary offset.
  root.updateMatrixWorld(true);
  const bbox = new THREE.Box3().setFromObject(root);
  if (!bbox.isEmpty()) {
    const autoCenter = bbox.getCenter(new THREE.Vector3());
    const center = fixedPivotCenter ? fixedPivotCenter.clone() : autoCenter;
    for (const child of root.children) {
      child.position.sub(center);
    }
    root.position.copy(center);
    root.userData._pivotCenter = center.clone();
  }

  return root;
}

function disposeShapeStateRoot(root) {
  if (!root) return;
  root.traverse((obj) => {
    if (!obj?.isMesh) return;
    obj.geometry?.dispose?.();
    disposePrimitiveMaterial(obj.material);
  });
}

async function instantiateAsset(a) {
  if (!a?.states) return;
  const sId = a.currentStateId || a.currentState || (Array.isArray(a.states) ? a.states[0]?.id : "A");
  const state = Array.isArray(a.states)
    ? a.states.find((s) => s.id === sId) || a.states[0]
    : a.states[sId] || a.states.A;
  let root = null;
  if (state?.scene || state?.shapeScene) {
    let fixedPivotCenter = null;
    if (a._shapePivotCenter
      && Number.isFinite(a._shapePivotCenter.x)
      && Number.isFinite(a._shapePivotCenter.y)
      && Number.isFinite(a._shapePivotCenter.z)) {
      fixedPivotCenter = new THREE.Vector3(a._shapePivotCenter.x, a._shapePivotCenter.y, a._shapePivotCenter.z);
    } else if (Array.isArray(a.states) && a.states.length > 0) {
      const anchorState = a.states[0];
      const anchorRoot = buildShapeStateRoot(anchorState, `${a.id}:anchor`);
      const anchorCenter = anchorRoot.userData?._pivotCenter;
      if (anchorCenter) {
        fixedPivotCenter = anchorCenter.clone();
        a._shapePivotCenter = { x: anchorCenter.x, y: anchorCenter.y, z: anchorCenter.z };
      }
      disposeShapeStateRoot(anchorRoot);
    }
    root = buildShapeStateRoot(state, a.id, fixedPivotCenter);
    const rootCenter = root.userData?._pivotCenter;
    if (rootCenter && !a._shapePivotCenter) {
      a._shapePivotCenter = { x: rootCenter.x, y: rootCenter.y, z: rootCenter.z };
    }
  } else if (state?.dataBase64) {
    const buf = arrayBufferFromBase64(state.dataBase64);
    const url = URL.createObjectURL(new Blob([buf], { type: "model/gltf-binary" }));
    const gltf = await new Promise((resolve, reject) => {
      gltfLoader.load(url, (g) => resolve(g), undefined, (e) => reject(e));
    });
    URL.revokeObjectURL(url);
    root = gltf.scene;
  } else if (state?.glbUrl) {
    // Cache by URL so library assets shared across N placed instances (e.g.
    // 4 dining chairs pointing to the same file) parse once and share
    // geometry+materials in memory. clone(true) deep-copies the node tree but
    // reuses BufferGeometry/Material — standard Three.js instancing pattern.
    let cached = _glbResultCache.get(state.glbUrl);
    if (!cached) {
      cached = await new Promise((resolve, reject) => {
        gltfLoader.load(state.glbUrl, (g) => resolve(g), undefined, (e) => reject(e));
      });
      _glbResultCache.set(state.glbUrl, cached);
    }
    root = cached.scene.clone(true);
  } else {
    return;
  }
  root.name = `asset:${a.id}`;
  const wantShadow = a.castShadow === true; // opt-in, default OFF
  const wantReceive = a.receiveShadow === true; // opt-in, default OFF

  root.traverse((m) => {
    if (m.isMesh) {
      if (!m.userData?.isAssetPrimitive) m.castShadow = false; // GLB assets keep cheap shadow behavior
      m.receiveShadow = wantReceive;
      m.userData.assetId = a.id;
    }
  });

  // Pre-compute local bounding sphere ONCE (cached — never call setFromObject again)
  const bbox = new THREE.Box3().setFromObject(root);
  const localSphere = new THREE.Sphere();
  bbox.getBoundingSphere(localSphere);
  const localCenter = localSphere.center.clone();
  root.worldToLocal(localCenter);
  root.userData._localSphereCenter = localCenter;
  root.userData._localSphereRadius = Math.max(localSphere.radius, 0.2);

  // Blob shadow: a cheap flat gradient circle beneath the asset.
  // Uses zero shadow-map resources — just a textured plane with transparency.
  if (wantShadow) {
    const bboxSize = bbox.getSize(new THREE.Vector3());
    const localGroundY = bbox.min.y + 0.005;
    const blob = createBlobShadow(a.id, bboxSize.x, bboxSize.z, localGroundY, {
      opacity: a.blobShadow?.opacity ?? 0.5,
      scale: a.blobShadow?.scale ?? 1.0,
      stretch: a.blobShadow?.stretch ?? 1.0,
      rotationDeg: a.blobShadow?.rotationDeg ?? 0,
      offsetX: a.blobShadow?.offsetX ?? 0,
      offsetY: a.blobShadow?.offsetY ?? 0,
      offsetZ: a.blobShadow?.offsetZ ?? 0,
    });
    if (blob) root.add(blob);
  }

  const tr = a.transform || {};
  if (tr.position) root.position.set(tr.position.x, tr.position.y, tr.position.z);
  if (tr.rotation) root.rotation.set(tr.rotation.x, tr.rotation.y, tr.rotation.z);
  if (tr.scale) root.scale.set(tr.scale.x, tr.scale.y, tr.scale.z);
  assetsGroup.add(root);
  await rebuildAssetCollider(a.id);
}

async function setAssetState(assetId, nextState) {
  const a = assets.find((x) => x.id === assetId);
  if (!a) return;
  const exists = Array.isArray(a.states) ? a.states.some((s) => s.id === nextState) : !!a.states?.[nextState];
  if (!exists) return;
  a.currentStateId = nextState;
  saveTagsForWorld();
  // Replace visual
  const existing = assetsGroup.getObjectByName(`asset:${a.id}`);
  if (existing?.parent) existing.parent.remove(existing);
  await instantiateAsset(a);
}

async function applyAssetAction(assetId, actionId) {
  const a = assets.find((x) => x.id === assetId);
  if (!a) return false;

  const act = (a.actions || []).find((x) => x.id === actionId) || null;
  if (!act) return false;
  const cur = a.currentStateId || a.currentState || "A";
  if (cur !== act.from) return false;
  await setAssetState(assetId, act.to);
  return true;
}


// ============================================================================
// PLAYER INTERACTION SYSTEM
// ============================================================================
const PLAYER_INTERACT_DISTANCE = 1.5; // Max distance player can interact with assets
const _playerInteractRaycaster = new THREE.Raycaster();
let _interactionPopup = null;
let _currentInteractableAsset = null;
let _crosshairInteractCycleIndex = 0;
let _crosshairInteractCycleSig = "";
let _crosshairInteractCandidates = [];

// ============================================================================
// PICK UP / DROP SYSTEM
// ============================================================================
let playerHeldAsset = null; // Asset ID currently held by player
let playerHeldGroupId = null; // Group ID currently held by player
const agentHeldAssets = new Map(); // Map<agentId, assetId> - assets held by each agent

/**
 * Check if an asset is currently being held by anyone
 */
function isAssetHeld(assetId) {
  if (playerHeldAsset === assetId) return { held: true, by: "player" };
  for (const [agentId, heldId] of agentHeldAssets.entries()) {
    if (heldId === assetId) return { held: true, by: "agent", agentId };
  }
  return { held: false };
}

function isGroupHeld(groupId) {
  if (playerHeldGroupId === groupId) return { held: true, by: "player" };
  return { held: false };
}

function getGroupById(groupId) {
  return groups.find((g) => g.id === groupId) || null;
}

function getGroupCentroid(groupId) {
  const g = getGroupById(groupId);
  if (!g || !Array.isArray(g.children) || g.children.length === 0) return null;
  let cx = 0, cy = 0, cz = 0, count = 0;
  for (const cid of g.children) {
    const p = primitives.find((x) => x.id === cid);
    const pos = p?.transform?.position;
    if (!pos) continue;
    cx += pos.x || 0;
    cy += pos.y || 0;
    cz += pos.z || 0;
    count++;
  }
  if (count === 0) return null;
  return { x: cx / count, y: cy / count, z: cz / count };
}

function playerPickUpGroup(groupId) {
  const g = getGroupById(groupId);
  if (!g) return { ok: false, reason: "not-found" };
  if (!g.pickable) return { ok: false, reason: "not-pickable" };
  if (playerHeldAsset || playerHeldGroupId) return { ok: false, reason: "hands-full" };
  const holdStatus = isGroupHeld(groupId);
  if (holdStatus.held) return { ok: false, reason: "already-held", by: holdStatus.by };

  playerHeldGroupId = groupId;
  for (const cid of g.children || []) {
    const mesh = primitivesGroup.getObjectByName(`prim:${cid}`);
    if (mesh) mesh.visible = false;
    const prim = primitives.find((p) => p.id === cid);
    if (prim) removePrimitiveCollider(prim);
  }
  setStatus(`Picked up group: ${g.name || "group"}`);
  return { ok: true };
}

function playerDropGroup() {
  if (!playerHeldGroupId) return { ok: false, reason: "not-holding" };
  const g = getGroupById(playerHeldGroupId);
  if (!g) {
    playerHeldGroupId = null;
    return { ok: false, reason: "not-found" };
  }
  const centroid = getGroupCentroid(g.id);
  if (!centroid) {
    playerHeldGroupId = null;
    return { ok: false, reason: "invalid-group" };
  }
  // Raycast from crosshair to find drop point
  const dropRay = new THREE.Raycaster();
  dropRay.setFromCamera({ x: 0, y: 0 }, camera);
  dropRay.far = 6;
  const candidates = [];
  // Exclude held group's own meshes
  const heldChildSet = new Set(g.children || []);
  primitivesGroup.traverse((c) => {
    if (c.isMesh && !heldChildSet.has(c.name?.replace("prim:", ""))) candidates.push(c);
  });
  assetsGroup.traverse((c) => { if (c.isMesh) candidates.push(c); });
  scene.traverse((c) => {
    if (c.isMesh && !candidates.includes(c) && c.parent !== assetsGroup && c.parent !== primitivesGroup) candidates.push(c);
  });
  const hits = dropRay.intersectObjects(candidates, false);
  let dropPos;
  if (hits.length > 0) {
    dropPos = { x: hits[0].point.x, y: hits[0].point.y, z: hits[0].point.z };
  } else {
    const forward = new THREE.Vector3();
    camera.getWorldDirection(forward);
    dropPos = {
      x: camera.position.x + forward.x * 1.5,
      y: 0,
      z: camera.position.z + forward.z * 1.5,
    };
  }
  const dx = dropPos.x - centroid.x;
  const dz = dropPos.z - centroid.z;
  for (const cid of g.children || []) {
    const prim = primitives.find((p) => p.id === cid);
    if (!prim?.transform?.position) continue;
    prim.transform.position.x += dx;
    prim.transform.position.z += dz;
    const mesh = primitivesGroup.getObjectByName(`prim:${cid}`);
    if (mesh) {
      mesh.position.x = prim.transform.position.x;
      mesh.position.y = prim.transform.position.y;
      mesh.position.z = prim.transform.position.z;
      mesh.visible = true;
    }
    rebuildPrimitiveColliderSync(prim);
  }
  const droppedId = playerHeldGroupId;
  playerHeldGroupId = null;
  saveTagsForWorld();
  setStatus(`Dropped group: ${g.name || "group"}`);
  return { ok: true, groupId: droppedId };
}

/**
 * Pick up an asset (for player)
 */
function playerPickUpAsset(assetId) {
  const asset = assets.find(a => a.id === assetId);
  if (!asset) return { ok: false, reason: "not-found" };
  if (!asset.pickable) return { ok: false, reason: "not-pickable" };

  const holdStatus = isAssetHeld(assetId);
  if (holdStatus.held) return { ok: false, reason: "already-held", by: holdStatus.by };

  if (playerHeldAsset) return { ok: false, reason: "hands-full" };

  playerHeldAsset = assetId;

  // Hide the asset from the scene (it's now "in hand")
  const obj = assetsGroup.getObjectByName(`asset:${assetId}`);
  if (obj) obj.visible = false;

  // Remove collider while held
  removeAssetCollider(assetId);

  console.log(`[PICKUP] Player picked up: ${asset.title || assetId}`);
  setStatus(`Picked up: ${asset.title || "item"}`);
  return { ok: true };
}

/**
 * Drop the held asset (for player)
 */
function playerDropAsset() {
  if (!playerHeldAsset) return { ok: false, reason: "not-holding" };

  const asset = assets.find(a => a.id === playerHeldAsset);
  if (!asset) {
    playerHeldAsset = null;
    return { ok: false, reason: "not-found" };
  }

  // Raycast from crosshair to find where the player is looking
  const dropRay = new THREE.Raycaster();
  dropRay.setFromCamera({ x: 0, y: 0 }, camera);
  dropRay.far = 6;
  // Collect all scene meshes except the held asset itself
  const candidates = [];
  primitivesGroup.traverse((c) => { if (c.isMesh) candidates.push(c); });
  assetsGroup.traverse((c) => {
    if (c.isMesh && !c.name?.includes(playerHeldAsset)) candidates.push(c);
  });
  // Also include splat / collision meshes if any
  scene.traverse((c) => {
    if (c.isMesh && !candidates.includes(c) && c.parent !== assetsGroup && c.parent !== primitivesGroup) candidates.push(c);
  });
  const hits = dropRay.intersectObjects(candidates, false);
  let dropPos;
  if (hits.length > 0) {
    // Place at the hit point
    dropPos = hits[0].point.clone();
  } else {
    // Fallback: fixed distance along look direction, at ground level
    const forward = new THREE.Vector3();
    camera.getWorldDirection(forward);
    const fallbackDist = 1.5;
    dropPos = new THREE.Vector3(
      camera.position.x + forward.x * fallbackDist,
      0,
      camera.position.z + forward.z * fallbackDist
    );
  }

  // Update asset transform
  asset.transform.position = { x: dropPos.x, y: dropPos.y, z: dropPos.z };

  // Show and reposition the asset — traverse to ensure all children are visible
  const obj = assetsGroup.getObjectByName(`asset:${playerHeldAsset}`);
  if (obj) {
    obj.position.copy(dropPos);
    obj.visible = true;
    obj.traverse((child) => { child.visible = true; });
  } else {
    // Object was lost — re-instantiate from asset data
    console.warn(`[DROP] 3D object missing for ${playerHeldAsset}, re-instantiating...`);
    instantiateAsset(asset);
  }

  // Rebuild collider
  rebuildAssetCollider(playerHeldAsset);

  console.log(`[DROP] Player dropped: ${asset.title || playerHeldAsset}`);
  setStatus(`Dropped: ${asset.title || "item"}`);

  const droppedId = playerHeldAsset;
  playerHeldAsset = null;
  saveTagsForWorld();

  return { ok: true, assetId: droppedId };
}

/**
 * Pick up an asset (for AI agent)
 */

/**
 * Drop the held asset (for AI agent)
 */

/**
 * Remove collider for an asset (when picked up)
 */
function removeAssetCollider(assetId) {
  const handle = _assetColliderHandles.get(assetId);
  if (handle != null && rapierWorld) {
    const collider = rapierWorld.getCollider(handle);
    if (collider) rapierWorld.removeCollider(collider, true);
    _assetColliderHandles.delete(assetId);
  }
}

/**
 * Get what the player is currently holding
 */
function getPlayerHeldAsset() {
  if (!playerHeldAsset) return null;
  return assets.find(a => a.id === playerHeldAsset) || null;
}

/**
 * Get what an agent is currently holding
 */

/**
 * Get the interactable asset at the player's crosshair (center of screen).
 * Returns { asset, actions, dist, canPickUp } if found, or null if nothing interactable.
 */
const _hintRayOrigin = new THREE.Vector3();
const _hintRayDir = new THREE.Vector3();
const _hintTmpSphere = new THREE.Sphere();
const _hintRay = new THREE.Ray();
const _cachedSphereCenter = new THREE.Vector3();

// Get the world-space bounding sphere of an asset from its cached local data.
// This is O(1) — no vertex traversal, just one matrix-vector multiply.
function getAssetWorldSphere(obj, outSphere) {
  const lc = obj.userData._localSphereCenter;
  const lr = obj.userData._localSphereRadius;
  if (lc && lr) {
    _cachedSphereCenter.copy(lc);
    obj.localToWorld(_cachedSphereCenter);
    const scale = obj.matrixWorld.getMaxScaleOnAxis();
    outSphere.set(_cachedSphereCenter, lr * scale);
    return true;
  }
  return false;
}

function getInteractableAssetCandidatesAtCrosshair() {
  if (!camera) return [];
  camera.getWorldPosition(_hintRayOrigin);
  camera.getWorldDirection(_hintRayDir);
  _hintRay.set(_hintRayOrigin, _hintRayDir);

  const maxDist = PLAYER_INTERACT_DISTANCE + 0.8;
  const candidates = [];
  for (const child of assetsGroup.children) {
    const aid = child.name?.startsWith("asset:") ? child.name.slice(6) : null;
    if (!aid) continue;
    if (!getAssetWorldSphere(child, _hintTmpSphere)) continue;
    _hintTmpSphere.radius = Math.max(_hintTmpSphere.radius, 0.3);
    const centerDist = _hintRayOrigin.distanceTo(_hintTmpSphere.center);
    if (centerDist > maxDist + _hintTmpSphere.radius) continue;
    const hitPoint = _hintRay.intersectSphere(_hintTmpSphere, _tmpV1);
    if (!hitPoint) continue;
    const d = _hintRayOrigin.distanceTo(hitPoint);
    if (d > maxDist) continue;
    const toCenter = _tmpV2.copy(_hintTmpSphere.center).sub(_hintRayOrigin).normalize();
    const aim = Math.max(0, _hintRayDir.dot(toCenter));
    const score = aim * 4.0 - d * 0.45;
    candidates.push({ id: aid, dist: d, aim, score });
  }
  candidates.sort((a, b) => (b.score - a.score) || (a.dist - b.dist));
  return candidates.slice(0, 6);
}

function cycleInteractableTarget(step = 1) {
  const candidates = getInteractableAssetCandidatesAtCrosshair();
  if (!Array.isArray(candidates) || candidates.length <= 1) return false;
  const sig = candidates.map((c) => c.id).join("|");
  if (sig !== _crosshairInteractCycleSig) {
    _crosshairInteractCycleSig = sig;
    _crosshairInteractCycleIndex = 0;
  }
  const len = candidates.length;
  _crosshairInteractCycleIndex = (_crosshairInteractCycleIndex + step + len) % len;
  _crosshairInteractCandidates = candidates;
  return true;
}

function getInteractableAssetAtCrosshair() {
  const candidates = getInteractableAssetCandidatesAtCrosshair();
  const sig = candidates.map((c) => c.id).join("|");
  if (sig !== _crosshairInteractCycleSig) {
    _crosshairInteractCycleSig = sig;
    _crosshairInteractCycleIndex = 0;
  }
  _crosshairInteractCandidates = candidates;
  const primary = candidates[_crosshairInteractCycleIndex] || null;

  if (!primary) {
    // Fallback: pickable grouped shape assets
    _playerInteractRaycaster.setFromCamera({ x: 0, y: 0 }, camera);
    const hits = _playerInteractRaycaster.intersectObjects(primitivesGroup.children, false);
    for (const hit of hits) {
      if (hit.distance > PLAYER_INTERACT_DISTANCE + 0.5) continue;
      const name = hit.object?.name || "";
      const m = name.match(/^prim:(.+)$/);
      if (!m) continue;
      const primId = m[1];
      const g = groups.find((gr) => (gr.children || []).includes(primId) && gr.pickable);
      if (!g) continue;
      const canPickUp = !playerHeldAsset && !playerHeldGroupId && !isGroupHeld(g.id).held;
      return { kind: "group", group: g, actions: [], dist: hit.distance, canPickUp };
    }
    return null;
  }

  const asset = assets.find((a) => a.id === primary.id);
  if (!asset) return null;

  const currentState = asset.currentStateId || asset.currentState || "A";
  const actions = (asset.actions || []).filter((act) => act.from === currentState);
  const holdStatus = isAssetHeld(primary.id);
  const canPickUp = asset.pickable && !holdStatus.held && !playerHeldAsset && !playerHeldGroupId;

  if (actions.length === 0 && !canPickUp) return null;

  return {
    kind: "asset",
    asset,
    actions,
    dist: primary.dist,
    canPickUp,
    candidateIndex: _crosshairInteractCycleIndex,
    candidateCount: candidates.length,
  };
}

/**
 * Create or get the interaction popup element
 */
function getInteractionPopup() {
  if (_interactionPopup) return _interactionPopup;

  _interactionPopup = document.createElement("div");
  _interactionPopup.id = "interaction-popup";
  // Styles are now in CSS, just set display none initially
  _interactionPopup.style.display = "none";
  document.body.appendChild(_interactionPopup);
  return _interactionPopup;
}

/**
 * Show the interaction popup with available actions
 */
function showInteractionPopup(asset, actions) {
  const popup = getInteractionPopup();

  // Build popup content
  const title = asset.title || "(asset)";
  const stateObj = Array.isArray(asset.states)
    ? asset.states.find((s) => s.id === (asset.currentStateId || asset.currentState))
    : null;
  const stateName = stateObj?.name || "";

  let html = `<div style="font-size: 11px; color: rgba(255,255,255,0.5); padding: 6px 10px 4px; font-weight: 600; letter-spacing: 0.02em;">${escapeHtml(title)}${stateName ? ` · ${escapeHtml(stateName)}` : ""}</div>`;

  actions.forEach((act, idx) => {
    html += `<button class="interact-action-btn" data-action-id="${escapeHtml(act.id)}" data-idx="${idx}">
      <span style="color: #6366f1; font-size: 11px; font-weight: 700; min-width: 24px;">[${idx + 1}]</span>
      ${escapeHtml(act.label || "interact")}
    </button>`;
  });

  html += `<div style="font-size: 10px; color: rgba(255,255,255,0.35); padding: 8px 10px 4px; text-align: center; border-top: 1px solid rgba(255,255,255,0.06); margin-top: 4px;">Press <b style="color: rgba(255,255,255,0.6);">1-${actions.length}</b> or click · <b style="color: rgba(255,255,255,0.6);">Esc</b> to cancel</div>`;

  popup.innerHTML = html;
  popup.style.display = "flex";
  _currentInteractableAsset = { asset, actions };

  // Add click handlers to buttons
  popup.querySelectorAll(".interact-action-btn").forEach((btn) => {
    btn.addEventListener("click", async (e) => {
      e.stopPropagation();
      const actionId = btn.getAttribute("data-action-id");

      // Hide popup first
      hideInteractionPopup();

      // Execute the action
      if (actionId === "__PICK_UP__") {
        playerPickUpAsset(asset.id);
      } else {
        await executePlayerInteraction(asset.id, actionId);
      }

      // Re-lock pointer after a short delay (click events can re-lock)
      setTimeout(() => {
        try {
          controls?.lock?.();
        } catch (err) {
          // Ignore
        }
      }, 50);
    });
    // Hover effects handled in CSS
  });
}

/**
 * Hide the interaction popup
 */
function hideInteractionPopup() {
  if (_interactionPopup) {
    _interactionPopup.style.display = "none";
  }
  _currentInteractableAsset = null;
}

/**
 * Check if interaction popup is visible
 */
function isInteractionPopupVisible() {
  return _interactionPopup?.style.display === "flex";
}

/**
 * Execute a player interaction with an asset
 */
async function executePlayerInteraction(assetId, actionId) {
  // Handle special pick up action
  if (actionId === "__PICK_UP__") {
    const result = playerPickUpAsset(assetId);
    return result.ok;
  }

  const asset = assets.find((a) => a.id === assetId);
  if (!asset) {
    setStatus("Asset not found.");
    return false;
  }

  const action = (asset.actions || []).find((a) => a.id === actionId);
  if (!action) {
    setStatus("Action not available.");
    return false;
  }

  const ok = await applyAssetAction(assetId, actionId);
  if (ok) {
    setStatus(`${action.label || "Interacted"}: ${asset.title || "asset"}`);
  } else {
    setStatus("Interaction failed.");
  }
  return ok;
}

/**
 * Handle player interaction attempt (click or E key)
 */
async function handlePlayerInteraction() {
  // If popup is already showing, do nothing (let popup handle it)
  if (isInteractionPopupVisible()) {
    return;
  }

  // First, check if player is holding something - pressing E drops it
  if (playerHeldAsset) {
    playerDropAsset();
    return;
  }
  if (playerHeldGroupId) {
    playerDropGroup();
    return;
  }

  const target = getInteractableAssetAtCrosshair();
  if (!target) {
    // No interactable asset at crosshair
    return;
  }

  const { kind, asset, group, actions, dist, canPickUp } = target;
  if (kind === "group") {
    if (canPickUp) playerPickUpGroup(group.id);
    return;
  }

  // Build combined action list (regular actions + pick up if available)
  const combinedActions = [...actions];
  if (canPickUp) {
    combinedActions.push({ id: "__PICK_UP__", label: "Pick up", special: true });
  }

  if (combinedActions.length === 1) {
    // Single action - execute immediately
    if (combinedActions[0].id === "__PICK_UP__") {
      playerPickUpAsset(asset.id);
    } else {
      await executePlayerInteraction(asset.id, combinedActions[0].id);
    }
  } else if (combinedActions.length > 1) {
    // Multiple actions - show selection popup
    // Temporarily unlock pointer to allow clicking popup
    controls?.unlock?.();
    showInteractionPopup(asset, combinedActions);
  }
}

// ============================================================================
// END PLAYER INTERACTION SYSTEM
// ============================================================================


async function buildRapierTriMeshColliderFromObject(obj) {
  await ensureRapierLoaded();
  const verts = [];
  const indices = [];
  let vertBase = 0;
  const tmpPos = new THREE.Vector3();
  obj.updateMatrixWorld(true);

  obj.traverse((m) => {
    if (!m.isMesh) return;
    const geom = m.geometry;
    const posAttr = geom?.attributes?.position;
    if (!posAttr) return;
    const indexAttr = geom.index;
    const matWorld = m.matrixWorld;

    for (let i = 0; i < posAttr.count; i++) {
      tmpPos.fromBufferAttribute(posAttr, i).applyMatrix4(matWorld);
      verts.push(tmpPos.x, tmpPos.y, tmpPos.z);
    }

    if (indexAttr) {
      for (let i = 0; i < indexAttr.count; i++) indices.push(indexAttr.getX(i) + vertBase);
    } else {
      for (let i = 0; i < posAttr.count; i++) indices.push(vertBase + i);
    }
    vertBase += posAttr.count;
  });

  if (verts.length < 9 || indices.length < 3) return null;
  const desc = RAPIER.ColliderDesc.trimesh(verts, indices).setFriction(0.9);
  return rapierWorld.createCollider(desc);
}

async function rebuildAssetCollider(assetId) {
  const a = assets.find((x) => x.id === assetId);
  if (!a) return;
  await ensureRapierLoaded();
  if (!rapierWorld || !RAPIER) return;

  // Remove existing collider
  if (a._colliderHandle != null) {
    try {
      if (typeof a._colliderHandle === 'object' && a._colliderHandle.handle !== undefined) {
        rapierWorld.removeCollider(a._colliderHandle, true);
      }
    } catch (e) {
      console.warn(`[COLLIDER] Failed to remove collider for ${assetId}:`, e);
    }
    a._colliderHandle = null;
  }

  const obj = assetsGroup.getObjectByName(`asset:${assetId}`);
  if (!obj) return;
  const collider = await buildRapierTriMeshColliderFromObject(obj);
  if (collider) {
    a._colliderHandle = collider;
  }
}


async function rebuildAssets() {
  while (assetsGroup.children.length) assetsGroup.remove(assetsGroup.children[0]);
  for (const a of assets) {
    try {
      await instantiateAsset(a);
    } catch (e) {
      console.warn("Failed to rebuild asset", a?.glb?.name, e);
    }
  }
}

// =============================================================================
// PRIMITIVES – Parametric Shape System (Level Editor)
// =============================================================================

function createPrimitiveGeometry(type, dims) {
  dims = dims || {};
  const num = (v, fallback) => {
    const n = Number(v);
    return Number.isFinite(n) ? n : fallback;
  };
  const degToRad = (deg, fallback = 0) => (Number.isFinite(deg) ? deg : fallback) * Math.PI / 180;
  const clampInt = (v, fallback, min = 1) => Math.max(min, Math.floor(Number(v) || fallback));
  const clamp01 = (v, fallback = 0) => {
    const n = Number(v);
    if (!Number.isFinite(n)) return fallback;
    return Math.max(0, Math.min(1, n));
  };
  switch (type) {
    case "box": {
      const width = Math.max(0.01, num(dims.width, 1));
      const height = Math.max(0.01, num(dims.height, 1));
      const depth = Math.max(0.01, num(dims.depth, 1));
      const edgeRadius = Math.max(0, num(dims.edgeRadius, 0));
      if (edgeRadius > 0) {
        const radius = Math.min(edgeRadius, width * 0.5, height * 0.5, depth * 0.5);
        const edgeSegments = clampInt(dims.edgeSegments, 4, 1);
        return new RoundedBoxGeometry(width, height, depth, edgeSegments, radius);
      }
      return new THREE.BoxGeometry(
        width,
        height,
        depth,
        clampInt(dims.widthSegments, 1, 1),
        clampInt(dims.heightSegments, 1, 1),
        clampInt(dims.depthSegments, 1, 1)
      );
    }
    case "sphere":
      return new THREE.SphereGeometry(
        Math.max(0.01, num(dims.radius, 0.5)),
        clampInt(dims.widthSegments, 32, 3),
        clampInt(dims.heightSegments, 16, 2),
        degToRad(num(dims.phiStartDeg, 0), 0),
        degToRad(num(dims.phiLengthDeg, 360), 360),
        degToRad(num(dims.thetaStartDeg, 0), 0),
        degToRad(num(dims.thetaLengthDeg, 180), 180)
      );
    case "cylinder":
      return new THREE.CylinderGeometry(
        Math.max(0.01, num(dims.radiusTop, 0.5)),
        Math.max(0.01, num(dims.radiusBottom, 0.5)),
        Math.max(0.01, num(dims.height, 1)),
        clampInt(dims.radialSegments, 32, 3),
        clampInt(dims.heightSegments, 1, 1),
        clamp01(dims.openEnded, 0) >= 0.5
      );
    case "cone":
      return new THREE.ConeGeometry(
        Math.max(0.01, num(dims.radius, 0.5)),
        Math.max(0.01, num(dims.height, 1)),
        clampInt(dims.radialSegments, 32, 3),
        clampInt(dims.heightSegments, 1, 1),
        clamp01(dims.openEnded, 0) >= 0.5
      );
    case "torus":
      return new THREE.TorusGeometry(
        Math.max(0.01, num(dims.radius, 0.5)),
        Math.max(0.01, num(dims.tube, 0.15)),
        clampInt(dims.radialSegments, 16, 3),
        clampInt(dims.tubularSegments, 48, 3),
        degToRad(num(dims.arcDeg, 360), 360)
      );
    case "plane":
      return new THREE.PlaneGeometry(
        Math.max(0.01, num(dims.width, 2)),
        Math.max(0.01, num(dims.height, 2)),
        clampInt(dims.widthSegments, 1, 1),
        clampInt(dims.heightSegments, 1, 1)
      );
    default:
      return new THREE.BoxGeometry(1, 1, 1);
  }
}

const _textureLoader = new THREE.TextureLoader();
const _textureCache = new Map(); // dataUrl → THREE.Texture
const _glbResultCache = new Map(); // glbUrl → loaded gltf result (shared across instances)

function createPrimitiveMaterial(mat) {
  mat = mat || {};
  const uv = mat.uvTransform || {};
  const clamp01 = (v, fallback = 0) => {
    const n = Number(v);
    if (!Number.isFinite(n)) return fallback;
    return Math.max(0, Math.min(1, n));
  };
  const hardness = clamp01(mat.hardness, 0);
  const fluffiness = clamp01(mat.fluffiness, 0);
  const params = {
    color: new THREE.Color(mat.color || "#808080"),
    roughness: mat.softness ?? mat.roughness ?? 0.7,
    metalness: mat.metalness ?? 0.0,
    specularIntensity: mat.specularIntensity ?? 1.0,
    specularColor: new THREE.Color(mat.specularColor || "#ffffff"),
    envMapIntensity: mat.envMapIntensity ?? 1.0,
    opacity: mat.opacity ?? 1.0,
    transparent: (mat.opacity ?? 1.0) < 1 || (mat.transmission ?? 0) > 0,
    transmission: mat.transmission ?? 0.0,
    ior: mat.ior ?? 1.45,
    thickness: mat.thickness ?? 0.0,
    attenuationColor: new THREE.Color(mat.attenuationColor || "#ffffff"),
    attenuationDistance: Math.max(0.01, mat.attenuationDistance ?? 1.0),
    iridescence: mat.iridescence ?? 0.0,
    iridescenceIOR: mat.ior ?? 1.45,
    emissive: new THREE.Color(mat.emissive || "#000000"),
    emissiveIntensity: mat.emissiveIntensity ?? 0.0,
    clearcoat: Math.max(mat.clearcoat ?? 0.0, hardness * 0.85),
    clearcoatRoughness: Math.min(mat.clearcoatRoughness ?? 0.0, 1 - hardness * 0.8),
    sheen: fluffiness,
    sheenRoughness: 0.9,
    sheenColor: new THREE.Color(mat.sheenColor || mat.color || "#808080"),
    side: mat.doubleSided === false ? THREE.FrontSide : THREE.DoubleSide,
    flatShading: mat.flatShading === true,
    wireframe: mat.wireframe === true,
    alphaTest: clamp01(mat.alphaCutoff, 0),
    depthWrite: (mat.opacity ?? 1.0) >= 1 && (mat.transmission ?? 0) <= 0,
  };
  if (mat.textureDataUrl) {
    let baseTex = _textureCache.get(mat.textureDataUrl);
    if (!baseTex) {
      baseTex = _textureLoader.load(mat.textureDataUrl);
      baseTex.colorSpace = THREE.SRGBColorSpace;
      baseTex.wrapS = baseTex.wrapT = THREE.RepeatWrapping;
      _textureCache.set(mat.textureDataUrl, baseTex);
    }
    const tex = baseTex.clone();
    tex.needsUpdate = true;
    tex.repeat.set(uv.repeatX ?? 1, uv.repeatY ?? 1);
    tex.offset.set(uv.offsetX ?? 0, uv.offsetY ?? 0);
    tex.rotation = ((uv.rotationDeg ?? 0) * Math.PI) / 180;
    tex.center.set(0.5, 0.5);
    const textureSoftness = clamp01(mat.textureSoftness, 0.25);
    const textureHardness = clamp01(mat.textureHardness, 0.5);
    const maxAniso = renderer?.capabilities?.getMaxAnisotropy?.() || 1;
    const targetAniso = Math.max(1, Math.round(1 + textureHardness * (maxAniso - 1)));
    tex.anisotropy = Math.max(1, Math.round(targetAniso * (1 - textureSoftness * 0.85)));
    tex.minFilter = textureSoftness > 0.6 ? THREE.LinearMipmapLinearFilter : THREE.LinearMipmapNearestFilter;
    tex.magFilter = textureSoftness > 0.75 ? THREE.LinearFilter : (textureHardness > 0.9 ? THREE.NearestFilter : THREE.LinearFilter);
    tex.generateMipmaps = true;
    params.map = tex;
  }
  return new THREE.MeshPhysicalMaterial(params);
}

function sanitizePrimitiveCutouts(cutouts) {
  if (!Array.isArray(cutouts)) return [];
  const out = [];
  for (const c of cutouts) {
    if (!c || typeof c !== "object") continue;
    if (!Array.isArray(c.targetToSourceMatrix) || c.targetToSourceMatrix.length !== 16) continue;
    const type = String(c.type || "");
    if (!["box", "sphere", "cylinder", "cone", "torus"].includes(type)) continue;
    out.push({
      type,
      targetToSourceMatrix: c.targetToSourceMatrix.map((n) => Number(n) || 0),
      dimensions: { ...(c.dimensions || {}) },
    });
    if (out.length >= 8) break;
  }
  return out;
}

function applyPrimitiveCutoutShader(mesh, primData) {
  if (!mesh?.material?.isMeshPhysicalMaterial) return;
  const cutouts = sanitizePrimitiveCutouts(primData?.cutouts);
  if (!cutouts.length) return;
  const mat = mesh.material;
  const maxCuts = 8;
  const cutMatrices = Array.from({ length: maxCuts }, () => new THREE.Matrix4());
  const cutA = Array.from({ length: maxCuts }, () => new THREE.Vector4(0, 0, 0, 0));
  const cutB = Array.from({ length: maxCuts }, () => new THREE.Vector4(0, 0, 0, 0));
  const typeCodeFor = (t) => (t === "sphere" ? 1 : t === "box" ? 2 : t === "cylinder" ? 3 : t === "cone" ? 4 : t === "torus" ? 5 : 0);
  for (let i = 0; i < cutouts.length && i < maxCuts; i++) {
    const c = cutouts[i];
    cutMatrices[i].fromArray(c.targetToSourceMatrix);
    const d = c.dimensions || {};
    switch (c.type) {
      case "sphere":
        cutA[i].set(Number(d.radius) || 0.5, 0, 0, typeCodeFor(c.type));
        break;
      case "box":
        cutA[i].set((Number(d.width) || 1) * 0.5, (Number(d.height) || 1) * 0.5, (Number(d.depth) || 1) * 0.5, typeCodeFor(c.type));
        break;
      case "cylinder":
      case "cone":
        cutA[i].set(Math.max(Number(d.radiusTop) || Number(d.radius) || 0.5, Number(d.radiusBottom) || Number(d.radius) || 0.5), Number(d.height) || 1, 0, typeCodeFor(c.type));
        break;
      case "torus":
        cutA[i].set(Number(d.radius) || 0.5, Number(d.tube) || 0.15, 0, typeCodeFor(c.type));
        break;
      default:
        break;
    }
  }
  mat.onBeforeCompile = (shader) => {
    shader.uniforms.uCutoutCount = { value: cutouts.length };
    shader.uniforms.uCutoutInv = { value: cutMatrices };
    shader.uniforms.uCutoutA = { value: cutA };
    shader.uniforms.uCutoutB = { value: cutB };
    shader.vertexShader = `
varying vec3 vPrimLocalPos;
${shader.vertexShader}`.replace(
      "#include <begin_vertex>",
      `#include <begin_vertex>
vPrimLocalPos = position;`
    );
    shader.fragmentShader = `
uniform int uCutoutCount;
uniform mat4 uCutoutInv[8];
uniform vec4 uCutoutA[8];
uniform vec4 uCutoutB[8];
varying vec3 vPrimLocalPos;

float sdfBox(vec3 p, vec3 b) {
  vec3 q = abs(p) - b;
  return length(max(q, 0.0)) + min(max(q.x, max(q.y, q.z)), 0.0);
}
float sdfSphere(vec3 p, float r) { return length(p) - r; }
float sdfCylinderY(vec3 p, float r, float h) {
  vec2 d = abs(vec2(length(p.xz), p.y)) - vec2(r, h * 0.5);
  return min(max(d.x, d.y), 0.0) + length(max(d, 0.0));
}
float sdfTorus(vec3 p, float r, float t) {
  vec2 q = vec2(length(p.xz) - r, p.y);
  return length(q) - t;
}
${shader.fragmentShader}`.replace(
      "#include <alphatest_fragment>",
      `#include <alphatest_fragment>
for (int i = 0; i < 8; i++) {
  if (i >= uCutoutCount) break;
  vec3 lp = (uCutoutInv[i] * vec4(vPrimLocalPos, 1.0)).xyz;
  float typ = uCutoutA[i].w;
  float d = 1e6;
  if (typ < 1.5) d = sdfSphere(lp, uCutoutA[i].x);
  else if (typ < 2.5) d = sdfBox(lp, vec3(uCutoutA[i].x, uCutoutA[i].y, uCutoutA[i].z));
  else if (typ < 3.5) d = sdfCylinderY(lp, uCutoutA[i].x, uCutoutA[i].y);
  else if (typ < 4.5) d = sdfCylinderY(lp, uCutoutA[i].x, uCutoutA[i].y);
  else d = sdfTorus(lp, uCutoutA[i].x, uCutoutA[i].y);
  if (d < 0.0) discard;
}`
    );
  };
  mat.customProgramCacheKey = () => `cutouts:${cutouts.length}:${cutouts.map((c) => c.type).join(",")}`;
  mat.needsUpdate = true;
}

function disposePrimitiveMaterial(material) {
  if (!material) return;
  const mats = Array.isArray(material) ? material : [material];
  for (const m of mats) {
    if (!m) continue;
    const maps = [
      "map",
      "alphaMap",
      "aoMap",
      "normalMap",
      "roughnessMap",
      "metalnessMap",
      "emissiveMap",
      "clearcoatMap",
      "clearcoatRoughnessMap",
      "transmissionMap",
      "thicknessMap",
    ];
    for (const key of maps) {
      const tex = m[key];
      if (tex?.isTexture) tex.dispose();
    }
    m.dispose?.();
  }
}

// Deferred collider queue — colliders are only created at a safe frame boundary
const _pendingColliderBuilds = [];

function flushPendingColliderBuilds() {
  if (!rapierWorld || !worldBody || _pendingColliderBuilds.length === 0) return;
  while (_pendingColliderBuilds.length > 0) {
    const prim = _pendingColliderBuilds.shift();
    // Verify the primitive still exists and still wants physics
    if (primitives.includes(prim) && prim.physics !== false) {
      rebuildPrimitiveColliderSync(prim);
    }
  }
}

function instantiatePrimitive(prim) {
  // Remove existing
  const existing = primitivesGroup.getObjectByName(`prim:${prim.id}`);
  if (existing) {
    existing.geometry?.dispose();
    disposePrimitiveMaterial(existing.material);
    primitivesGroup.remove(existing);
  }

  const geom = createPrimitiveGeometry(prim.type, prim.dimensions);
  const mat = createPrimitiveMaterial(prim.material);
  const mesh = new THREE.Mesh(geom, mat);
  applyPrimitiveCutoutShader(mesh, prim);
  mesh.name = `prim:${prim.id}`;
  mesh.userData.primitiveId = prim.id;
  mesh.userData.isPrimitive = true;
  // Default both to true — shapes should always participate in shadows
  mesh.castShadow = prim.castShadow !== false;
  mesh.receiveShadow = prim.receiveShadow !== false;

  const tr = prim.transform || {};
  if (tr.position) mesh.position.set(tr.position.x, tr.position.y, tr.position.z);
  if (tr.rotation) mesh.rotation.set(tr.rotation.x, tr.rotation.y, tr.rotation.z);
  if (tr.scale) mesh.scale.set(tr.scale.x ?? 1, tr.scale.y ?? 1, tr.scale.z ?? 1);

  primitivesGroup.add(mesh);

  // Build collider — if Rapier is ready, do it now; otherwise queue it
  if (prim.physics !== false) {
    if (rapierWorld && worldBody) {
      rebuildPrimitiveColliderSync(prim);
    } else {
      // Queue for deferred build once Rapier is ready
      _pendingColliderBuilds.push(prim);
      // Kick off Rapier init (non-blocking, collider will be built by flush)
      ensureRapierLoaded();
    }
  }
}

// Safely remove a primitive's existing collider from the Rapier world
function removePrimitiveCollider(prim) {
  if (prim._colliderHandle == null || !rapierWorld) return;
  try {
    if (typeof prim._colliderHandle === "object" && prim._colliderHandle.handle !== undefined) {
      rapierWorld.removeCollider(prim._colliderHandle, true);
    }
  } catch (e) {
    console.warn(`[COLLIDER] Primitive collider remove failed for ${prim.id}:`, e);
  }
  prim._colliderHandle = null;
}

// SYNCHRONOUS collider creation for native Rapier shapes.
// Only falls back to async for trimesh (torus, plane).
function rebuildPrimitiveColliderSync(prim) {
  if (!prim) return;
  // Rapier must already be loaded for sync creation
  if (!rapierWorld || !RAPIER || !worldBody) return;

  removePrimitiveCollider(prim);
  if (prim.physics === false) return;

  const mesh = primitivesGroup.getObjectByName(`prim:${prim.id}`);
  if (!mesh) return;

  const dims = prim.dimensions || {};
  const s = prim.transform?.scale || { x: 1, y: 1, z: 1 };
  const pos = prim.transform?.position || { x: 0, y: 0, z: 0 };
  const rot = prim.transform?.rotation || { x: 0, y: 0, z: 0 };

  // Clamp all half-extents / radii to a safe minimum to avoid WASM traps
  const clamp = (v) => Math.max(v, 0.001);

  let desc = null;

  // Use native Rapier collision shapes – far more compute-efficient than trimesh
  switch (prim.type) {
    case "box":
      desc = RAPIER.ColliderDesc.cuboid(
        clamp(((dims.width || 1) * (s.x ?? 1)) / 2),
        clamp(((dims.height || 1) * (s.y ?? 1)) / 2),
        clamp(((dims.depth || 1) * (s.z ?? 1)) / 2)
      );
      break;
    case "sphere":
      desc = RAPIER.ColliderDesc.ball(
        clamp((dims.radius || 0.5) * Math.max(s.x ?? 1, s.y ?? 1, s.z ?? 1))
      );
      break;
    case "cylinder":
      desc = RAPIER.ColliderDesc.cylinder(
        clamp(((dims.height || 1) * (s.y ?? 1)) / 2),
        clamp(Math.max(dims.radiusTop ?? 0.5, dims.radiusBottom ?? 0.5) * Math.max(s.x ?? 1, s.z ?? 1))
      );
      break;
    case "cone":
      desc = RAPIER.ColliderDesc.cone(
        clamp(((dims.height || 1) * (s.y ?? 1)) / 2),
        clamp((dims.radius || 0.5) * Math.max(s.x ?? 1, s.z ?? 1))
      );
      break;
    case "plane": {
      // PlaneGeometry lies in the XY plane (normal along +Z), so make the
      // cuboid thin in Z to match the visual exactly. No rotation offset needed.
      const pw = clamp(((dims.width || 2) * (s.x ?? 1)) / 2);
      const ph = clamp(((dims.height || 2) * (s.y ?? 1)) / 2);
      desc = RAPIER.ColliderDesc.cuboid(pw, ph, 0.005);
      break; // fall through to the standard rotation/translation below
    }
    case "torus": {
      // Torus: use trimesh async fallback (deferred, won't block)
      rebuildPrimitiveColliderAsync(prim);
      return;
    }
    default:
      return;
  }

  if (desc) {
    desc.setTranslation(pos.x, pos.y, pos.z);
    const euler = new THREE.Euler(rot.x, rot.y, rot.z);
    const quat = new THREE.Quaternion().setFromEuler(euler);
    desc.setRotation({ x: quat.x, y: quat.y, z: quat.z, w: quat.w });
    desc.setFriction(0.9);
    try {
      const collider = rapierWorld.createCollider(desc);
      prim._colliderHandle = collider;
    } catch (e) {
      console.warn(`[COLLIDER] Failed to create primitive collider for ${prim.type}:`, e);
    }
  }
}

// Async fallback only used for torus (trimesh)
async function rebuildPrimitiveColliderAsync(prim) {
  if (!prim) return;
  await ensureRapierLoaded();
  if (!rapierWorld || !RAPIER || !worldBody) return;
  removePrimitiveCollider(prim);
  if (prim.physics === false) return;
  const mesh = primitivesGroup.getObjectByName(`prim:${prim.id}`);
  if (!mesh) return;
  try {
    const collider = await buildRapierTriMeshColliderFromObject(mesh);
    if (collider) prim._colliderHandle = collider;
  } catch (e) {
    console.warn(`[COLLIDER] Trimesh fallback failed for ${prim.id}:`, e);
  }
}

// Keep old name as alias for callers (e.g. dimension/transform change handlers)


function getPlacementAtCrosshair({ raycastDistance = 500, fallbackDistance = 3, surfaceOffset = 0.02 } = {}) {
  const hit = rapierRaycastFromCamera(raycastDistance);
  if (hit) {
    const n = hit.normal
      ? new THREE.Vector3(hit.normal.x, hit.normal.y, hit.normal.z).normalize()
      : new THREE.Vector3(0, 1, 0);
    return {
      hit: true,
      point: { x: hit.point.x, y: hit.point.y, z: hit.point.z },
      normal: { x: n.x, y: n.y, z: n.z },
      position: {
        x: hit.point.x + n.x * surfaceOffset,
        y: hit.point.y + n.y * surfaceOffset,
        z: hit.point.z + n.z * surfaceOffset,
      },
    };
  }

  // If no collider is hit, place directly in front of the crosshair.
  const dir = camera.getWorldDirection(new THREE.Vector3());
  const p = camera.getWorldPosition(new THREE.Vector3());
  return {
    hit: false,
    point: {
      x: p.x + dir.x * fallbackDistance,
      y: p.y + dir.y * fallbackDistance,
      z: p.z + dir.z * fallbackDistance,
    },
    normal: { x: 0, y: 1, z: 0 },
    position: {
      x: p.x + dir.x * fallbackDistance,
      y: p.y + dir.y * fallbackDistance,
      z: p.z + dir.z * fallbackDistance,
    },
  };
}


function rebuildAllPrimitives() {
  // Remove all existing colliders first
  for (const p of primitives) {
    removePrimitiveCollider(p);
  }
  // Remove all visual meshes
  while (primitivesGroup.children.length) {
    const c = primitivesGroup.children[0];
    c.geometry?.dispose();
    disposePrimitiveMaterial(c.material);
    primitivesGroup.remove(c);
  }
  // Rebuild all
  for (const p of primitives) {
    try {
      instantiatePrimitive(p);
    } catch (e) {
      console.warn("Failed to rebuild primitive", p.id, e);
    }
  }
}

// =============================================================================
// EDITOR LIGHTS – User-placed lights with visible proxy icons
// =============================================================================

function instantiateEditorLight(lightData) {
  // Remove existing
  removeEditorLightObjects(lightData.id);

  let lightObj;
  const color = new THREE.Color(lightData.color || "#ffffff");
  const intensity = lightData.intensity ?? 1.0;

  switch (lightData.type) {
    case "point":
      lightObj = new THREE.PointLight(color, intensity, lightData.distance || 0);
      break;
    case "spot": {
      lightObj = new THREE.SpotLight(
        color,
        intensity,
        lightData.distance || 0,
        lightData.angle ?? Math.PI / 4,
        lightData.penumbra ?? 0.1
      );
      const tgt = lightData.target || { x: 0, y: 0, z: 0 };
      lightObj.target.position.set(tgt.x, tgt.y, tgt.z);
      lightsGroup.add(lightObj.target);
      break;
    }
    case "directional":
    default: {
      lightObj = new THREE.DirectionalLight(color, intensity);
      const tgt = lightData.target || { x: 0, y: 0, z: 0 };
      lightObj.target.position.set(tgt.x, tgt.y, tgt.z);
      lightsGroup.add(lightObj.target);
      break;
    }
  }

  const pos = lightData.position || { x: 5, y: 10, z: 5 };
  lightObj.position.set(pos.x, pos.y, pos.z);
  lightObj.castShadow = lightData.castShadow ?? false;
  lightObj.name = `light:${lightData.id}`;
  lightObj.userData.editorLightId = lightData.id;
  lightObj.userData.isEditorLight = true;

  // Configure shadow map for this light (only renders when castShadow=true).
  // Use 512 for point/spot (6-face cubemap = expensive) and 1024 for directional.
  if (lightObj.shadow) {
    const res = lightObj.isDirectionalLight ? 1024 : 512;
    lightObj.shadow.mapSize.width = res;
    lightObj.shadow.mapSize.height = res;
    lightObj.shadow.bias = -0.003;
    if (lightObj.shadow.camera) {
      if (lightObj.isDirectionalLight) {
        lightObj.shadow.camera.near = 0.5;
        lightObj.shadow.camera.far = 50;
        lightObj.shadow.camera.left = -20;
        lightObj.shadow.camera.right = 20;
        lightObj.shadow.camera.top = 20;
        lightObj.shadow.camera.bottom = -20;
      } else {
        lightObj.shadow.camera.near = 0.5;
        lightObj.shadow.camera.far = Math.min(lightData.distance || 30, 30);
      }
    }
  }

  lightsGroup.add(lightObj);
  lightData._lightObj = lightObj;
  lightData._proxyObj = null;
  lightData._helperObj = null;
}

function removeEditorLightObjects(id) {
  const names = [`light:${id}`, `lightHelper:${id}`, `lightProxy:${id}`];
  for (const n of names) {
    const obj = lightsGroup.getObjectByName(n);
    if (obj) {
      // Remove target if it exists (directional/spot)
      if (obj.target && obj.target.parent) obj.target.parent.remove(obj.target);
      // Dispose children meshes
      obj.traverse?.((c) => {
        if (c.geometry) c.geometry.dispose();
        if (c.material) {
          if (c.material.map) c.material.map.dispose();
          c.material.dispose();
        }
      });
      lightsGroup.remove(obj);
    }
  }
}


function rebuildAllEditorLights() {
  // Remove all light objects
  while (lightsGroup.children.length) {
    const c = lightsGroup.children[0];
    c.traverse?.((m) => { m.geometry?.dispose(); m.material?.dispose(); });
    lightsGroup.remove(c);
  }
  for (const ld of editorLights) {
    ld._lightObj = null;
    ld._helperObj = null;
    ld._proxyObj = null;
    try {
      instantiateEditorLight(ld);
    } catch (e) {
      console.warn("Failed to rebuild light", ld.id, e);
    }
  }
  // Enable/disable the renderer shadow map based on whether any light casts shadows
  syncShadowMapEnabled();
}

// =============================================================================
// DETAILS PANEL & TRANSFORM XYZ – UE-style unified properties
// =============================================================================

const RAD2DEG = 180 / Math.PI;
const DEG2RAD = Math.PI / 180;

// Dynamically enable/disable the shadow map system.
// When no light casts shadows, the renderer skips ALL shadow work (zero overhead).
function enforceShadowSamplerBudget() {
  // Prevent WebGL shader validation failures:
  // "texture image units count exceeds MAX_TEXTURE_IMAGE_UNITS"
  // Point-light shadows are especially expensive (cube map = ~6 samplers).
  const budget = 8;
  const costFor = (lightObj) => (lightObj?.isPointLight ? 6 : 1);

  const candidates = [];
  for (const sl of sceneLights) {
    if (!sl?.obj || sl.obj.visible === false) continue;
    if (!sl.obj.castShadow) continue;
    candidates.push({ obj: sl.obj, source: "scene", meta: sl });
  }
  for (const ld of editorLights) {
    if (!ld?._lightObj || ld._lightObj.visible === false) continue;
    if (!ld._lightObj.castShadow) continue;
    candidates.push({ obj: ld._lightObj, source: "editor", meta: ld });
  }

  // Prefer non-point shadow lights first (directional/spot), then points.
  candidates.sort((a, b) => {
    const ac = costFor(a.obj);
    const bc = costFor(b.obj);
    if (ac !== bc) return ac - bc; // cheaper first
    return 0;
  });

  let used = 0;
  for (const c of candidates) {
    const cost = costFor(c.obj);
    if (used + cost <= budget) {
      used += cost;
      continue;
    }
    c.obj.castShadow = false;
    // keep data model consistent so UI reflects actual runtime state
    if (c.source === "editor") c.meta.castShadow = false;
  }
}

function syncShadowMapEnabled() {
  enforceShadowSamplerBudget();
  let anyCast = false;
  // Check scene lights
  for (const sl of sceneLights) {
    if (sl.obj?.castShadow && sl.obj?.visible !== false) { anyCast = true; break; }
  }
  // Check editor lights
  if (!anyCast) {
    for (const ld of editorLights) {
      if (ld._lightObj?.castShadow && ld._lightObj?.visible !== false) { anyCast = true; break; }
    }
  }
  // enableShadows() forces shadows on even though scene lights aren't in the arrays above.
  const want = anyCast || renderer.shadowMap.__dimsimForced === true;
  if (renderer.shadowMap.enabled !== want) {
    renderer.shadowMap.enabled = want;
    // When toggling shadow maps, Three.js needs to recompile materials
    scene.traverse((obj) => { if (obj.material) obj.material.needsUpdate = true; });
  }
  if (want) renderer.shadowMap.needsUpdate = true;
}

function renderSceneInMode(mode) {
  const savedOverride = scene.overrideMaterial;
  const savedBg = scene.background;
  const savedAssets = assetsGroup.visible;
  const savedPrims = primitivesGroup.visible;
  const savedLights = lightsGroup.visible;
  const savedTags = tagsGroup.visible;
  const savedLidar = lidarVizGroup.visible;

  if (mode === "rgb") {
    scene.overrideMaterial = null;
    assetsGroup.visible = true;
    primitivesGroup.visible = true;
    lightsGroup.visible = true;
    tagsGroup.visible = false;
    lidarVizGroup.visible = false;
    scene.background = DEFAULT_SCENE_BG;
    renderer.render(scene, camera);
  } else if (mode === "lidar") {
    scene.overrideMaterial = null;
    assetsGroup.visible = false;
    primitivesGroup.visible = false;
    lightsGroup.visible = false;
    tagsGroup.visible = false;
    lidarVizGroup.visible = true;
    scene.background = RGBD_BG;
    renderer.render(scene, camera);
  }

  scene.overrideMaterial = savedOverride;
  scene.background = savedBg;
  assetsGroup.visible = savedAssets;
  primitivesGroup.visible = savedPrims;
  lightsGroup.visible = savedLights;
  tagsGroup.visible = savedTags;
  lidarVizGroup.visible = savedLidar;
}

function renderCompareViews() {
  const sz = renderer.getSize(new THREE.Vector2());
  const W = sz.x;
  const H = sz.y;
  const halfW = Math.floor(W / 2);
  const halfH = Math.floor(H / 2);

  renderer.setScissorTest(true);
  renderer.autoClear = false;

  renderer.setViewport(0, 0, W, H);
  renderer.setScissor(0, 0, W, H);
  renderer.setClearColor(0x000000, 1);
  renderer.clear(true, true, true);

  // Top-left: RGB
  renderer.setViewport(0, halfH, halfW, halfH);
  renderer.setScissor(0, halfH, halfW, halfH);
  renderer.setClearColor(DEFAULT_SCENE_BG, 1);
  renderer.clear(true, true, true);
  renderSceneInMode("rgb");

  // Top-right: RGB-D
  renderRgbdMetricPassOffscreen();
  rgbdVizMaterial.uniforms.uGrayMode.value = rgbdVizMode === "gray" ? 1.0 : 0.0;
  renderer.setRenderTarget(null);
  renderer.setViewport(halfW, halfH, W - halfW, halfH);
  renderer.setScissor(halfW, halfH, W - halfW, halfH);
  renderer.setClearColor(RGBD_BG, 1);
  renderer.clear(true, true, true);
  renderer.render(rgbdVizScene, rgbdPostCamera);

  // Bottom-center: LiDAR
  const lidarX = Math.floor((W - halfW) / 2);
  renderer.setViewport(lidarX, 0, halfW, halfH);
  renderer.setScissor(lidarX, 0, halfW, halfH);
  renderer.setClearColor(RGBD_BG, 1);
  renderer.clear(true, true, true);
  renderSceneInMode("lidar");

  renderer.setScissorTest(false);
  renderer.autoClear = true;
  renderer.setViewport(0, 0, W, H);
  renderer.setScissor(0, 0, W, H);
}

function renderActiveView() {
  syncShadowMapEnabled();
  if (simCompareView) {
    renderCompareViews();
  } else if (simSensorViewMode === "rgbd") {
    renderRgbdView();
  } else {
    renderer.render(scene, camera);
  }
}

const _tmpCamPos = new THREE.Vector3();
const _tmpCamDir = new THREE.Vector3();
const _raycaster = new THREE.Raycaster();

function rapierRaycastFromCamera(maxToi = 250) {
  if (!rapierWorld || !RAPIER) return null;
  // Query pipeline is kept current by rapierWorld.step() in updateRapier

  const o = camera.getWorldPosition(_tmpCamPos);
  const d = camera.getWorldDirection(_tmpCamDir).normalize();

  const ray = new RAPIER.Ray({ x: o.x, y: o.y, z: o.z }, { x: d.x, y: d.y, z: d.z });
  const hit = rapierWorld.queryPipeline.castRayAndGetNormal(
    rapierWorld.bodies,
    rapierWorld.colliders,
    ray,
    maxToi,
    false, // hollow: can hit boundary even if ray starts inside
    RAPIER.QueryFilterFlags.EXCLUDE_SENSORS,
    undefined,
    playerCollider?.handle
  );
  if (!hit) return null;
  const toi = hit.toi ?? hit.timeOfImpact ?? 0;
  const p = { x: o.x + d.x * toi, y: o.y + d.y * toi, z: o.z + d.z * toi };
  const n = hit.normal ? { x: hit.normal.x, y: hit.normal.y, z: hit.normal.z } : null;
  return { point: p, normal: n, colliderHandle: hit.colliderHandle ?? null, toi };
}

function isShapeFreeAt(shape, rot, pos, excludeColliderHandle = null) {
  if (!rapierWorld || !RAPIER) return false;
  const hit = rapierWorld.queryPipeline.intersectionWithShape(
    rapierWorld.bodies,
    rapierWorld.colliders,
    pos,
    rot,
    shape,
    RAPIER.QueryFilterFlags.EXCLUDE_SENSORS,
    undefined,
    excludeColliderHandle
  );
  return hit == null;
}

function findNearbyFreeSpotForCollider(collider, startPos, maxR = 2.0, step = 0.12) {
  if (!collider) return null;
  const shape = collider.shape;
  const rot = collider.rotation();
  const exclude = collider.handle;

  if (isShapeFreeAt(shape, rot, startPos, exclude)) return startPos;

  const dirs = [
    [1, 0, 0],
    [-1, 0, 0],
    [0, 0, 1],
    [0, 0, -1],
    [1, 0, 1],
    [1, 0, -1],
    [-1, 0, 1],
    [-1, 0, -1],
    [0, 1, 0],
    [0, -1, 0],
  ];
  for (let r = step; r <= maxR; r += step) {
    for (const [dx, dy, dz] of dirs) {
      const len = Math.hypot(dx, dy, dz) || 1;
      const pos = { x: startPos.x + (dx / len) * r, y: startPos.y + (dy / len) * r, z: startPos.z + (dz / len) * r };
      if (isShapeFreeAt(shape, rot, pos, exclude)) return pos;
    }
  }
  return null;
}

function removeAiAgent(agent, reason = "manual") {
  if (!agent) return;
  const removedId = String(agent.id || "");
  try {
    aiAgents = aiAgents.filter((a) => a !== agent);
    agentUiPush(`${new Date().toLocaleTimeString()}\nAGENT DESPAWN\n${agent.id} (${reason})`);
    agent.dispose?.();
  } catch {}
  if (removedId) {
    _agentTasks.delete(removedId);
    agentInspectorStateById.delete(removedId);
  }
  if (removedId) removeAgentBadge(removedId);
  if (agentCameraFollowId === removedId) {
    disableAgentCameraFollow();
  }
  if (selectedAgentInspectorId === removedId) {
    selectedAgentInspectorId = aiAgents[0]?.id || null;
    if (selectedAgentInspectorId) renderAgentInspector(selectedAgentInspectorId);
    else {
      clearAgentInspectorViews();
    }
  }
  if (aiAgents.length === 0) {
    disableAgentCameraFollow();
  }
  renderAgentTaskUi();
}

function stopAiAgent(agent, reason = "manual-stop") {
  if (!agent) return;
  agentUiPush(`${new Date().toLocaleTimeString()}\nAGENT STOP\n${agent.id} (${reason})`);
  renderAgentTaskUi();
}


function createAiAgent({ ephemeral = false, avatarUrl, radius, halfHeight } = {}) {
  const id = `agent-${Date.now().toString(36)}-${Math.random().toString(36).slice(2, 6)}`;
  const _avatarUrl = avatarUrl !== undefined
    ? avatarUrl
    : ["/embodiment/dimsim_unitree_stub.glb"];
  const agent = new AiAvatar({
    id,
    scene,
    rapierWorld,
    RAPIER,
    avatarUrl: _avatarUrl,
    radius,
    halfHeight,
    // Headless mode in dimos: skip visual rendering, keep colliders for physics
    headless: false,
  });
  agent._ephemeral = !!ephemeral;
  // Manually spawned editor agents should clean themselves up after task completion.
  agent._autoDespawnAfterTask = true;
  // Only inherit the active task if this agent was spawned as part of a worker pool (ephemeral).
  // Manually spawned agents start idle and wait for their own task assignment.
  agent._taskStartedAt = ephemeral ? Number(agentTask.startedAt || 0) : 0;
  getOrCreateAgentInspectorState(id);
  if (!selectedAgentInspectorId) selectedAgentInspectorId = id;
  renderSelectedAgentControls();
  return agent;
}


async function ensureRapierLoaded() {
  if (RAPIER) return;
  if (!_rapierInitPromise) {
    _rapierInitPromise = _doRapierInit();
  }
  return _rapierInitPromise;
}

async function _doRapierInit() {
  RAPIER = await import("@dimforge/rapier3d-compat");
  await RAPIER.init();
  rapierWorld = new RAPIER.World({ x: 0, y: -9.81, z: 0 });
  worldBody = rapierWorld.createRigidBody(RAPIER.RigidBodyDesc.fixed());

  const radius = PLAYER_RADIUS;
  const halfHeight = PLAYER_HALF_HEIGHT;
  playerBody = rapierWorld.createRigidBody(
    RAPIER.RigidBodyDesc.kinematicPositionBased().setTranslation(0, 3, 0)
  );
  playerCollider = rapierWorld.createCollider(
    RAPIER.ColliderDesc.capsule(halfHeight, radius).setFriction(0.0).setSensor(ghostMode),
    playerBody
  );

  characterController = rapierWorld.createCharacterController(0.02);
  characterController.setSlideEnabled(true);
  characterController.enableAutostep(0.55, 0.25, true);
  characterController.enableSnapToGround(0.25);
  characterController.setMaxSlopeClimbAngle(Math.PI / 3);
  characterController.setMinSlopeSlideAngle(Math.PI / 2);
}

async function spawnOrMoveAiAtAim({ createNew = false, silent = false, ephemeral = false } = {}) {
  await ensureRapierLoaded();
  const hit = rapierRaycastFromCamera(500);
  const placement = hit
    ? { point: hit.point, normal: hit.normal || { x: 0, y: 1, z: 0 } }
    : getPlacementAtCrosshair({ raycastDistance: 500, fallbackDistance: 3, surfaceOffset: 0.0 });
  if (!hit && !silent) {
    setStatus("No collider hit; spawned AI using crosshair fallback placement.");
  }

  let agent = createNew ? null : aiAgents[0] || null;
  if (!agent) {
    if (aiAgents.length >= MAX_AGENT_COUNT) {
      if (!silent) setStatus(`Agent cap reached (${MAX_AGENT_COUNT}).`);
      return;
    }
    agent = createAiAgent({ ephemeral });
    aiAgents.push(agent);
  } else if (ephemeral) {
    agent._ephemeral = true;
  }

  const n = placement.normal
    ? new THREE.Vector3(placement.normal.x, placement.normal.y, placement.normal.z).normalize()
    : new THREE.Vector3(0, 1, 0);
  const offset = Math.max(0.12, (agent.radius ?? PLAYER_RADIUS) + 0.06);
  const p0 = placement.point;
  const candA = { x: p0.x + n.x * offset, y: p0.y + n.y * offset, z: p0.z + n.z * offset };
  const candB = { x: p0.x - n.x * offset, y: p0.y - n.y * offset, z: p0.z - n.z * offset };

  let chosen = null;
  chosen = findNearbyFreeSpotForCollider(agent.collider, candA, 2.0, 0.12);
  if (!chosen) chosen = findNearbyFreeSpotForCollider(agent.collider, candB, 2.0, 0.12);
  if (!chosen) chosen = findNearbyFreeSpotForCollider(agent.collider, { x: p0.x, y: p0.y + offset, z: p0.z }, 2.5, 0.12);
  // Fallback: use placement point directly (slightly above surface) rather than failing silently.
  if (!chosen) {
    chosen = { x: p0.x + n.x * 0.5, y: p0.y + n.y * 0.5 + 0.5, z: p0.z + n.z * 0.5 };
    console.warn("[Spawn] No free collision spot found – using direct fallback placement at", chosen);
  }

  agent.setPosition(chosen.x, chosen.y, chosen.z);
  if (agentTask.active) {
    agent._taskStartedAt = agentTask.startedAt;
  }
  renderAgentTaskUi();
  if (!silent) {
    const label = createNew ? "AI worker spawned." : "AI placed.";
    setStatus(`${label} (${aiAgents.length} total)`);
  }
}


function removeAgentBadge(agentId) {
  const id = String(agentId || "");
  const el = agentBadgeElsById.get(id);
  if (el?.parentElement) el.parentElement.removeChild(el);
  agentBadgeElsById.delete(id);
}


function pickTagMarkerFromCamera() {
  _raycaster.setFromCamera({ x: 0, y: 0 }, camera);
  const hits = _raycaster.intersectObjects(tagsGroup.children, false);
  for (const h of hits) {
    const obj = h.object;
    if (obj?.userData?.isRadius) continue;
    const id = obj?.userData?.tagId;
    if (id) return id;
  }
  return null;
}


// Lock pointer and interact on click for FPS navigation.
canvas.addEventListener("click", async (e) => {
  if (!controls.isLocked) {
    controls.enabled = true;
    try { controls.lock(); } catch {}
  } else if (e.button === 0) {
    await handlePlayerInteraction();
  }
});

// Right-click to lock pointer (for FPS navigation)
canvas.addEventListener("contextmenu", (e) => {
  e.preventDefault();
  if (!controls.isLocked) {
    controls.enabled = true;
    try { controls.lock(); } catch {}
  }
});

controls.addEventListener("lock", () => {
  controls.enabled = true;
});
controls.addEventListener("unlock", () => {
  setStatus("Click to look around.");
});


function setGhostMode(enabled) {
  ghostMode = !!enabled;
  // Ghost mode indicator shown in status
  if (enabled) setStatus("Ghost mode ON");

  // Disable collisions by turning the player collider into a sensor.
  // (Sensors don't generate contact forces, so you can pass through walls.)
  try {
    if (playerCollider && typeof playerCollider.setSensor === "function") {
      playerCollider.setSensor(ghostMode);
    }
  } catch {
    // ignore
  }
}

// Ghost mode toggled via 'G' key only

// Tagging UI
document.documentElement.dataset.mode = "sim";
simPanelCollapseBtn?.addEventListener("click", () => {
  simPanelCollapsed = true;
  applySimPanelCollapsedState();
});
simPanelOpenBtn?.addEventListener("click", () => {
  simPanelCollapsed = false;
  applySimPanelCollapsedState();
});
simCameraModeToggleBtn?.addEventListener("click", () => {
  simUserCameraMode = simUserCameraMode === "user" ? "agent" : "user";
  localStorage.setItem("sparkWorldSimCameraMode", simUserCameraMode);
  updateSimCameraModeToggleUi();
  if (simUserCameraMode === "user") {
    if (agentCameraFollow) disableAgentCameraFollow();
  } else if (agentTask.active) {
    enableAgentCameraFollow();
  }
});
simViewRgbdBtn?.addEventListener("click", () => {
  simCompareView = false;
  setSimSensorViewMode("rgbd");
});
simRgbdGrayBtn?.addEventListener("click", () => {
  rgbdVizMode = "gray";
  updateSimSensorButtons();
  if (simSensorViewMode === "rgbd") setStatus("RGB-D: metric grayscale");
});
simRgbdColormapBtn?.addEventListener("click", () => {
  rgbdVizMode = "colormap";
  updateSimSensorButtons();
  if (simSensorViewMode === "rgbd") setStatus("RGB-D: metric colormap");
});
simRgbdAutoRangeBtn?.addEventListener("click", () => {
  rgbdAutoRange = !rgbdAutoRange;
  updateSimSensorButtons();
  if (simSensorViewMode === "rgbd") setStatus(rgbdAutoRange ? "RGB-D auto-range ON (p5/p95)" : "RGB-D auto-range OFF");
});
simRgbdNoiseBtn?.addEventListener("click", () => {
  rgbdNoiseEnabled = !rgbdNoiseEnabled;
  updateSimSensorButtons();
  setStatus(rgbdNoiseEnabled ? "RGB-D noise ON" : "RGB-D noise OFF");
});
simRgbdSpeckleBtn?.addEventListener("click", () => {
  rgbdSpeckleEnabled = !rgbdSpeckleEnabled;
  updateSimSensorButtons();
  setStatus(rgbdSpeckleEnabled ? "RGB-D speckle ON" : "RGB-D speckle OFF");
});
simRgbdMinEl?.addEventListener("input", () => {
  if (rgbdAutoRange) return;
  const minV = Number(simRgbdMinEl.value);
  const maxV = Number(simRgbdMaxEl?.value ?? rgbdRangeMaxM);
  setRgbdRange(minV, maxV);
});
simRgbdMaxEl?.addEventListener("input", () => {
  if (rgbdAutoRange) return;
  const minV = Number(simRgbdMinEl?.value ?? rgbdRangeMinM);
  const maxV = Number(simRgbdMaxEl.value);
  setRgbdRange(minV, maxV);
});
simViewLidarBtn?.addEventListener("click", () => {
  // Main LiDAR button always maps to accumulated unordered 3D point cloud.
  simCompareView = false;
  lidarOrderedDebugView = false;
  if (simSensorViewMode !== "lidar") {
    _lidarAccumFrames.length = 0;
    _lidarLastAccumPose = null;
    resetLidarScanState();
  }
  setSimSensorViewMode("lidar");
});
simViewCompareBtn?.addEventListener("click", () => {
  simCompareView = !simCompareView;
  if (simCompareView) {
    // Auto-collapse panel so tiles get full canvas width.
    simPanelCollapsed = true;
    applySimPanelCollapsedState();
    simSensorViewMode = "lidar";
    lidarOrderedDebugView = false;
    setStatus("Compare view: RGB | RGB-D | LiDAR");
  } else {
    simPanelCollapsed = false;
    applySimPanelCollapsedState();
    setStatus("Compare view OFF");
  }
  applySimSensorViewMode();
});
simLidarColorRangeBtn?.addEventListener("click", () => {
  lidarColorByRange = !lidarColorByRange;
  updateSimSensorButtons();
  if (simSensorViewMode === "lidar") {
    updateLidarPointCloud();
    setStatus(lidarColorByRange ? "LiDAR: range-color mode" : "LiDAR: intensity mode");
  }
});
simLidarOrderedDebugBtn?.addEventListener("click", () => {
  // Single Sweep is the explicit ring/scan debug view.
  lidarOrderedDebugView = true;
  _lidarAccumFrames.length = 0;
  _lidarLastAccumPose = null;
  resetLidarScanState();
  if (simSensorViewMode !== "lidar") simSensorViewMode = "lidar";
  updateSimSensorButtons();
  applySimSensorViewMode();
  setStatus("LiDAR: single sweep view");
});
simLidarNoiseBtn?.addEventListener("click", () => {
  lidarNoiseEnabled = !lidarNoiseEnabled;
  _lidarAccumFrames.length = 0;
  _lidarLastAccumPose = null;
  resetLidarScanState();
  updateSimSensorButtons();
  if (simSensorViewMode === "lidar") updateLidarPointCloud();
  setStatus(lidarNoiseEnabled ? "LiDAR noise ON" : "LiDAR noise OFF");
});
simLidarMultiReturnBtn?.addEventListener("click", () => {
  lidarMultiReturnMode = lidarMultiReturnMode === "strongest" ? "last" : "strongest";
  _lidarAccumFrames.length = 0;
  _lidarLastAccumPose = null;
  resetLidarScanState();
  updateSimSensorButtons();
  if (simSensorViewMode === "lidar") updateLidarPointCloud();
  setStatus(`LiDAR return mode: ${lidarMultiReturnMode}`);
});

// --- Blob shadow live-adjustment helpers ---
// Updates the blob shadow mesh in-place without rebuilding the entire asset.


// Initialize agent UI visibility/content.
applySimPanelCollapsedState();
renderAgentTaskUi();
agentTaskStartBtn?.addEventListener("click", () => {
  if (agentTask.active) return;
  void startAgentTask(agentTaskInputEl?.value);
});
agentTaskEndBtn?.addEventListener("click", () => endAgentTask("manual"));
// Enter key in command input starts task; stop propagation so WASD doesn't trigger
agentTaskInputEl?.addEventListener("keydown", (e) => {
  e.stopPropagation();
  if (e.key === "Enter" && !agentTask.active && aiAgents.length > 0) {
    void startAgentTask(agentTaskInputEl.value);
  }
});

// Shared import logic — used by both editor Import and sim Load Level
async function importLevelFromJSON(json, options = {}) {
  const importedTags = Array.isArray(json?.tags) ? json.tags : Array.isArray(json) ? json : null;
  const preserveAssetsWhenMissing = options.preserveAssetsWhenMissing === true;
  const importedAssets = Array.isArray(json?.assets)
    ? json.assets
    : (preserveAssetsWhenMissing ? assets : []);
  const importedPrimitives = Array.isArray(json?.primitives) ? json.primitives : [];
  const importedLights = Array.isArray(json?.lights) ? json.lights : [];
  const importedSceneSettings = json && typeof json === "object" && json.sceneSettings
    ? normalizeSceneSettings(json.sceneSettings)
    : null;
  if (!importedTags) throw new Error("Invalid level file.");
  // Clean up old primitive colliders
  for (const p of primitives) removePrimitiveCollider(p);
  tags = importedTags;
  assets = importedAssets;
  primitives = importedPrimitives;
  editorLights = importedLights;
  if (importedSceneSettings) sceneSettings = importedSceneSettings;
  if (!options.skipWorldSave) saveTagsForWorld();
  rebuildTagMarkers();
  await rebuildAssets();
  rebuildAllPrimitives();
  rebuildAllEditorLights();
  applySceneSkySettings();
  applySceneRgbBackground();
  syncShadowMapEnabled();
}


// Sim-mode "Load Level JSON" input
const simLevelImportEl = document.getElementById("sim-level-import");
simLevelImportEl?.addEventListener("change", async (e) => {
  const file = e.target.files?.[0];
  if (!file) return;
  try {
    setStatus("Loading level...");
    const text = await file.text();
    await importLevelFromJSON(JSON.parse(text));
    await ensureRapierLoaded();
    spawnPlayerInsideScene();
    setStatus("Level loaded. Click to enter, then spawn an agent.");
  } catch (err) {
    console.error(err);
    setStatus(err?.message || "Failed to load level.");
  } finally {
    e.target.value = "";
  }
});

canvas?.addEventListener("mousedown", () => {
  const id = pickTagMarkerFromCamera();
  if (!id) return;
  selectedTagId = id;
  draftTag = null;
  updateMarkerMaterials();
});


// =============================================================================
// PRIMITIVE & LIGHT EVENT HANDLERS
// =============================================================================


function teleportPlayerTo(x, y, z) {
  if (!playerBody) return;
  playerBody.setTranslation({ x, y, z }, true);
  playerBody.setLinvel({ x: 0, y: 0, z: 0 }, true);
}

// Find a reasonable interior floor Y by casting a ray straight down through the
// loaded scene meshes and picking the lowest up-facing surface. This skips the
// roof when a building has both a roof and an interior floor, so the player
// lands inside rather than on top.
function _findSceneFloorY(x = 0, z = 0) {
  const bbox = new THREE.Box3();
  const tmp = new THREE.Box3();
  try { tmp.setFromObject(assetsGroup); if (tmp.isEmpty() === false) bbox.union(tmp); } catch {}
  try { tmp.setFromObject(primitivesGroup); if (tmp.isEmpty() === false) bbox.union(tmp); } catch {}
  const fromY = bbox.isEmpty() ? 50 : bbox.max.y + 5;
  const raycaster = new THREE.Raycaster(
    new THREE.Vector3(x, fromY, z),
    new THREE.Vector3(0, -1, 0),
    0,
    fromY + 500,
  );
  // Pure-Three.js scenes (e.g. apartment loading structure.glb directly via
  // loadGLTF + scene.add) put their floor outside of assetsGroup/primitivesGroup.
  // Ray-cast the full scene as a third target — the worldN.y >= 0.5 filter
  // below still rules out walls, ceilings, and the skybox.
  const hits = [
    ...raycaster.intersectObject(assetsGroup, true),
    ...raycaster.intersectObject(primitivesGroup, true),
    ...raycaster.intersectObject(scene, true),
  ];
  let floorY = null;
  for (const h of hits) {
    const n = h.face?.normal;
    if (!n) continue;
    // Transform face normal to world space to test "up-facing"
    const worldN = n.clone().transformDirection(h.object.matrixWorld);
    if (worldN.y < 0.5) continue; // skip walls/ceilings
    if (floorY == null || h.point.y < floorY) floorY = h.point.y;
  }
  return floorY;
}

function spawnPlayerInsideScene() {
  const floorY = _findSceneFloorY(0, 0);
  if (floorY == null) return false;
  const y = floorY + PLAYER_EYE_HEIGHT;
  camera.position.set(0, y, 0);
  teleportPlayerTo(0, y, 0);
  return true;
}


function safeDisableGhost() {
  // If we're currently inside occupied geometry, turning collisions back on will
  // trap the character (penetration state). Use Rapier query pipeline to find a safe spot.
  if (!playerBody) return setGhostMode(false);
  const p = playerBody.translation();

  // Use Rapier query pipeline to find a non-penetrating spot.
  if (rapierWorld && playerCollider) {
    try {
      const shape = playerCollider.shape;
      const rot = playerCollider.rotation();
      const here = { x: p.x, y: p.y, z: p.z };

      const intersectsHere = rapierWorld.queryPipeline.intersectionWithShape(
        rapierWorld.bodies,
        rapierWorld.colliders,
        here,
        rot,
        shape,
        RAPIER.QueryFilterFlags.EXCLUDE_SENSORS,
        undefined,
        playerCollider.handle
      );

      // If we're not intersecting anything solid, we can safely disable ghost immediately.
      if (intersectsHere == null) {
        setGhostMode(false);
        setStatus("Ghost disabled.");
        return;
      }

      const tryOffsets = (maxR, step) => {
        for (let r = step; r <= maxR; r += step) {
          // sample a handful of directions per radius
          const dirs = [
            [1, 0, 0],
            [-1, 0, 0],
            [0, 0, 1],
            [0, 0, -1],
            [1, 0, 1],
            [1, 0, -1],
            [-1, 0, 1],
            [-1, 0, -1],
            [0, 1, 0],
            [0, -1, 0],
          ];
          for (const [dx, dy, dz] of dirs) {
            const len = Math.hypot(dx, dy, dz) || 1;
            const pos = { x: p.x + (dx / len) * r, y: p.y + (dy / len) * r, z: p.z + (dz / len) * r };
            const hit = rapierWorld.queryPipeline.intersectionWithShape(
              rapierWorld.bodies,
              rapierWorld.colliders,
              pos,
              rot,
              shape,
              RAPIER.QueryFilterFlags.EXCLUDE_SENSORS,
              undefined,
              playerCollider.handle
            );
            if (hit == null) return pos;
          }
        }
        return null;
      };

      const pos = tryOffsets(2.5, 0.15);
      if (pos) {
        teleportPlayerTo(pos.x, pos.y, pos.z);
        setGhostMode(false);
        setStatus("Ghost disabled (moved to nearest free space).");
        return;
      }
    } catch {
      // ignore
    }
  }

  setStatus("Couldn't find free space to disable Ghost. Staying in Ghost mode.");
  setGhostMode(true);
}

window.addEventListener("keydown", (e) => {
  const tagName = e.target?.tagName?.toLowerCase?.();
  const isTyping =
    tagName === "input" || tagName === "textarea" || tagName === "select" || e.target?.isContentEditable;
  if (!isTyping) {
    if (e.code === "KeyB") {
      void spawnOrMoveAiAtAim({ createNew: false, ephemeral: false });
      e.preventDefault();
    }
  }
  if (e.code === "KeyW") keys.forward = true;
  if (e.code === "KeyS") keys.backward = true;
  if (e.code === "KeyA") keys.left = true;
  if (e.code === "KeyD") keys.right = true;
  if (e.code === "Space") keys.up = true;
  if (e.code === "ShiftLeft" || e.code === "ShiftRight") keys.down = true;
  if (e.code === "KeyF") flyMode = !flyMode;
  if (e.code === "KeyG") {
    if (ghostMode) safeDisableGhost();
    else setGhostMode(true);
  }

  // === PLAYER INTERACTION KEYS ===
  // E key to interact with asset at crosshair
  if (e.code === "KeyE" && controls?.isLocked && !isTyping) {
    handlePlayerInteraction();
    e.preventDefault();
  }
  if (e.code === "KeyR" && controls?.isLocked && !isTyping && !isInteractionPopupVisible()) {
    if (cycleInteractableTarget(1)) {
      updateInteractionHint();
      e.preventDefault();
    }
  }

  // Escape to close interaction popup
  if (e.code === "Escape" && isInteractionPopupVisible()) {
    hideInteractionPopup();
    // Re-lock pointer after closing popup
    controls?.lock?.();
    e.preventDefault();
  }

  // Number keys 1-9 to select action when popup is visible
  if (isInteractionPopupVisible() && _currentInteractableAsset) {
    const numMatch = e.code.match(/^(?:Digit|Numpad)([1-9])$/);
    if (numMatch) {
      const idx = parseInt(numMatch[1], 10) - 1;
      const { asset, actions } = _currentInteractableAsset;
      if (idx >= 0 && idx < actions.length) {
        const actionId = actions[idx].id;

        // Hide popup and re-lock pointer FIRST (before async operations)
        hideInteractionPopup();

        // Execute the action
        if (actionId === "__PICK_UP__") {
          playerPickUpAsset(asset.id);
        } else {
          executePlayerInteraction(asset.id, actionId);
        }

        // Re-lock pointer (use setTimeout since pointer lock may need a moment)
        setTimeout(() => {
          try {
            controls?.lock?.();
          } catch (err) {
            // Pointer lock requires user gesture, may fail silently
          }
        }, 10);

        e.preventDefault();
        e.stopPropagation();
        return;
      }
    }
  }
});

window.addEventListener("keyup", (e) => {
  if (e.code === "KeyW") keys.forward = false;
  if (e.code === "KeyS") keys.backward = false;
  if (e.code === "KeyA") keys.left = false;
  if (e.code === "KeyD") keys.right = false;
  if (e.code === "Space") keys.up = false;
  if (e.code === "ShiftLeft" || e.code === "ShiftRight") keys.down = false;
});


// Shadow catcher: a large transparent ground plane that only shows shadows.
// ShadowMaterial is fully transparent where there's no shadow, so the splat
// floor shows through, but shadows appear as dark patches on top.
const shadowCatcherMat = new THREE.ShadowMaterial({ opacity: 0.35 });
const shadowCatcher = new THREE.Mesh(
  new THREE.PlaneGeometry(200, 200),
  shadowCatcherMat
);
shadowCatcher.rotation.x = -Math.PI / 2; // lie flat
shadowCatcher.position.y = 0.001;         // just above grid to avoid z-fighting
shadowCatcher.receiveShadow = true;
shadowCatcher.name = "__shadowCatcher";
scene.add(shadowCatcher);
// Add to scene lights registry so it's controllable from the editor
sceneLights.push({ id: "_shadow_ground", label: "Shadow Ground", obj: shadowCatcher, type: "shadow_ground" });

function _hasBumpableAssets() {
  for (const a of assets) { if (a?.bumpable) return true; }
  return false;
}

function updateBumpableAssets(dt, playerPos, agentPushers = []) {
  if (!playerPos || !_hasBumpableAssets()) {
    _playerPosPrevForBumpValid = false;
    return;
  }
  if (!_playerPosPrevForBumpValid) {
    _playerPosPrevForBump.copy(playerPos);
    _playerPosPrevForBumpValid = true;
    return;
  }
  const playerVel = new THREE.Vector3().subVectors(playerPos, _playerPosPrevForBump).divideScalar(Math.max(dt, 1e-3));
  _playerPosPrevForBump.copy(playerPos);
  const speedXZ = Math.hypot(playerVel.x, playerVel.z);
  const playerCanPush = !ghostMode;
  const intent = new THREE.Vector3();
  const camForward = new THREE.Vector3();
  camera.getWorldDirection(camForward);
  camForward.y = 0;
  if (camForward.lengthSq() > 1e-6) camForward.normalize();
  const camRight = new THREE.Vector3().crossVectors(camForward, camera.up).normalize();
  if (keys.forward) intent.add(camForward);
  if (keys.backward) intent.sub(camForward);
  if (keys.right) intent.add(camRight);
  if (keys.left) intent.sub(camRight);
  if (intent.lengthSq() > 1e-6) intent.normalize();
  const intentPush = playerCanPush && intent.lengthSq() > 0;
  const pushDir = intentPush ? intent.clone() : new THREE.Vector3(playerVel.x, 0, playerVel.z);
  if (pushDir.lengthSq() > 1e-6) pushDir.normalize();
  const playerRadius = 0.35;
  const pushThreshold = 0.05;
  let anyMoved = false;
  let anyColliderNeedsSync = false;
  for (const a of assets) {
    if (!a?.bumpable) continue;
    const obj = assetsGroup.getObjectByName(`asset:${a.id}`);
    if (!obj) continue;
    const vel = _assetBumpVelocities.get(a.id) || new THREE.Vector3();
    const localCenter = obj.userData?._localSphereCenter || new THREE.Vector3();
    const worldCenter = localCenter.clone();
    obj.localToWorld(worldCenter);
    const worldRadius = (obj.userData?._localSphereRadius || 0.6) * Math.max(obj.scale.x, obj.scale.y, obj.scale.z);
    const dx = worldCenter.x - playerPos.x;
    const dz = worldCenter.z - playerPos.z;
    const dist = Math.hypot(dx, dz);
    const minDist = worldRadius + playerRadius;
    const ahead = pushDir.lengthSq() > 0 ? (dx * pushDir.x + dz * pushDir.z) : 0;
    const lateral = pushDir.lengthSq() > 0 ? Math.abs(dx * -pushDir.z + dz * pushDir.x) : dist;
    const inPushCone = intentPush && ahead > -0.05 && ahead < (minDist + 0.9) && lateral < (worldRadius + 0.55);
    if (playerCanPush && (dist < (minDist + 0.35) || inPushCone) && (speedXZ > pushThreshold || intentPush)) {
      const dirX = dist > 1e-3 ? dx / dist : (intentPush ? pushDir.x : (Math.sign(playerVel.x) || 1));
      const dirZ = dist > 1e-3 ? dz / dist : (intentPush ? pushDir.z : (Math.sign(playerVel.z) || 0));
      const penetration = minDist - dist;
      const response = Number(a.bumpResponse) || 0.9;
      const driveSpeed = Math.max(speedXZ, intentPush ? 1.4 : 0);
      const intentBonus = inPushCone ? 0.35 : 0;
      const impulse = Math.min(2.4, (Math.max(0, penetration) * 3 + driveSpeed * 0.35 + intentBonus) * response);
      vel.x += dirX * impulse;
      vel.z += dirZ * impulse;
    }
    // AI agents can push bumpable assets as well.
    for (const ap of agentPushers) {
      const apPos = ap?.pos;
      const apVel = ap?.vel;
      if (!apPos || !apVel) continue;
      const av = Math.hypot(apVel.x || 0, apVel.z || 0);
      if (av <= 0.04) continue;
      const adx = worldCenter.x - apPos.x;
      const adz = worldCenter.z - apPos.z;
      const adist = Math.hypot(adx, adz);
      const aminDist = worldRadius + Math.max(0.22, Number(ap.radius) || 0.22);
      if (adist > aminDist + 0.3) continue;
      const dirX = adist > 1e-3 ? adx / adist : (Math.sign(apVel.x) || 1);
      const dirZ = adist > 1e-3 ? adz / adist : (Math.sign(apVel.z) || 0);
      const penetration = aminDist - adist;
      const response = Number(a.bumpResponse) || 0.9;
      const impulse = Math.min(2.2, (Math.max(0, penetration) * 2.4 + av * 0.28) * response);
      vel.x += dirX * impulse;
      vel.z += dirZ * impulse;
    }
    const damping = Math.min(0.995, Math.max(0.65, Number(a.bumpDamping) || 0.9));
    const dampPow = Math.pow(damping, dt * 60);
    vel.multiplyScalar(dampPow);
    const maxSpeed = 2.5;
    const speed = Math.hypot(vel.x, vel.z);
    if (speed > maxSpeed) {
      const s = maxSpeed / speed;
      vel.x *= s;
      vel.z *= s;
    }
    if (vel.lengthSq() < 1e-4) {
      vel.set(0, 0, 0);
      _assetBumpVelocities.set(a.id, vel);
      continue;
    }
    let moveX = THREE.MathUtils.clamp(vel.x * dt, -0.2, 0.2);
    let moveZ = THREE.MathUtils.clamp(vel.z * dt, -0.2, 0.2);
    const myBox = new THREE.Box3().setFromObject(obj);
    const testBoxX = myBox.clone().translate(new THREE.Vector3(moveX, 0, 0));
    const testBoxZ = myBox.clone().translate(new THREE.Vector3(0, 0, moveZ));
    let blockedX = false, blockedZ = false;
    const checkCollision = (testBox, excludeObj) => {
      for (const child of primitivesGroup.children) {
        if (child === excludeObj) continue;
        const cb = new THREE.Box3().setFromObject(child);
        if (!cb.isEmpty() && testBox.intersectsBox(cb)) return true;
      }
      for (const child of assetsGroup.children) {
        if (child === excludeObj) continue;
        if (child.userData?.isBlobShadow) continue;
        const cb = new THREE.Box3().setFromObject(child);
        if (!cb.isEmpty() && testBox.intersectsBox(cb)) return true;
      }
      return false;
    };
    if (Math.abs(moveX) > 1e-5 && checkCollision(testBoxX, obj)) {
      blockedX = true;
      vel.x *= -0.15;
    }
    if (Math.abs(moveZ) > 1e-5 && checkCollision(testBoxZ, obj)) {
      blockedZ = true;
      vel.z *= -0.15;
    }
    if (!blockedX) obj.position.x += moveX;
    if (!blockedZ) obj.position.z += moveZ;
    if (blockedX && blockedZ) {
      _assetBumpVelocities.set(a.id, vel);
      continue;
    }
    anyMoved = true;
    anyColliderNeedsSync = true;
    if (!a.transform) a.transform = {};
    if (!a.transform.position) a.transform.position = { x: 0, y: 0, z: 0 };
    a.transform.position.x = obj.position.x;
    a.transform.position.z = obj.position.z;
    _assetBumpVelocities.set(a.id, vel);
  }
  if (anyMoved) {
    const now = performance.now();
    if (now - _lastBumpSaveAt > 500) {
      _lastBumpSaveAt = now;
      saveTagsForWorld();
    }
    if (anyColliderNeedsSync && now - _lastBumpColliderSyncAt > 50) {
      _lastBumpColliderSyncAt = now;
      for (const a of assets) {
        if (!a?.bumpable) continue;
        if (!_assetBumpVelocities.has(a.id)) continue;
        const v = _assetBumpVelocities.get(a.id);
        if (!v || v.lengthSq() < 1e-4) continue;
        rebuildAssetCollider(a.id);
      }
    }
  }
}

function collectAgentBumpPushers(dt) {
  const pushers = [];
  const alive = new Set();
  const invDt = 1 / Math.max(dt, 1e-3);
  for (const agent of aiAgents) {
    const id = String(agent?.id || "");
    const posRaw = agent?.body?.translation?.();
    if (!id || !posRaw) continue;
    alive.add(id);
    const pos = new THREE.Vector3(posRaw.x, posRaw.y, posRaw.z);
    const prev = _agentPosPrevForBump.get(id);
    const vel = prev ? pos.clone().sub(prev).multiplyScalar(invDt) : new THREE.Vector3();
    _agentPosPrevForBump.set(id, pos.clone());
    pushers.push({
      id,
      pos,
      vel,
      radius: Math.max(0.2, Number(agent?.radius) || 0.2),
    });
  }
  for (const id of _agentPosPrevForBump.keys()) {
    if (!alive.has(id)) _agentPosPrevForBump.delete(id);
  }
  return pushers;
}

function updateRapier(dt) {
  // No physics world loaded → free-fly camera movement so user can still navigate
  if (!rapierWorld || !playerBody) {
    const flySpeed = 8.0;
    const fwd = new THREE.Vector3();
    camera.getWorldDirection(fwd);
    const right = new THREE.Vector3().crossVectors(fwd, camera.up).normalize();
    const move = new THREE.Vector3();
    if (keys.forward) move.add(fwd);
    if (keys.backward) move.sub(fwd);
    if (keys.right) move.add(right);
    if (keys.left) move.sub(right);
    if (keys.up) move.y += 1;
    if (keys.down) move.y -= 1;
    if (move.lengthSq() > 0) {
      move.normalize().multiplyScalar(flySpeed * dt);
      controls.object.position.add(move);
      avatar.position.copy(controls.object.position).y -= PLAYER_EYE_HEIGHT;
    }
    return;
  }

  // Flush any deferred collider builds BEFORE stepping
  flushPendingColliderBuilds();

  // Step physics FIRST — this integrates last frame's kinematic moves and
  // updates the query pipeline internally, avoiding the RefCell double-borrow
  // that happens with manual `queryPipeline.update(colliders)`.
  rapierWorld.timestep = dt;
  try {
    rapierWorld.step();
    _rapierStepFaultCount = 0;
  } catch (e) {
    _rapierStepFaultCount += 1;
    console.warn(`[RAPIER] step() failed (${_rapierStepFaultCount})`, e);
    // Prevent hard crash loop; skip this frame and try again next tick.
    return;
  }

  // Sync camera and avatar to the body position that step() just resolved
  const p = playerBody.translation();

  // Skip player movement when camera is following agent
  if (agentCameraFollow) {
    avatar.position.set(p.x, p.y, p.z);
    return;
  }

  const baseSpeed = 6.0;
  const runSpeed = 10.0;
  const flySpeed = 8.0;
  const speed = flyMode ? flySpeed : keys.down ? runSpeed : baseSpeed;
  const gravity = 20.0;
  const jumpVel = 8.0;

  const forward = new THREE.Vector3();
  camera.getWorldDirection(forward);
  forward.y = 0;
  forward.normalize();
  const right = new THREE.Vector3().crossVectors(forward, camera.up).normalize();

  const wish = new THREE.Vector3();
  if (keys.forward) wish.add(forward);
  if (keys.backward) wish.sub(forward);
  if (keys.right) wish.add(right);
  if (keys.left) wish.sub(right);
  if (wish.lengthSq() > 0) wish.normalize();

  const upDown = flyMode ? (keys.up ? 1 : 0) + (keys.down ? -1 : 0) : 0;

  const t = p; // body position after step
  let desired = { x: 0, y: 0, z: 0 };

  if (ghostMode) {
    desired = {
      x: wish.x * flySpeed * dt,
      y: ((keys.up ? 1 : 0) + (keys.down ? -1 : 0)) * flySpeed * dt,
      z: wish.z * flySpeed * dt,
    };
    playerBody.setNextKinematicTranslation({
      x: t.x + desired.x,
      y: t.y + desired.y,
      z: t.z + desired.z,
    });
  } else if (flyMode) {
    desired = {
      x: wish.x * flySpeed * dt,
      y: upDown * flySpeed * dt,
      z: wish.z * flySpeed * dt,
    };
    if (characterController && playerCollider) {
      characterController.computeColliderMovement(
        playerCollider,
        desired,
        RAPIER.QueryFilterFlags.EXCLUDE_SENSORS
      );
      const m = characterController.computedMovement();
      const mx = m.x, my = m.y, mz = m.z;
      playerBody.setNextKinematicTranslation({ x: t.x + mx, y: t.y + my, z: t.z + mz });
    } else {
      playerBody.setNextKinematicTranslation({ x: t.x + desired.x, y: t.y + desired.y, z: t.z + desired.z });
    }
  } else {
    walkVerticalVel -= gravity * dt;

    if (keys.up && characterController?.computedGrounded?.()) {
      walkVerticalVel = jumpVel;
    }

    desired = { x: wish.x * speed * dt, y: walkVerticalVel * dt, z: wish.z * speed * dt };

    if (characterController && playerCollider) {
      characterController.computeColliderMovement(
        playerCollider,
        desired,
        RAPIER.QueryFilterFlags.EXCLUDE_SENSORS
      );
      const m = characterController.computedMovement();
      const mx = m.x, my = m.y, mz = m.z;
      const grounded = characterController.computedGrounded();
      if (grounded && walkVerticalVel < 0) walkVerticalVel = 0;
      playerBody.setNextKinematicTranslation({ x: t.x + mx, y: t.y + my, z: t.z + mz });
    } else {
      playerBody.setNextKinematicTranslation({ x: t.x + desired.x, y: t.y + desired.y, z: t.z + desired.z });
    }
  }

  // Safety: if Ghost is OFF, ensure the collider is not a sensor
  try {
    if (!ghostMode && playerCollider && typeof playerCollider.isSensor === "function" && playerCollider.isSensor()) {
      playerCollider.setSensor(false);
    }
  } catch {}
  avatar.position.set(p.x, p.y, p.z);

  // If agent camera follow is active, DON'T sync player camera to player body
  // The tick() function will handle camera positioning via updateAgentCameraFollow
  if (!agentCameraFollow) {
    controls.object.position.set(p.x, p.y + PLAYER_EYE_HEIGHT, p.z);
  }

}

function tick() {
  const rawDt = clock.getDelta();
  const physicsDt = Math.min(rawDt, 0.05);
  const motionDt = Math.min(rawDt, 0.02);

  updateRapier(physicsDt);

  // Bumpable assets: only compute if any exist
  if (_hasBumpableAssets()) {
    const agentPushers = aiAgents.length ? collectAgentBumpPushers(physicsDt) : [];
    let bumpPlayerPos = null;
    if (playerBody) {
      const p = playerBody.translation();
      bumpPlayerPos = new THREE.Vector3(p.x, p.y, p.z);
    } else {
      bumpPlayerPos = controls.object.position.clone();
      bumpPlayerPos.y -= PLAYER_EYE_HEIGHT;
    }
    updateBumpableAssets(physicsDt, bumpPlayerPos, agentPushers);
  }

  // Update AI agents (if Rapier is initialized).
  if (aiAgents.length && rapierWorld) {
    const now = Date.now();
    for (const a of aiAgents) {
      try {
        // Keep cmd_vel integration tied to wall-clock delta even when physics dt is clamped.
        a.update(motionDt, now);
      } catch (e) {
        console.warn("AI update failed:", e);
      }
    }
  }

  // Update agent camera follow (after agent update, before render)
  if (agentCameraFollow) {
    updateAgentCameraFollow(physicsDt);
    avatar.visible = false;
  }

  // Update interaction hint at reduced rate
  const now = performance.now();
  if (now - _lastHintUpdate > 300) {
    _lastHintUpdate = now;
    updateInteractionHint();
  }

  // LiDAR viz — browser-side raycasting only outside dimos mode (in dimos
  // mode the server handles lidar via Rapier snapshots).
  if (!dimosMode && (simSensorViewMode === "lidar" || simCompareView)) {
    lidarVizGroup.visible = true;
    updateLidarPointCloud();
    if (_lidarGeom.drawRange.count <= 0 && _lidarLastNonZeroDrawCount > 0) {
      _lidarGeom.setDrawRange(0, _lidarLastNonZeroDrawCount);
    }
  }

  if (!dimosMode) pushLidarPoseSample();

  renderActiveView();
  requestAnimationFrame(tick);
}

// Interaction hint elements (cached)
let _lastHintUpdate = 0;
let _crosshairEl = null;
let _interactionHintEl = null;

function updateInteractionHint() {
  // Cache DOM elements
  if (!_crosshairEl) _crosshairEl = document.getElementById("crosshair");
  if (!_interactionHintEl) _interactionHintEl = document.getElementById("interaction-hint");

  // Only show when pointer is locked and no popup is visible
  if (!controls?.isLocked || isInteractionPopupVisible()) {
    _crosshairEl?.classList.remove("interactable");
    if (_interactionHintEl) {
      _interactionHintEl.classList.remove("visible");
    }
    return;
  }

  // If holding something, show drop hint
  if (playerHeldAsset) {
    const heldAsset = getPlayerHeldAsset();
    const heldName = heldAsset?.title || "item";
    _crosshairEl?.classList.remove("interactable");
    _crosshairEl?.classList.add("holding");
    if (_interactionHintEl) {
      _interactionHintEl.innerHTML = `Holding: ${escapeHtml(heldName)} · Drop<span class="hint-key">E</span>`;
      _interactionHintEl.classList.add("visible");
    }
    return;
  }
  if (playerHeldGroupId) {
    const heldGroup = groups.find((g) => g.id === playerHeldGroupId);
    const heldName = heldGroup?.name || "group";
    _crosshairEl?.classList.remove("interactable");
    _crosshairEl?.classList.add("holding");
    if (_interactionHintEl) {
      _interactionHintEl.innerHTML = `Holding: ${escapeHtml(heldName)} · Drop<span class="hint-key">E</span>`;
      _interactionHintEl.classList.add("visible");
    }
    return;
  }

  // Not holding anything - remove holding class
  _crosshairEl?.classList.remove("holding");

  const target = getInteractableAssetAtCrosshair();

  if (target) {
    const { kind, asset, group, actions, dist, canPickUp } = target;
    const title = kind === "group" ? (group?.name || "(group)") : (asset.title || "(asset)");

    // Build action description
    let actionText;
    if (kind === "group") {
      actionText = "Pick up";
    } else if (actions.length === 0 && canPickUp) {
      actionText = "Pick up";
    } else if (actions.length === 1 && !canPickUp) {
      actionText = actions[0].label || "interact";
    } else {
      const count = actions.length + (canPickUp ? 1 : 0);
      actionText = `${count} actions`;
    }

    _crosshairEl?.classList.add("interactable");
    if (_interactionHintEl) {
      const cycleHint = kind === "asset" && target.candidateCount > 1
        ? ` · Cycle ${target.candidateIndex + 1}/${target.candidateCount}<span class="hint-key">R</span>`
        : "";
      _interactionHintEl.innerHTML = `${escapeHtml(title)} · ${escapeHtml(actionText)}<span class="hint-key">E</span>${cycleHint}`;
      _interactionHintEl.classList.add("visible");
    }
  } else {
    _crosshairEl?.classList.remove("interactable");
    if (_interactionHintEl) {
      _interactionHintEl.classList.remove("visible");
    }
  }
}

setStatus("Select a .ply/.spz to start.");
tick();

// Expose debug utilities
window.clearWorldStorage = clearWorldStorage;
window.__robovalLidar = {
  // Returns the latest standardized frames (raw + deskewed + optional range image)
  getLatestFrames() {
    return {
      raw: _lidarLatestRawFrame,
      deskewed: _lidarLatestDeskewedFrame,
      rangeImage: _lidarLatestRangeImage,
    };
  },
  // ROS2 PointCloud2-compatible dict converter
  toPointCloud2(frame) {
    return to_pointcloud2(frame);
  },
  // Manual export of the latest frame set to NPZ files.
  async exportLatest() {
    if (!_lidarLatestRawFrame || !_lidarLatestDeskewedFrame) return false;
    await writeLidarFrameFiles(_lidarLatestRawFrame, _lidarLatestDeskewedFrame, _lidarLatestRangeImage);
    return true;
  },
  // Auto-export each LiDAR frame (warning: downloads many files in browser).
  setAutoExport(enabled) {
    _lidarAutoExport = !!enabled;
    return _lidarAutoExport;
  },
  getAutoExport() {
    return _lidarAutoExport;
  },
  // Force a known-good synthetic cloud to isolate renderer issues from sensor math.
  setKnownGoodDebugCloud(enabled) {
    _lidarUseKnownGoodDebugCloud = !!enabled;
    _lidarAccumFrames.length = 0;
    _lidarLastAccumPose = null;
    resetLidarScanState();
    if (simSensorViewMode === "lidar") updateLidarPointCloud();
    return _lidarUseKnownGoodDebugCloud;
  },
  getKnownGoodDebugCloud() {
    return _lidarUseKnownGoodDebugCloud;
  },
  // Toggle ordered scan debug render (single-frame, lidar-frame) vs accumulated world cloud.
  setOrderedDebugView(enabled) {
    lidarOrderedDebugView = !!enabled;
    if (!lidarOrderedDebugView) {
      _lidarAccumFrames.length = 0;
      _lidarLastAccumPose = null;
      resetLidarScanState();
    }
    updateSimSensorButtons();
    if (simSensorViewMode === "lidar") updateLidarPointCloud();
    return lidarOrderedDebugView;
  },
  getOrderedDebugView() {
    return lidarOrderedDebugView;
  },
  setNoiseModel(enabled) {
    lidarNoiseEnabled = !!enabled;
    _lidarAccumFrames.length = 0;
    _lidarLastAccumPose = null;
    resetLidarScanState();
    updateSimSensorButtons();
    if (simSensorViewMode === "lidar") updateLidarPointCloud();
    return lidarNoiseEnabled;
  },
  getNoiseModel() {
    return lidarNoiseEnabled;
  },
  setMultiReturnMode(mode) {
    lidarMultiReturnMode = mode === "last" ? "last" : "strongest";
    _lidarAccumFrames.length = 0;
    _lidarLastAccumPose = null;
    resetLidarScanState();
    updateSimSensorButtons();
    if (simSensorViewMode === "lidar") updateLidarPointCloud();
    return lidarMultiReturnMode;
  },
  getMultiReturnMode() {
    return lidarMultiReturnMode;
  },
};

window.__robovalRgbd = {
  // Returns metric camera-space Z depth map in meters (Float32Array length W*H).
  // Uses the same render path as on-screen RGB-D mode.
  getMetricDepthFrame() {
    renderRgbdView();
    const depth = readRgbdMetricDepthFrameMeters();
    if (!depth) return null;
    return {
      width: rgbdMetricTarget.width,
      height: rgbdMetricTarget.height,
      depth_m: depth,
      semantics: "camera_space_z",
      units: "meters",
      min_depth_m: RGBD_MIN_DEPTH_M,
      max_depth_m: RGBD_MAX_DEPTH_M,
    };
  },
};

// Debug: List all colliders in the physics world
window.debugColliders = function() {
  if (!rapierWorld) {
    console.log("[DEBUG] No physics world loaded");
    return;
  }

  console.log("[DEBUG] === ALL COLLIDERS IN PHYSICS WORLD ===");
  let count = 0;
  rapierWorld.colliders.forEach((collider) => {
    const pos = collider.translation();
    const shape = collider.shape;
    const isSensor = collider.isSensor();
    const handle = collider.handle;
    console.log(`Collider #${count} (handle=${handle}): pos=(${pos.x.toFixed(2)}, ${pos.y.toFixed(2)}, ${pos.z.toFixed(2)}), sensor=${isSensor}, shapeType=${shape.type}`);
    count++;
  });
  console.log(`[DEBUG] Total colliders: ${count}`);

  // Also show asset collider handles
  console.log("[DEBUG] === ASSET COLLIDERS (on asset objects) ===");
  let assetColCount = 0;
  for (const a of assets) {
    if (a._colliderHandle) {
      const handleInfo = typeof a._colliderHandle === 'object' ? `obj.handle=${a._colliderHandle.handle}` : `num=${a._colliderHandle}`;
      console.log(`  ${a.id}: "${a.title}", _colliderHandle=${handleInfo}`);
      assetColCount++;
    }
  }
  console.log(`[DEBUG] Assets with colliders: ${assetColCount}`);

  // Show tracked map
  console.log("[DEBUG] === _assetColliderHandles Map ===");
  console.log(`Map size: ${_assetColliderHandles.size}`);
};

// Debug: Remove all colliders except world/player
window.debugClearAssetColliders = function() {
  if (!rapierWorld) return;

  // Helper to remove a collider (handles both object and number)
  const removeCol = (handle) => {
    try {
      if (typeof handle === 'object' && handle.handle !== undefined) {
        rapierWorld.removeCollider(handle, true);
        return true;
      } else if (typeof handle === 'number') {
        const collider = rapierWorld.getCollider(handle);
        if (collider) {
          rapierWorld.removeCollider(collider, true);
          return true;
        }
      }
    } catch (e) {}
    return false;
  };

  let removed = 0;

  // Remove all tracked asset colliders
  _assetColliderHandles.forEach((handle, assetId) => {
    if (removeCol(handle)) removed++;
  });
  _assetColliderHandles.clear();

  // Also clear colliders stored on asset objects
  for (const asset of assets) {
    if (asset._colliderHandle != null) {
      if (removeCol(asset._colliderHandle)) removed++;
      asset._colliderHandle = null;
    }
  }

  console.log(`[DEBUG] Cleared ${removed} asset colliders`);
};

// ── dimos integration mode boot ──────────────────────────────────────────────
// When dimosMode is active, auto-load a scene and spawn an agent, then connect
// the LCM bridge so sensor data flows and external /odom drives the agent.
if (dimosMode) {
  (async () => {
    try {
      // 1. Load Rapier first — scene module's build() may create colliders
      const sceneName = dimosScene || "empty";
      console.log(`[dimos] Loading scene: ${sceneName}`);
      await ensureRapierLoaded();

      // 2. Initialize the scene-api module shared with the runtime exec sandbox.
      //    Scenes can either receive the api as a build() arg or — for runtime
      const sceneApi = await import("./sceneApi.ts");
      sceneApi._init({
        scene, THREE, RAPIER, rapierWorld,
        renderer, camera, agent: null,
        gltfLoader,
        // Bridge isn't connected yet; pre-bridge collider sends are dropped
        // and the initial-state Rapier snapshot (shipped later) covers them.
        sendPhysics: (msg) => window.__dimosBridge?.sendCommand?.(msg),
        sceneBaseUrl: new URL(`/scenes/${sceneName}/`, window.location.href).toString(),
        importLevelFromJSON,
        setSky: (opts) => {
          sceneSettings.sky = { ...sceneSettings.sky, enabled: opts.enabled !== false, ...opts };
          applySceneSkySettings();
          applySceneRgbBackground();
        },
      });

      // 3. Dynamic-import the scene module + run its build()
      const sceneMod = await import(/* @vite-ignore */ `/scenes/${sceneName}/index.js`);
      if (typeof sceneMod.default !== "function") {
        throw new Error(`Scene "${sceneName}" must export a default build function`);
      }
      const sceneCfg = (await sceneMod.default(sceneApi)) || {};
      if (typeof sceneMod.afterBuild === "function") {
        await sceneMod.afterBuild(sceneApi);
      }
      console.log(`[dimos] Scene loaded: ${sceneName}`);

      // 4. Spawn player + agent based on scene config
      spawnPlayerInsideScene();
      // In dimos mode the agent is always driven by an external Python process,
      // so an embodiment visual is always wanted.  Scenes return `embodiment:
      // null` because they don't want to dictate the model — let createAiAgent's
      // default avatarUrl (the dimsim_unitree_stub.glb) take over.  The legacy
      // "hide the group when no embodiment" path predates dimos integration and
      // is no longer reached here.
      const pendingEmb = sceneApi._getPendingEmbodiment?.();
      const agent = createAiAgent({
        ephemeral: false,
        avatarUrl: pendingEmb?.avatarUrl,
        radius: pendingEmb?.radius,
        halfHeight: pendingEmb?.halfHeight,
      });
      aiAgents.push(agent);
      sceneApi._setAgent(agent);
      const spawnPos = sceneCfg.spawnPoint || { x: 2, y: 0.5, z: 3 };
      agent.setPosition(spawnPos.x, spawnPos.y, spawnPos.z);
      renderAgentTaskUi(); // update UI: hide spawn button, enable task controls
      // Server-side physics: agent pose is driven by ServerPhysics (Deno).
      // Browser just receives position updates and moves the visual avatar.
      let _dimosYaw = 0;
      // Bridge updates _dimosYaw via this setter when server sends pose
      window.__dimosSetYaw = (yaw) => { _dimosYaw = yaw; };
      agent.update = function(_dt) {
        this._syncVisual();
      };
      console.log(`[dimos] Agent spawned: ${agent.id}`);

      // 3. Set up fixed-size offscreen capture for dimos.
      // Keep sensor cost independent of the headed browser window size.
      const _dimosCapW = 640, _dimosCapH = 288;
      const _dimosCapTarget = new THREE.WebGLRenderTarget(_dimosCapW, _dimosCapH, {
        minFilter: THREE.LinearFilter, magFilter: THREE.LinearFilter,
        format: THREE.RGBAFormat, depthBuffer: true, stencilBuffer: false,
      });
      // Go2 depth camera: 87° horizontal. At 640x288 (2.22:1 aspect), that's 46° vertical.
      const _dimosFov = window.__dimosCameraFov || 46;
      const _dimosCapCam = new THREE.PerspectiveCamera(_dimosFov, _dimosCapW / _dimosCapH, camera.near, camera.far);
      const _dimosCapBuf = new Uint8Array(_dimosCapW * _dimosCapH * 4);
      const _dimosCapCvs = document.createElement("canvas");
      _dimosCapCvs.width = _dimosCapW;
      _dimosCapCvs.height = _dimosCapH;
      const _dimosCapCtx = _dimosCapCvs.getContext("2d");
      const _dimosDepthTarget = new THREE.WebGLRenderTarget(_dimosCapW, _dimosCapH, {
        minFilter: THREE.NearestFilter,
        magFilter: THREE.NearestFilter,
        format: THREE.RGBAFormat,
        type: THREE.UnsignedByteType,
        depthBuffer: true,
        stencilBuffer: false,
      });
      _dimosDepthTarget.texture.generateMipmaps = false;
      _dimosDepthTarget.depthTexture = new THREE.DepthTexture(_dimosCapW, _dimosCapH, THREE.UnsignedIntType);
      _dimosDepthTarget.depthTexture.minFilter = THREE.NearestFilter;
      _dimosDepthTarget.depthTexture.magFilter = THREE.NearestFilter;
      _dimosDepthTarget.depthTexture.generateMipmaps = false;
      const _dimosMetricTarget = new THREE.WebGLRenderTarget(_dimosCapW, _dimosCapH, {
        minFilter: THREE.NearestFilter,
        magFilter: THREE.NearestFilter,
        format: rgbdMetricUsesR32F ? THREE.RedFormat : THREE.RGBAFormat,
        type: rgbdMetricTargetType,
        depthBuffer: false,
        stencilBuffer: false,
      });
      if (rgbdMetricUsesR32F) _dimosMetricTarget.texture.internalFormat = "R32F";
      _dimosMetricTarget.texture.generateMipmaps = false;

      function _dimosReadMetricDepthFrameMeters() {
        const w = _dimosMetricTarget.width;
        const h = _dimosMetricTarget.height;
        if (!w || !h) return null;

        if (rgbdMetricUsesR32F) {
          const depth = new Float32Array(w * h);
          renderer.readRenderTargetPixels(_dimosMetricTarget, 0, 0, w, h, depth);
          return depth;
        }

        if (_dimosMetricTarget.texture.type === THREE.FloatType) {
          const raw = new Float32Array(w * h * 4);
          renderer.readRenderTargetPixels(_dimosMetricTarget, 0, 0, w, h, raw);
          const depth = new Float32Array(w * h);
          for (let i = 0; i < w * h; i++) depth[i] = raw[i * 4 + 0];
          return depth;
        }

        const raw = new Uint16Array(w * h * 4);
        renderer.readRenderTargetPixels(_dimosMetricTarget, 0, 0, w, h, raw);
        const depth = new Float32Array(w * h);
        for (let i = 0; i < w * h; i++) depth[i] = halfToFloat(raw[i * 4 + 0]);
        return depth;
      }

      function _dimosCaptureRgb() {
        const [ax, ay, az] = agent.getPosition?.() || [0, 0, 0];
        const yaw = agent.group?.rotation?.y ?? 0;
        const pitch = typeof agent.pitch === "number" ? agent.pitch : 0;
        const cp = Math.cos(pitch), sp = Math.sin(pitch);
        const feetY = ay - ((agent.halfHeight || 0.25) + (agent.radius || 0.12));
        const eyeY = feetY + GO2_CAMERA_HEIGHT;
        const eyeX = ax + Math.sin(yaw) * GO2_CAMERA_FORWARD;
        const eyeZ = az + Math.cos(yaw) * GO2_CAMERA_FORWARD;
        _dimosCapCam.position.set(eyeX, eyeY, eyeZ);
        _dimosCapCam.lookAt(eyeX + Math.sin(yaw)*cp, eyeY + sp, eyeZ + Math.cos(yaw)*cp);
        _dimosCapCam.updateProjectionMatrix();
        _dimosCapCam.updateMatrixWorld(true);

        const prev = renderer.getRenderTarget();
        const prevAgentVisible = agent.group?.visible;
        if (agent.group) agent.group.visible = false;
        renderer.setRenderTarget(_dimosCapTarget);
        renderer.render(scene, _dimosCapCam);
        renderer.setRenderTarget(prev);
        if (agent.group) agent.group.visible = prevAgentVisible;

        renderer.readRenderTargetPixels(_dimosCapTarget, 0, 0, _dimosCapW, _dimosCapH, _dimosCapBuf);
        // Flip Y — return raw RGBA pixels (no JPEG encode)
        const flipped = new Uint8Array(_dimosCapW * _dimosCapH * 4);
        const rowB = _dimosCapW * 4;
        for (let y = 0; y < _dimosCapH; y++) {
          flipped.set(_dimosCapBuf.subarray((_dimosCapH-1-y)*rowB, (_dimosCapH-y)*rowB), y*rowB);
        }
        return { data: flipped, width: _dimosCapW, height: _dimosCapH };
      }

      // Offscreen depth capture from agent POV using a dedicated low-res target.
      function _dimosCaptureDepth() {
        const [ax, ay, az] = agent.getPosition?.() || [0, 0, 0];
        const yaw = agent.group?.rotation?.y ?? 0;
        const pitch = typeof agent.pitch === "number" ? agent.pitch : 0;
        const cp = Math.cos(pitch), sp = Math.sin(pitch);
        const feetY = ay - ((agent.halfHeight || 0.25) + (agent.radius || 0.12));
        const eyeY = feetY + GO2_CAMERA_HEIGHT;
        const eyeX = ax + Math.sin(yaw) * GO2_CAMERA_FORWARD;
        const eyeZ = az + Math.cos(yaw) * GO2_CAMERA_FORWARD;
        _dimosCapCam.position.set(eyeX, eyeY, eyeZ);
        _dimosCapCam.lookAt(eyeX + Math.sin(yaw)*cp, eyeY + sp, eyeZ + Math.cos(yaw)*cp);
        _dimosCapCam.updateProjectionMatrix();
        _dimosCapCam.updateMatrixWorld(true);

        const prevDepthTex = rgbdMetricMaterial.uniforms.uDepthTex.value;
        const prevNear = rgbdMetricMaterial.uniforms.uNear.value;
        const prevFar = rgbdMetricMaterial.uniforms.uFar.value;
        const prevNoise = rgbdMetricMaterial.uniforms.uNoiseEnabled.value;
        const prevSpeckle = rgbdMetricMaterial.uniforms.uSpeckleEnabled.value;
        rgbdMetricMaterial.uniforms.uDepthTex.value = _dimosDepthTarget.depthTexture;
        rgbdMetricMaterial.uniforms.uNear.value = _dimosCapCam.near;
        rgbdMetricMaterial.uniforms.uFar.value = _dimosCapCam.far;
        rgbdMetricMaterial.uniforms.uNoiseEnabled.value = rgbdNoiseEnabled ? 1.0 : 0.0;
        rgbdMetricMaterial.uniforms.uSpeckleEnabled.value = rgbdSpeckleEnabled ? 1.0 : 0.0;

        const savedOverride = scene.overrideMaterial;
        const savedAssets = assetsGroup.visible;
        const savedPrims = primitivesGroup.visible;
        const savedLights = lightsGroup.visible;
        const savedTags = tagsGroup.visible;
        const savedLidarViz = lidarVizGroup.visible;

        scene.overrideMaterial = null;
        assetsGroup.visible = true;
        primitivesGroup.visible = true;
        lightsGroup.visible = true;
        tagsGroup.visible = false;
        lidarVizGroup.visible = false;
        const savedAgentVisible = agent.group?.visible;
        if (agent.group) agent.group.visible = false;

        renderer.setRenderTarget(_dimosDepthTarget);
        renderer.setClearColor(0x000000, RGBD_CLEAR_ALPHA);
        renderer.clear(true, true, true);
        renderer.render(scene, _dimosCapCam);

        renderer.setRenderTarget(_dimosMetricTarget);
        renderer.setClearColor(0x000000, RGBD_CLEAR_ALPHA);
        renderer.clear(true, true, true);
        renderer.render(rgbdMetricScene, rgbdPostCamera);

        scene.overrideMaterial = savedOverride;
        assetsGroup.visible = savedAssets;
        primitivesGroup.visible = savedPrims;
        lightsGroup.visible = savedLights;
        tagsGroup.visible = savedTags;
        lidarVizGroup.visible = savedLidarViz;
        if (agent.group) agent.group.visible = savedAgentVisible;
        rgbdMetricMaterial.uniforms.uDepthTex.value = prevDepthTex;
        rgbdMetricMaterial.uniforms.uNear.value = prevNear;
        rgbdMetricMaterial.uniforms.uFar.value = prevFar;
        rgbdMetricMaterial.uniforms.uNoiseEnabled.value = prevNoise;
        rgbdMetricMaterial.uniforms.uSpeckleEnabled.value = prevSpeckle;
        renderer.setRenderTarget(null);

        const depthData = _dimosReadMetricDepthFrameMeters();
        if (!depthData) return null;

        const dw = _dimosMetricTarget.width, dh = _dimosMetricTarget.height;

        // Flip rows: WebGL reads bottom-to-top, image convention is top-to-bottom
        const flipped = new Float32Array(dw * dh);
        for (let y = 0; y < dh; y++) {
          flipped.set(depthData.subarray((dh - 1 - y) * dw, (dh - y) * dw), y * dw);
        }
        return { data: flipped, width: dw, height: dh };
      }


      // 5. Connect dimos bridge
      let _lastRgbBase64 = null;
      const { DimosBridge } = await import("./bridge.ts");
      const bridge = new DimosBridge({
        agent,
        rates: window.__dimosSensorRates || undefined,
        sensorEnable: window.__dimosSensorEnable || undefined,
        sensorSources: {
          captureRgb: () => {
            const frame = _dimosCaptureRgb();
            if (!frame) return null;
            // Render to canvas → JPEG (used for both LCM publish and eval/sidebar)
            _dimosCapCtx.putImageData(new ImageData(new Uint8ClampedArray(frame.data.buffer, frame.data.byteOffset, frame.data.byteLength), frame.width, frame.height), 0, 0);
            const dataUrl = _dimosCapCvs.toDataURL("image/jpeg", 0.75);
            _lastRgbBase64 = dataUrl.split("base64,")[1] || null;
            if (!_lastRgbBase64) return null;
            // Decode base64 → Uint8Array for JPEG LCM transport
            const bin = atob(_lastRgbBase64);
            const jpegBytes = new Uint8Array(bin.length);
            for (let i = 0; i < bin.length; i++) jpegBytes[i] = bin.charCodeAt(i);
            return { data: jpegBytes, width: frame.width, height: frame.height };
          },
          captureDepth: () => _dimosCaptureDepth(),
        },
      });


      bridge.connect();
      window.__dimosBridge = bridge;
      sceneApi._flushPendingEmbodiment?.();
      window.__dimosAgent = agent;

      // Send Rapier world snapshot to bridge server for server-side physics + lidar.
      // Flush any deferred collider builds first — primitives (floor, walls) may be
      // queued in _pendingColliderBuilds if they were created before the render loop ran.
      flushPendingColliderBuilds();

      // Chunked snapshot protocol (DSC1) — single-frame send stalls when the
      // browser main thread is CPU-saturated (e.g. headless SwiftShader on a
      // weak runner): WebSocket.bufferedAmount climbs and never drains.
      // Splitting into ~256KB chunks with a setTimeout(0) yield between each
      // lets the WS pump run, and bridge reassembles in receive order.
      // Wire format:
      //   prelude:  [DSC1 4B BE][total u32 LE][sx f32 LE][sy f32 LE][sz f32 LE]   (20B)
      //   chunks:   raw bytes, in order, until `total` accumulated bridge-side.
      const SNAPSHOT_CHUNK_SIZE = 256 * 1024;
      const _waitSensorWs = () => {
        if (bridge.wsSensors && bridge.wsSensors.readyState === WebSocket.OPEN) {
          try {
            const snapshot = rapierWorld.takeSnapshot();
            const [sx, sy, sz] = agent.getPosition?.() || [2, 0.5, 3];
            const total = snapshot.byteLength;

            const prelude = new Uint8Array(20);
            const pdv = new DataView(prelude.buffer);
            pdv.setUint32(0, 0x44534331, false); // "DSC1"
            pdv.setUint32(4, total, true);
            pdv.setFloat32(8, sx, true);
            pdv.setFloat32(12, sy, true);
            pdv.setFloat32(16, sz, true);
            bridge.wsSensors.send(prelude.buffer);

            let sent = 0;
            let chunkN = 0;
            const sendNextChunk = () => {
              if (bridge.wsSensors.readyState !== WebSocket.OPEN) {
                console.warn("[DimosBridge] sensor WS closed mid-snapshot");
                return;
              }
              // Backpressure: don't outpace the WS pump.
              if (bridge.wsSensors.bufferedAmount > 4 * SNAPSHOT_CHUNK_SIZE) {
                setTimeout(sendNextChunk, 50);
                return;
              }
              const end = Math.min(sent + SNAPSHOT_CHUNK_SIZE, total);
              bridge.wsSensors.send(snapshot.subarray(sent, end));
              sent = end;
              chunkN++;
              if (sent >= total) return;
              setTimeout(sendNextChunk, 0); // yield to event loop
            };
            sendNextChunk();
          } catch (e) {
            console.warn("[DimosBridge] snapshot send failed:", e);
          }
        } else {
          setTimeout(_waitSensorWs, 200);
        }
      };
      _waitSensorWs();
      // Expose yaw for lidar pose sampling (avoids reading Three.js Euler)
      Object.defineProperty(window, '__dimosYaw', { get: () => _dimosYaw });

      // Odom: server-side physics publishes odom directly to LCM.
      // Browser no longer needs to publish odom — server is authoritative.

      // Eval harness — scores objectDistance rubric when triggered by dimsim eval runner
      const harnessMod = await import("../evals/harness.ts");
      const { EvalHarness, setEvalHarness } = harnessMod;
      const channel = new URLSearchParams(location.search).get("channel") || undefined;
      const evalHarness = new EvalHarness({
        bridge,
        channel,
        getSceneState: () => {
          const enriched = assets.map(a => {
            const obj = assetsGroup.getObjectByName(`asset:${a.id}`);
            if (obj) {
              const bbox = new THREE.Box3().setFromObject(obj);
              if (!bbox.isEmpty()) {
                const center = new THREE.Vector3();
                const size = new THREE.Vector3();
                bbox.getCenter(center);
                bbox.getSize(size);
                return {
                  ...a,
                  transform: { x: center.x, y: center.y, z: center.z },
                  _bbox: { w: size.x, h: size.y, d: size.z },
                };
              }
            }
            return a;
          });
          return { assets: enriched };
        },
        getAgentPose: () => {
          const pos = agent.getPosition?.();
          if (!pos) return null;
          const camOffset = 0.3;
          const cx = pos[0] + Math.sin(_dimosYaw) * camOffset;
          const cz = pos[2] + Math.cos(_dimosYaw) * camOffset;
          return { x: cx, y: pos[1], z: cz, yaw: _dimosYaw, pitch: 0 };
        },
      });
      // Register the singleton so workflow files importing `runEval` from
      // `@dimsim/eval` (importmap → dist/assets/dimsim-eval.js → this same
      // module) get a working runEval.
      setEvalHarness(evalHarness);

      // Scene editor — script execution engine for sim editing (exec_js API)
      const { SceneEditor } = await import("./sceneEditor.ts");
      const sceneEditor = new SceneEditor({
        bridge,
        channel,
        globals: { scene, THREE, RAPIER, rapierWorld, renderer, camera, agent, assets, assetsGroup, gltfLoader },
      });

      // Agent POV only in headless (sensor capture needs it). Headed = free orbit.
      if (window.__dimosHeadless) {
        enableAgentCameraFollow(agent.id);
      }

      // 7a. dimos mode UI cleanup handled in CSS via body.dimos-mode class
      // (panel hiding) and .shortcuts-floating in index.html (WASD strip).


      console.log("[dimos] Bridge connected. Sensor publishing active.");
    } catch (err) {
      console.error("[dimos] Initialization failed:", err);
    }
  })();
}
