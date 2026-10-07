// The three.js half of the map3d panel, imported lazily by the sink so pages
// without a map3d panel never download it. Rendering is event-driven: the
// sink renders on new data, on resizes and on orbit changes. There is no
// animation loop, so an idle panel costs no GPU.

import {
  BufferAttribute,
  BufferGeometry,
  ConeGeometry,
  GridHelper,
  type Material,
  Mesh,
  MeshBasicMaterial,
  PerspectiveCamera,
  Points,
  PointsMaterial,
  Scene,
  WebGLRenderer,
} from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import {
  type Bounds,
  boundsOf,
  carryView,
  clipPlanes,
  fitView,
  followView,
  NEAR_MIN,
  pointSize,
  type Pose3d,
  unionBounds,
  type View,
  type VoxelScene,
} from "./voxelView.ts";

const FOV = 50;
// The panel canvas is black like the map2d one; the grid picks the panel
// background tones (--bg-3 and --bg-2 in index.css) so it reads as a floor
// reference and not as data.
const BACKGROUND = 0x000000;
const GRID_CENTER = 0x22272f;
const GRID_LINES = 0x191d23;
const GRID_SIZE_M = 40;
// The grid is drawn too, so it shares the voxels' clip planes.
const GRID_BOUNDS: Bounds = {
  min: [-GRID_SIZE_M / 2, -GRID_SIZE_M / 2, 0],
  max: [GRID_SIZE_M / 2, GRID_SIZE_M / 2, 0],
};
// POSE_COLOR of the 2D map marker.
const POSE_COLOR = 0xff5c5c;

// three.js reads vertex colours as linear and encodes the output to sRGB,
// which would lift the palette's dark end (byte 18 comes out as 75). The
// panel's colour bytes are sRGB, so they go through this transfer first.
const SRGB_TO_LINEAR = new Uint8Array(256);
for (let i = 0; i < 256; i++) {
  const c = i / 255;
  const linear = c <= 0.04045 ? c / 12.92 : ((c + 0.055) / 1.055) ** 2.4;
  SRGB_TO_LINEAR[i] = Math.round(linear * 255);
}

/** Throws when WebGL is unavailable (the sink reports it). */
export function createVoxelScene(canvas: HTMLCanvasElement): VoxelScene {
  const renderer = new WebGLRenderer({ canvas, antialias: false });
  renderer.setClearColor(BACKGROUND);
  const scene = new Scene();
  const camera = new PerspectiveCamera(FOV, 1, NEAR_MIN, 1000);
  camera.up.set(0, 0, 1); // z-up world; OrbitControls orbit around camera.up
  camera.position.set(5, -5, 4);
  const controls = new OrbitControls(camera, canvas);
  controls.enableDamping = false; // damping needs a render loop
  const grid = new GridHelper(GRID_SIZE_M, GRID_SIZE_M, GRID_CENTER, GRID_LINES);
  grid.rotation.x = Math.PI / 2; // three's grid lies in XZ; ours is the XY floor
  scene.add(grid);
  const material = new PointsMaterial({ size: 1, sizeAttenuation: true, vertexColors: true });
  const points = new Points(new BufferGeometry(), material);
  points.frustumCulled = false; // one object: culling would only cost a bounding sphere
  scene.add(points);
  const marker = new Mesh(
    new ConeGeometry(0.12, 0.4, 12),
    new MeshBasicMaterial({ color: POSE_COLOR }),
  );
  marker.visible = false;
  scene.add(marker);
  let bounds: Bounds | null = null;

  // The clip planes follow the drawn box and the camera, so neither a map
  // fitted from kilometres away nor one zoomed or panned to is clipped.
  const updateClipping = (): void => {
    const box = bounds === null ? GRID_BOUNDS : unionBounds(bounds, GRID_BOUNDS);
    const { x, y, z } = camera.position;
    const { near, far } = clipPlanes(box, [x, y, z]);
    camera.near = near;
    camera.far = far;
    camera.updateProjectionMatrix();
  };
  updateClipping();

  const render = (): void => renderer.render(scene, camera);
  controls.addEventListener("change", () => {
    updateClipping(); // orbit, zoom and pan all move the camera relative to the map
    render();
  });
  const currentView = (): View => ({
    target: controls.target.toArray() as [number, number, number],
    position: camera.position.toArray() as [number, number, number],
  });
  const applyView = (view: View): void => {
    controls.target.set(...view.target);
    camera.position.set(...view.position);
    controls.update(); // aims the camera at the target and fires "change"
  };

  let lastPose: Pose3d | null = null;
  let following = false;
  // The pose the follow camera was last carried to. Null makes the next pose
  // snap the camera behind the robot again.
  let anchor: Pose3d | null = null;
  const follow = (pose: Pose3d): void => {
    applyView(anchor === null ? followView(pose) : carryView(currentView(), anchor, pose));
    anchor = pose;
  };

  return {
    setVoxels(next, colors, res) {
      bounds = boundsOf(next);
      for (let i = 0; i < colors.length; i++) colors[i] = SRGB_TO_LINEAR[colors[i]];
      const geometry = new BufferGeometry();
      geometry.setAttribute("position", new BufferAttribute(next, 3));
      geometry.setAttribute("color", new BufferAttribute(colors, 3, true));
      points.geometry.dispose(); // frees the previous frame's GPU buffers
      points.geometry = geometry;
      material.size = pointSize(res, FOV);
      updateClipping(); // a map that grew must not be clipped before the next orbit
    },
    setPose(pose: Pose3d | null) {
      lastPose = pose;
      marker.visible = pose !== null;
      if (pose === null) return;
      marker.position.set(pose.x, pose.y, pose.z);
      // The cone's apex points along +y; lay it along +x, then yaw about z.
      marker.rotation.set(0, 0, pose.yaw - Math.PI / 2);
      if (following) follow(pose);
    },
    fit() {
      if (bounds === null) return;
      applyView(fitView(bounds, FOV, camera.aspect));
    },
    setFollow(on) {
      following = on;
      anchor = null;
      if (on && lastPose !== null) follow(lastPose);
    },
    resize(cssWidth, cssHeight, dpr) {
      renderer.setPixelRatio(dpr);
      renderer.setSize(cssWidth, cssHeight, false); // the layout owns the CSS size
      camera.aspect = cssWidth / cssHeight;
      camera.updateProjectionMatrix();
    },
    render,
    dispose() {
      controls.dispose();
      points.geometry.dispose();
      material.dispose();
      marker.geometry.dispose();
      (marker.material as Material).dispose();
      grid.geometry.dispose();
      (grid.material as Material).dispose();
      renderer.dispose();
    },
  };
}
