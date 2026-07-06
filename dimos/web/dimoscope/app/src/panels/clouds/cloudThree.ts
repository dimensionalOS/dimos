// three.js point-cloud viewer for CloudCompare — one controller drives all three cells so they share
// a single orbit camera (drag/zoom/pan any cell → all move together). Each cell is its own canvas +
// renderer + scene (a THREE.Points fed the frame-synced xyz + a grid floor for spatial reference).
// three is loaded lazily (dynamic import) so it code-splits out of the main bundle, mirroring how the
// Draco wasm is loaded. World is z-up (robot frame); height (z) drives the blue→amber colour ramp.
import type * as THREE from "three";
import { POINTS_FRAG, POINTS_VERT } from "./pointsShader";

type Vec3 = [number, number, number];

export interface CloudTrio {
  ok: boolean;
  /** Upload cell i's points (interleaved xyz), colouring against a shared z range. No-op if unchanged. */
  setFrame(i: number, xyz: Float32Array | null, zRange: [number, number]): void;
  /** Frame the orbit camera + size the grid to the cloud (call once per source / on reset). */
  setTarget(center: Vec3, radius: number): void;
  render(): void;
  resize(): void;
  dispose(): void;
}

const VERT = POINTS_VERT;
const FRAG = POINTS_FRAG;

/** Build the shared-camera three.js trio, or null if three / WebGL is unavailable (→ 2D fallback). */
export async function createCloudTrio(
  canvases: HTMLCanvasElement[],
  orbitEl: HTMLElement,
): Promise<CloudTrio | null> {
  let T: typeof import("three");
  let OrbitControls: typeof import("three/examples/jsm/controls/OrbitControls.js").OrbitControls;
  try {
    T = await import("three");
    ({ OrbitControls } = await import("three/examples/jsm/controls/OrbitControls.js"));
  } catch (e) {
    console.warn("[cloudThree] three.js unavailable", e);
    return null;
  }

  const dpr = Math.min(2, window.devicePixelRatio || 1);
  const camera = new T.PerspectiveCamera(50, 1, 0.05, 2000);
  camera.up.set(0, 0, 1); // z-up world
  camera.position.set(6, -6, 4);

  const renderers: THREE.WebGLRenderer[] = [];
  const scenes: THREE.Scene[] = [];
  const points: THREE.Points[] = [];
  const grids: THREE.GridHelper[] = [];
  const materials: THREE.ShaderMaterial[] = [];
  const lastXyz: (Float32Array | null)[] = [null, null, null];

  try {
    for (const canvas of canvases) {
      const renderer = new T.WebGLRenderer({ canvas, antialias: true, alpha: false });
      renderer.setPixelRatio(dpr);
      renderer.setClearColor(0x0a0d13, 1);
      const scene = new T.Scene();
      const grid = new T.GridHelper(20, 20, 0x2c3a52, 0x1a2130); // --line-hi / near --line
      grid.rotation.x = Math.PI / 2; // GridHelper is XZ by default → lay it on the XY floor
      (grid.material as THREE.Material).transparent = true;
      (grid.material as THREE.Material).opacity = 0.5;
      scene.add(grid);
      const material = new T.ShaderMaterial({
        uniforms: { uZ: { value: new T.Vector2(0, 1) }, uSize: { value: 2.6 * dpr } },
        vertexShader: VERT,
        fragmentShader: FRAG,
      });
      const geom = new T.BufferGeometry();
      geom.setAttribute("position", new T.BufferAttribute(new Float32Array(0), 3));
      const pts = new T.Points(geom, material);
      pts.frustumCulled = false;
      scene.add(pts);
      renderers.push(renderer);
      scenes.push(scene);
      points.push(pts);
      grids.push(grid);
      materials.push(material);
    }
  } catch (e) {
    console.warn("[cloudThree] WebGL renderer init failed", e);
    renderers.forEach((r) => r.dispose());
    return null;
  }

  const controls = new OrbitControls(camera, orbitEl);
  controls.enableDamping = true;
  controls.dampingFactor = 0.09;
  controls.rotateSpeed = 0.7;
  controls.zoomSpeed = 0.9;

  const resize = () => {
    let aspect = 1;
    for (const r of renderers) {
      const c = r.domElement;
      const w = Math.max(1, c.clientWidth), h = Math.max(1, c.clientHeight);
      r.setSize(w, h, false);
      aspect = w / h;
    }
    camera.aspect = aspect;
    camera.updateProjectionMatrix();
  };

  return {
    ok: true,
    setFrame(i, xyz, zRange) {
      const m = materials[i];
      if (m) (m.uniforms.uZ.value as THREE.Vector2).set(zRange[0], zRange[1]);
      if (xyz === lastXyz[i]) return;
      lastXyz[i] = xyz;
      const geom = points[i].geometry as THREE.BufferGeometry;
      geom.setAttribute("position", new T.BufferAttribute(xyz ?? new Float32Array(0), 3));
      geom.setDrawRange(0, (xyz?.length ?? 0) / 3);
    },
    setTarget(center, radius) {
      controls.target.set(center[0], center[1], center[2]);
      const d = Math.max(1, radius * 2.4);
      camera.position.set(center[0] + d * 0.7, center[1] - d * 0.7, center[2] + d * 0.5);
      const g = radius * 2.2;
      for (const grid of grids) {
        grid.position.set(center[0], center[1], center[2] - radius); // floor under the cloud
        grid.scale.setScalar(Math.max(0.001, g / 20));
      }
      controls.update();
    },
    render() {
      controls.update();
      for (let i = 0; i < renderers.length; i++) renderers[i].render(scenes[i], camera);
    },
    resize,
    dispose() {
      controls.dispose();
      for (const r of renderers) r.dispose();
      for (const g of points) (g.geometry as THREE.BufferGeometry).dispose();
      for (const m of materials) m.dispose();
    },
  };
}
