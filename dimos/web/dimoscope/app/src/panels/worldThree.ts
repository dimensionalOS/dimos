// three.js 3D scene for WorldView — the fused robot scene (lidar cloud, laser scan, costmap ground,
// path, an oriented robot marker + trail, goal ring) in an orbit view, as the 3D counterpart to the
// 2D top-down nav map. World is z-up, x-forward (metres). three is loaded lazily so it stays code-split.
// Consumers feed it each frame from the same topic refs the 2D view uses; click-to-goal is a raycast
// onto the z=0 ground plane. Reuses the height-ramp point shader shared with the Clouds tab.
import type * as THREE from "three";
import { POINTS_FRAG, POINTS_VERT } from "./clouds/pointsShader";

type Vec2 = [number, number];

export interface WorldScene {
  ok: boolean;
  setLidar(xyz: Float32Array | null, zRange: [number, number]): void;
  setScan(xyz: Float32Array | null): void;
  setCostmap(grid: OccupancyGrid | null | undefined): void;
  setPath(poses: { pose: { position: { x: number; y: number } } }[] | null | undefined): void;
  setRobot(x: number, y: number, yaw: number, present: boolean): void;
  setGoal(xy: Vec2 | null): void;
  setFollow(on: boolean): void;
  reframe(): void;
  /** Ground-plane world (x,y) under a client pixel, or null if the ray misses. */
  raycastGround(clientX: number, clientY: number): Vec2 | null;
  render(): void;
  resize(): void;
  dispose(): void;
}

interface OccupancyGrid {
  info: {
    resolution: number;
    width: number;
    height: number;
    origin?: { position?: { x?: number; y?: number } };
  };
  data: ArrayLike<number>;
}

const TRAIL_MAX = 300;

export async function createWorldScene(
  canvas: HTMLCanvasElement,
  orbitEl: HTMLElement,
): Promise<WorldScene | null> {
  let T: typeof import("three");
  let OrbitControls: typeof import("three/examples/jsm/controls/OrbitControls.js").OrbitControls;
  try {
    T = await import("three");
    ({ OrbitControls } = await import("three/examples/jsm/controls/OrbitControls.js"));
  } catch (e) {
    console.warn("[worldThree] three.js unavailable", e);
    return null;
  }

  const dpr = Math.min(2, window.devicePixelRatio || 1);
  let renderer: THREE.WebGLRenderer;
  try {
    renderer = new T.WebGLRenderer({ canvas, antialias: true, alpha: false });
  } catch (e) {
    console.warn("[worldThree] WebGL init failed", e);
    return null;
  }
  renderer.setPixelRatio(dpr);
  renderer.setClearColor(0x0a0d13, 1);

  const scene = new T.Scene();
  scene.add(new T.AmbientLight(0x9fb0cc, 0.85));
  const sun = new T.DirectionalLight(0xffffff, 0.6);
  sun.position.set(3, -4, 8);
  scene.add(sun);

  const camera = new T.PerspectiveCamera(52, 1, 0.05, 3000);
  camera.up.set(0, 0, 1);
  camera.position.set(-6, -6, 5);

  const controls = new OrbitControls(camera, orbitEl);
  controls.enableDamping = true;
  controls.dampingFactor = 0.09;
  controls.target.set(0, 0, 0);

  // grid floor
  const grid = new T.GridHelper(40, 40, 0x2c3a52, 0x161d2b);
  grid.rotation.x = Math.PI / 2;
  (grid.material as THREE.Material).transparent = true;
  (grid.material as THREE.Material).opacity = 0.5;
  scene.add(grid);

  // lidar points (height ramp)
  const lidarMat = new T.ShaderMaterial({
    uniforms: { uZ: { value: new T.Vector2(0, 1) }, uSize: { value: 2.6 * dpr } },
    vertexShader: POINTS_VERT,
    fragmentShader: POINTS_FRAG,
  });
  const lidarGeom = new T.BufferGeometry();
  lidarGeom.setAttribute("position", new T.BufferAttribute(new Float32Array(0), 3));
  const lidar = new T.Points(lidarGeom, lidarMat);
  lidar.frustumCulled = false;
  scene.add(lidar);

  // laser scan points (flat cyan)
  const scanMat = new T.PointsMaterial({ color: 0x27d3e0, size: 3.2 * dpr, sizeAttenuation: false });
  const scanGeom = new T.BufferGeometry();
  scanGeom.setAttribute("position", new T.BufferAttribute(new Float32Array(0), 3));
  const scan = new T.Points(scanGeom, scanMat);
  scan.frustumCulled = false;
  scene.add(scan);

  // costmap plane (canvas texture, rebuilt on change)
  const off = document.createElement("canvas");
  const octx = off.getContext("2d")!;
  const costTex = new T.CanvasTexture(off);
  costTex.flipY = false;
  costTex.magFilter = T.NearestFilter;
  costTex.minFilter = T.LinearFilter;
  const costMat = new T.MeshBasicMaterial({
    map: costTex,
    transparent: true,
    opacity: 0.72,
    depthWrite: false,
  });
  const costMesh = new T.Mesh(new T.PlaneGeometry(1, 1), costMat);
  costMesh.position.z = 0.01;
  costMesh.visible = false;
  scene.add(costMesh);
  let lastGridData: ArrayLike<number> | null = null;

  // path line
  const pathMat = new T.LineBasicMaterial({ color: 0x4ade80 });
  const pathGeom = new T.BufferGeometry();
  pathGeom.setAttribute("position", new T.BufferAttribute(new Float32Array(0), 3));
  const path = new T.Line(pathGeom, pathMat);
  path.visible = false;
  scene.add(path);

  // robot marker (box + heading cone) + trail
  const robot = new T.Group();
  const body = new T.Mesh(
    new T.BoxGeometry(0.7, 0.31, 0.4),
    new T.MeshStandardMaterial({ color: 0xffcb47, roughness: 0.6, metalness: 0.05 }),
  );
  body.position.z = 0.2;
  const head = new T.Mesh(
    new T.ConeGeometry(0.12, 0.32, 16),
    new T.MeshStandardMaterial({ color: 0xffe08a, roughness: 0.5 }),
  );
  head.rotation.z = -Math.PI / 2; // cone points +y by default → aim +x (forward)
  head.position.set(0.5, 0, 0.2);
  robot.add(body, head);
  robot.visible = false;
  scene.add(robot);

  const trailPts: number[] = [];
  const trailMat = new T.LineBasicMaterial({ color: 0x3b6ea5, transparent: true, opacity: 0.7 });
  const trailGeom = new T.BufferGeometry();
  trailGeom.setAttribute("position", new T.BufferAttribute(new Float32Array(0), 3));
  const trail = new T.Line(trailGeom, trailMat);
  trail.visible = false;
  scene.add(trail);

  // goal ring (flat on ground)
  const goalMesh = new T.Mesh(
    new T.RingGeometry(0.14, 0.24, 24),
    new T.MeshBasicMaterial({ color: 0xf472b6, side: T.DoubleSide }),
  );
  goalMesh.position.z = 0.03;
  goalMesh.visible = false;
  scene.add(goalMesh);

  const groundPlane = new T.Plane(new T.Vector3(0, 0, 1), 0);
  const raycaster = new T.Raycaster();
  let follow = true;
  const robotPos = new T.Vector3(0, 0, 0);
  let robotSeen = false;

  const setPositions = (geom: THREE.BufferGeometry, arr: Float32Array | null) => {
    geom.setAttribute("position", new T.BufferAttribute(arr ?? new Float32Array(0), 3));
    geom.setDrawRange(0, (arr?.length ?? 0) / 3);
    geom.computeBoundingSphere();
  };

  const resize = () => {
    const w = Math.max(1, canvas.clientWidth), h = Math.max(1, canvas.clientHeight);
    renderer.setSize(w, h, false);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
  };

  return {
    ok: true,
    setLidar(xyz, zRange) {
      (lidarMat.uniforms.uZ.value as THREE.Vector2).set(zRange[0], zRange[1]);
      setPositions(lidarGeom, xyz);
      lidar.visible = !!xyz && xyz.length > 0;
    },
    setScan(xyz) {
      setPositions(scanGeom, xyz);
      scan.visible = !!xyz && xyz.length > 0;
    },
    setCostmap(gridMsg) {
      if (!gridMsg?.info || !gridMsg.data) {
        costMesh.visible = false;
        return;
      }
      const { resolution: res, width: W, height: H } = gridMsg.info;
      if (gridMsg.data === lastGridData && costMesh.visible) return; // only rebuild on change
      lastGridData = gridMsg.data;
      off.width = W;
      off.height = H;
      const img = octx.createImageData(W, H);
      const d = gridMsg.data;
      for (let i = 0; i < W * H; i++) {
        const occ = d[i] > 50;
        const p = i * 4;
        img.data[p] = 0x39;
        img.data[p + 1] = 0x42;
        img.data[p + 2] = 0x4f;
        img.data[p + 3] = occ ? 255 : 0; // occupied cells opaque, free transparent
      }
      octx.putImageData(img, 0, 0);
      costTex.needsUpdate = true;
      const ox = gridMsg.info.origin?.position?.x ?? 0;
      const oy = gridMsg.info.origin?.position?.y ?? 0;
      costMesh.geometry.dispose();
      costMesh.geometry = new T.PlaneGeometry(W * res, H * res);
      costMesh.position.set(ox + (W * res) / 2, oy + (H * res) / 2, 0.01);
      costMesh.visible = true;
    },
    setPath(poses) {
      if (!poses?.length) {
        path.visible = false;
        return;
      }
      const arr = new Float32Array(poses.length * 3);
      for (let i = 0; i < poses.length; i++) {
        arr[i * 3] = poses[i].pose.position.x;
        arr[i * 3 + 1] = poses[i].pose.position.y;
        arr[i * 3 + 2] = 0.04;
      }
      setPositions(pathGeom, arr);
      path.visible = true;
    },
    setRobot(x, y, yaw, present) {
      robot.visible = present;
      robotSeen = present;
      robotPos.set(x, y, 0);
      if (!present) return;
      robot.position.set(x, y, 0);
      robot.rotation.z = yaw;
      const n = trailPts.length;
      if (n < 3 || Math.hypot(x - trailPts[n - 3], y - trailPts[n - 2]) > 0.02) {
        trailPts.push(x, y, 0.02);
        if (trailPts.length > TRAIL_MAX * 3) trailPts.splice(0, trailPts.length - TRAIL_MAX * 3);
        setPositions(trailGeom, new Float32Array(trailPts));
        trail.visible = trailPts.length > 6;
      }
    },
    setGoal(xy) {
      if (!xy) {
        goalMesh.visible = false;
        return;
      }
      goalMesh.position.set(xy[0], xy[1], 0.03);
      goalMesh.visible = true;
    },
    setFollow(on) {
      follow = on;
    },
    reframe() {
      follow = true;
      const t = robotSeen ? robotPos : new T.Vector3(0, 0, 0);
      controls.target.copy(t);
      camera.position.set(t.x - 5, t.y - 5, t.z + 4);
      controls.update();
    },
    raycastGround(clientX, clientY) {
      const r = canvas.getBoundingClientRect();
      const ndc = new T.Vector2(
        ((clientX - r.left) / r.width) * 2 - 1,
        -((clientY - r.top) / r.height) * 2 + 1,
      );
      raycaster.setFromCamera(ndc, camera);
      const hit = new T.Vector3();
      return raycaster.ray.intersectPlane(groundPlane, hit) ? [hit.x, hit.y] : null;
    },
    render() {
      resize();
      if (follow && robotSeen) {
        const dx = robotPos.x - controls.target.x;
        const dy = robotPos.y - controls.target.y;
        if (Math.abs(dx) + Math.abs(dy) > 1e-4) {
          camera.position.x += dx;
          camera.position.y += dy;
          controls.target.x = robotPos.x;
          controls.target.y = robotPos.y;
        }
      }
      controls.update();
      renderer.render(scene, camera);
    },
    resize,
    dispose() {
      controls.dispose();
      renderer.dispose();
      scene.traverse((o) => {
        const any = o as unknown as { geometry?: THREE.BufferGeometry; material?: THREE.Material };
        any.geometry?.dispose?.();
        any.material?.dispose?.();
      });
      costTex.dispose();
    },
  };
}
