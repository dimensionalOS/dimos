import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import type { WorldDefinition, WorldSnapshot } from "./worldModel.ts";
import { geometry, readMeshes } from "./sceneVisuals.ts";
import { batchStaticMeshes } from "./sceneBatch.ts";
import { DuckFollowCamera } from "./followCamera.ts";
import { ScorerBoard } from "./scorerBoard.ts";
import { FrameRate } from "./frameRate.ts";
import { duckColor } from "./duckAppearance.ts";
import { PlayerTags } from "./playerTags.ts";
import type { PlayerIdentity } from "./players.ts";

export type CameraMode = "explore" | "follow";
interface BodyView {
  group: THREE.Group;
  position: THREE.Vector3;
  quaternion: THREE.Quaternion;
}

export class WorldRenderer {
  readonly renderer: THREE.WebGLRenderer;
  readonly camera = new THREE.PerspectiveCamera(42, 1, 0.015, 100);
  readonly controls: OrbitControls;
  readonly scene = new THREE.Scene();
  private readonly root = new THREE.Group();
  private readonly bodies: BodyView[] = [];
  private readonly geometries = new Set<THREE.BufferGeometry>();
  private readonly textures = new Map<string, THREE.Texture>();
  private readonly materials = new Set<THREE.Material>();
  private readonly lamps = new Map<
    number,
    { material: THREE.MeshStandardMaterial; color: THREE.Color }
  >();
  private lastLamps = "";
  private readonly scorerBoard = new ScorerBoard();
  private readonly tags: PlayerTags | null;
  private followedRobot: string | null = null;
  private readonly resizeObserver: ResizeObserver;
  private readonly followCamera = new DuckFollowCamera();
  private readonly cameraObstacles: THREE.Object3D[] = [];
  private definition: WorldDefinition | null = null;
  private focusIndex = 0;
  private frame = 0;
  private lastFrame = performance.now();
  private lastInfo = 0;
  private lastState = 0;
  private havePose = false;
  private disposed = false;
  private mode: CameraMode = "explore";
  private active = true;
  private readonly frameRate = new FrameRate();

  constructor(
    private readonly host: HTMLElement,
    private readonly onMode: (mode: CameraMode) => void,
    private readonly view: "world" | "pov" = "world",
    private readonly onFps: (fps: number) => void = () => {},
    private readonly robot: string = "duck1",
  ) {
    this.renderer = new THREE.WebGLRenderer({
      antialias: true,
      powerPreference: "high-performance",
    });
    this.renderer.setPixelRatio(Math.min(devicePixelRatio, 1.5));
    this.renderer.shadowMap.enabled = view === "world";
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    const canvas = this.renderer.domElement;
    canvas.dataset.testid = view === "pov" ? "duck3d-canvas" : "world3d-canvas";
    canvas.tabIndex = view === "pov" ? -1 : 0;
    canvas.setAttribute(
      "aria-label",
      view === "pov"
        ? "Three.js view from the duck camera"
        : "Interactive world. Drag to orbit, scroll to zoom, right-drag to pan.",
    );
    canvas.addEventListener("pointerdown", () => canvas.focus());
    this.host.appendChild(canvas);
    this.scene.add(this.scorerBoard.mesh);
    this.tags = view === "world" ? new PlayerTags(host) : null;
    this.camera.up.set(0, 0, 1);
    if (view === "world") this.camera.near = 0.05;
    this.controls = new OrbitControls(this.camera, canvas);
    this.controls.enabled = view === "world";
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.1;
    this.controls.minDistance = 0.2;
    this.controls.maxDistance = 30;
    this.controls.maxPolarAngle = Math.PI * 0.49;
    this.controls.listenToKeyEvents(canvas);
    // OrbitControls emits start on a press or wheel. A fixed follow view unlocks
    // only after an actual drag, and keeps the same gesture for free orbit/pan.
    let drag: { id: number; x: number; y: number } | null = null;
    canvas.addEventListener("pointerdown", (event) => {
      if (this.mode === "follow" && (event.button === 0 || event.button === 2)) {
        drag = { id: event.pointerId, x: event.clientX, y: event.clientY };
      }
    }, true);
    canvas.addEventListener("pointermove", (event) => {
      if (this.mode !== "follow" || !drag || drag.id !== event.pointerId) return;
      if (Math.hypot(event.clientX - drag.x, event.clientY - drag.y) >= 4) {
        drag = null;
        this.setMode("explore");
      } else event.stopImmediatePropagation();
    }, true);
    const release = () => {
      drag = null;
    };
    canvas.addEventListener("pointerup", release, true);
    canvas.addEventListener("pointercancel", release, true);
    const fill = new THREE.HemisphereLight(0xe8f2ff, 0xa3a087, 2.1);
    fill.position.set(0, 0, 10);
    this.scene.add(fill);
    const sun = new THREE.DirectionalLight(0xfff0d8, 3.5);
    sun.position.set(-5, -6, 10);
    sun.target.position.set(0, 2.1, 0);
    sun.castShadow = true;
    sun.shadow.mapSize.set(2048, 2048);
    Object.assign(sun.shadow.camera, {
      left: -7,
      right: 7,
      top: 7,
      bottom: -7,
      near: 0.1,
      far: 30,
    });
    sun.shadow.bias = -0.00015;
    sun.shadow.normalBias = 0.005;
    this.scene.add(sun, sun.target, this.root);
    this.resizeObserver = new ResizeObserver(() => this.resize());
    this.resizeObserver.observe(host);
    this.resize();
    this.tick();
  }

  load(model: WorldDefinition): void {
    this.clearModel();
    this.definition = model;
    this.scene.background = new THREE.Color(
      model.appearance.background ?? "#dfe8ed",
    );
    this.scene.fog = new THREE.Fog(this.scene.background, 22, 48);
    this.renderer.toneMappingExposure = model.appearance.exposure ?? 1.1;
    const loader = new THREE.TextureLoader();
    for (const [id, png] of Object.entries(model.textures ?? {})) {
      const texture = loader.load("data:image/png;base64," + png);
      texture.flipY = false; // The exported pixels are already in MuJoCo/OpenGL row order.
      texture.colorSpace = THREE.SRGBColorSpace;
      texture.wrapS = texture.wrapT = THREE.RepeatWrapping;
      texture.anisotropy = Math.min(
        8,
        this.renderer.capabilities.getMaxAnisotropy(),
      );
      this.textures.set(id, texture);
    }
    const meshes = readMeshes(model);
    for (const buffer of meshes.values()) this.geometries.add(buffer);
    const byId = new Map<number, BodyView>();
    for (const id of model.bodyIds) {
      const body = {
        group: new THREE.Group(),
        position: new THREE.Vector3(),
        quaternion: new THREE.Quaternion(),
      };
      this.bodies.push(body);
      byId.set(id, body);
      this.root.add(body.group);
    }
    const actorColors = new Map(
      (model.actors ?? []).flatMap((actor) =>
        actor.bodyIds.map((id) => [id, actor.color || "#418ee8"] as const)
      ),
    );
    for (const geom of model.geoms) {
      const buffer = geometry(geom, meshes);
      this.geometries.add(buffer);
      if (geom.texture) {
        const positions = buffer.getAttribute("position");
        const uv = buffer.getAttribute("uv");
        for (let i = 0; i < positions.count; i++) {
          uv.setXY(
            i,
            (positions.getX(i) / (2 * geom.size[0]) + .5) *
              geom.texture.repeat[0],
            (positions.getY(i) / (2 * geom.size[1]) + .5) *
              geom.texture.repeat[1],
          );
        }
        uv.needsUpdate = true;
      }
      const override = model.appearance.colors?.[geom.name];
      const color = override ? new THREE.Color(override) : new THREE.Color().setRGB(
        geom.rgba[0],
        geom.rgba[1],
        geom.rgba[2],
        THREE.SRGBColorSpace,
      );
      const teamColor = actorColors.get(geom.body);
      if (teamColor && geom.kind === "mesh") {
        color.copy(duckColor(geom, teamColor));
      }
      if (geom.kind === "plane") {
        color.set(model.appearance.ground ?? "#cbd8d9");
      }
      const material = new THREE.MeshStandardMaterial({
        color,
        map: geom.texture ? this.textures.get(geom.texture.id) : null,
        roughness: geom.kind === "mesh" ? 0.42 : 0.82,
        metalness: geom.kind === "mesh" ? 0.12 : 0,
        opacity: geom.rgba[3],
        transparent: geom.rgba[3] < 1,
      });
      this.materials.add(material);
      if (geom.name.startsWith("score_")) {
        this.lamps.set(geom.id, { material, color: color.clone() });
        material.color.setRGB(0.018, 0.025, 0.024, THREE.SRGBColorSpace);
      }
      const mesh = new THREE.Mesh(buffer, material);
      mesh.name = geom.name;
      mesh.position.fromArray(geom.position);
      const [w, x, y, z] = geom.quaternion;
      mesh.quaternion.set(x, y, z, w);
      mesh.castShadow = geom.kind !== "plane" && !geom.name.includes("floor");
      mesh.receiveShadow = true;
      byId.get(geom.body)!.group.add(mesh);
    }
    for (const geometry of batchStaticMeshes(byId.get(0)!.group)) {
      this.geometries.add(geometry);
    }
    const robotBodies = new Set(model.actors?.flatMap((actor) => actor.bodyIds) ?? []);
    for (const [id, body] of byId) {
      if (!robotBodies.has(id)) this.cameraObstacles.push(body.group);
    }
    const actor = model.actors?.find((a) => a.id === this.robot);
    this.focusIndex = model.bodyIds.indexOf(
      actor?.focusBody ?? model.focusBody,
    );
    if (this.view === "pov") {
      const pov = actor?.camera ?? model.camera;
      if (!pov) throw new Error("The world model has no duck camera.");
      byId.get(pov.body)!.group.add(this.camera);
      this.camera.position.fromArray(pov.position);
      const [w, x, y, z] = pov.quaternion;
      this.camera.quaternion.set(x, y, z, w);
      this.camera.fov = pov.fovy;
      this.camera.near = pov.near;
      this.camera.far = pov.far;
      this.camera.updateProjectionMatrix();
    } else if (model.appearance.views?.football) this.room("football");
    else this.overview();
    this.tags?.bind(model);
  }

  setPlayers(players: PlayerIdentity[], showNames: boolean): void {
    this.tags?.setPlayers(players, showNames);
  }

  setActive(active: boolean): void {
    this.active = active;
    this.controls.enabled = active && this.view === "world";
    this.frameRate.reset();
    if (!active) this.onFps(0);
  }

  push(snapshot: WorldSnapshot): void {
    this.tags?.push(snapshot);
    this.scorerBoard.update(snapshot.football?.scorers ?? []);
    if (snapshot.poses.length !== this.bodies.length) {
      throw new Error("World state does not match its model.");
    }
    for (let i = 0; i < this.bodies.length; i++) {
      const body = this.bodies[i];
      const p = snapshot.poses[i];
      body.position.set(p[0], p[1], p[2]);
      body.quaternion.set(p[4], p[5], p[6], p[3]).normalize();
      if (
        !this.havePose ||
        body.group.position.distanceToSquared(body.position) > 1
      ) {
        body.group.position.copy(body.position);
        body.group.quaternion.copy(body.quaternion);
      }
    }
    for (const actor of this.definition?.actors ?? []) {
      const live = snapshot.actors?.some((a) => a.id === actor.id && a.active);
      for (const id of actor.bodyIds) {
        const index = this.definition!.bodyIds.indexOf(id);
        if (index >= 0) this.bodies[index].group.visible = !!live;
      }
    }
    if (this.view === "world" && this.definition) {
      const active = this.definition.actors?.filter((actor) =>
        snapshot.actors?.some((state) =>
          state.id === actor.id && state.active
        )
      ) ?? [];
      const follow = active.find((actor) => actor.id === (this.followedRobot ?? this.robot)) ??
        active[0];
      if (follow) {
        this.focusIndex = this.definition.bodyIds.indexOf(follow.focusBody);
      } else if (this.mode === "follow") {
        this.room("football");
      }
    }
    const lit = snapshot.football?.lit ?? [];
    const lampKey = lit.join(",");
    if (lampKey !== this.lastLamps) {
      this.lastLamps = lampKey;
      const enabled = new Set(lit);
      for (const [id, lamp] of this.lamps) {
        lamp.material.color.copy(
          enabled.has(id) ? lamp.color : new THREE.Color().setRGB(
            0.018,
            0.025,
            0.024,
            THREE.SRGBColorSpace,
          ),
        );
        lamp.material.emissive.copy(
          enabled.has(id) ? lamp.color : new THREE.Color(0),
        );
        lamp.material.emissiveIntensity = 0.6;
      }
    }
    this.havePose = true;
    this.lastState = performance.now();
  }

  overview(): void {
    const camera = this.definition?.appearance.camera;
    this.camera.position.fromArray(camera?.position ?? [4.2, -5.2, 4.5]);
    this.controls.target.fromArray(camera?.target ?? [0, 0, 0.2]);
    // Fit the finite world within both axes, including a narrow cockpit panel.
    this.root.updateMatrixWorld(true);
    const bounds = new THREE.Box3();
    this.root.traverseVisible((object) => {
      if (
        object instanceof THREE.Mesh && object.geometry.type !== "PlaneGeometry"
      ) {
        bounds.expandByObject(object);
      }
    });
    if (!bounds.isEmpty()) {
      const radius = bounds.getSize(new THREE.Vector3()).length() / 2;
      const vertical = THREE.MathUtils.degToRad(this.camera.fov) / 2;
      const limitingAngle = Math.min(
        vertical,
        Math.atan(Math.tan(vertical) * this.camera.aspect),
      );
      const distance = radius / Math.sin(limitingAngle) * 1.05;
      const offset = this.camera.position.clone().sub(this.controls.target);
      if (offset.length() < distance) {
        this.camera.position.copy(this.controls.target).add(
          offset.setLength(distance),
        );
      }
    }
    this.setMode("explore");
    this.controls.update();
  }

  room(name: string): void {
    const view = this.definition?.appearance.views?.[name];
    if (!view || this.view !== "world") return;
    this.camera.position.fromArray(view.position);
    this.controls.target.fromArray(view.target);
    if (view.extent) {
      const halfFov = THREE.MathUtils.degToRad(this.camera.fov) / 2;
      const angle = Math.min(
        halfFov,
        Math.atan(Math.tan(halfFov) * this.camera.aspect),
      );
      const distance = new THREE.Vector3(...view.extent).length() /
        (2 * Math.sin(angle)) * 1.02;
      this.camera.position.copy(this.controls.target).add(
        new THREE.Vector3(...view.position).sub(this.controls.target).setLength(
          distance,
        ),
      );
    }
    this.setMode("explore");
    this.controls.update();
  }

  follow(robot?: string): void {
    if (robot && this.definition) {
      const actor = this.definition.actors?.find((actor) => actor.id === robot);
      if (!actor) return;
      this.followedRobot = robot;
      this.focusIndex = this.definition.bodyIds.indexOf(actor.focusBody);
    }
    const body = this.bodies[this.focusIndex];
    if (!body?.group.visible) {
      this.room("football");
      return;
    }
    this.setMode("follow");
    // Flush any remaining orbit damping before applying the locked pose.
    this.controls.update();
    this.updateFollowCamera();
  }

  private updateFollowCamera(): void {
    const body = this.bodies[this.focusIndex];
    if (!body) return;
    this.followCamera.update(
      this.camera,
      this.controls.target,
      body.group.position,
      body.group.quaternion,
      this.cameraObstacles,
    );
  }

  private setMode(mode: CameraMode): void {
    this.mode = mode;
    this.controls.enableDamping = mode !== "follow";
    this.controls.enableZoom = mode !== "follow";
    this.controls.keyPanSpeed = mode === "follow" ? 0 : 7;
    this.host.dataset.mode = mode;
    this.onMode(mode);
  }

  private resize(): void {
    const { width, height } = this.host.getBoundingClientRect();
    if (width < 1 || height < 1) return;
    const drawWidth = this.view === "pov" ? Math.min(width, height * 16 / 9) : width;
    const drawHeight = this.view === "pov" ? drawWidth * 9 / 16 : height;
    this.renderer.setSize(drawWidth, drawHeight);
    this.camera.aspect = drawWidth / drawHeight;
    this.camera.updateProjectionMatrix();
  }

  private tick = (): void => {
    if (this.disposed) return;
    this.frame = requestAnimationFrame(this.tick);
    const now = performance.now();
    const dt = Math.min(0.1, (now - this.lastFrame) / 1000);
    this.lastFrame = now;
    if (document.hidden || !this.active || !this.havePose) {
      this.frameRate.reset();
      return;
    }
    const blend = 1 - Math.exp(-25 * dt);
    for (const body of this.bodies) {
      body.group.position.lerp(body.position, blend);
      body.group.quaternion.slerp(body.quaternion, blend);
    }
    if (this.view === "world") {
      if (this.mode === "follow") this.updateFollowCamera();
      else this.controls.update();
    }
    this.renderer.render(this.scene, this.camera);
    this.tags?.draw(
      this.camera,
      this.bodies,
      this.renderer.domElement.clientWidth,
      this.renderer.domElement.clientHeight,
    );
    const fps = this.frameRate.draw(performance.now());
    if (fps !== null) this.onFps(fps);
    if (now - this.lastInfo > 500) {
      this.lastInfo = now;
      this.host.dataset.camera = this.camera.getWorldPosition(
        new THREE.Vector3(),
      ).toArray().map((
        v,
      ) => v.toFixed(3)).join(",");
      this.host.dataset.focus = this.bodies[this.focusIndex]?.group.position.toArray().map((v) =>
        v.toFixed(3)
      ).join(",") ?? "";
      this.host.dataset.target = this.controls.target.toArray().map((v) => v.toFixed(3)).join(",");
      this.host.dataset.live = this.havePose ? String(now - this.lastState < 1500) : "loading";
      this.host.dataset.frame = String(this.renderer.info.render.frame);
      this.host.dataset.draws = String(this.renderer.info.render.calls);
    }
  };

  private clearModel(): void {
    this.tags?.clear();
    this.cameraObstacles.length = 0;
    this.camera.removeFromParent();
    this.root.clear();
    for (const geometry of this.geometries) geometry.dispose();
    for (const material of this.materials) material.dispose();
    this.geometries.clear();
    this.materials.clear();
    for (const texture of this.textures.values()) texture.dispose();
    this.textures.clear();
    this.lamps.clear();
    this.lastLamps = "";
    this.bodies.length = 0;
    this.havePose = false;
  }

  dispose(): void {
    this.disposed = true;
    cancelAnimationFrame(this.frame);
    this.resizeObserver.disconnect();
    this.controls.dispose();
    this.clearModel();
    this.scene.traverse((object) => {
      if (object instanceof THREE.Light && "shadow" in object) {
        (object as THREE.DirectionalLight).shadow?.dispose();
      }
    });
    this.scorerBoard.dispose();
    this.renderer.dispose();
    this.renderer.forceContextLoss();
    this.renderer.domElement.remove();
    this.tags?.dispose();
  }
}
