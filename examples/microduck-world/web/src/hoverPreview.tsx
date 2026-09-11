import { useEffect, useRef } from "react";
import * as THREE from "three";
import { geometry, readMeshes } from "./sceneVisuals.ts";
import { duckColor } from "./duckAppearance.ts";
import type { VisualGeom, WorldDefinition } from "./worldModel.ts";
import { type RobotId, roster } from "./roster.ts";

export interface DuckPreview {
  version: number;
  model: WorldDefinition;
  standing: number[][];
  walk: { fps: number; frames: number[][][] };
}

/** One browser renderer serves the hovered card; idle cards remain cached PNGs. */
export function HoverPreview({ asset, active, onPortraits }: {
  asset: DuckPreview | null;
  active: string | null;
  onPortraits: (portraits: Record<string, string>) => void;
}) {
  const current = useRef(active);
  current.current = active;
  const select = useRef<((id: string | null) => void) | null>(null);
  useEffect(() => {
    if (!asset) return;
    let renderer: THREE.WebGLRenderer;
    try {
      renderer = new THREE.WebGLRenderer({ alpha: true, antialias: true });
    } catch {
      return;
    }
    renderer.setPixelRatio(1);
    renderer.outputColorSpace = THREE.SRGBColorSpace;
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    renderer.toneMappingExposure = 1.05;
    const canvas = renderer.domElement;
    canvas.setAttribute("aria-hidden", "true");
    canvas.dataset.testid = "duck-hover-canvas";
    Object.assign(canvas.style, {
      position: "absolute",
      inset: "0",
      pointerEvents: "none",
      zIndex: "2",
      display: "none",
    });
    const scene = new THREE.Scene();
    scene.add(new THREE.HemisphereLight(0xeaf6ff, 0x648096, 2.8));
    const key = new THREE.DirectionalLight(0xffffff, 3.3);
    key.position.set(1, -2, 3);
    scene.add(key);
    const camera = new THREE.PerspectiveCamera(30, 1, .01, 10);
    camera.up.set(0, 0, 1);
    const buffers = readMeshes(asset.model);
    const geometries = new Set(buffers.values());
    const materials: THREE.MeshStandardMaterial[] = [];
    const shells: { material: THREE.MeshStandardMaterial; geom: VisualGeom }[] = [];
    const bodies = new Map<number, THREE.Group>();
    for (const [i, id] of asset.model.bodyIds.entries()) {
      const body = new THREE.Group();
      const pose = asset.standing[i];
      body.position.set(pose[0], pose[1], pose[2]);
      body.quaternion.set(pose[4], pose[5], pose[6], pose[3]);
      bodies.set(id, body);
      scene.add(body);
    }
    for (const geom of asset.model.geoms) {
      const buffer = geometry(geom, buffers);
      geometries.add(buffer);
      const material = new THREE.MeshStandardMaterial({
        color: new THREE.Color().setRGB(
          ...geom.rgba.slice(0, 3) as [number, number, number],
          THREE.SRGBColorSpace,
        ),
        roughness: .38,
        metalness: .1,
      });
      materials.push(material);
      if (geom.kind === "mesh") {
        shells.push({ material, geom });
      }
      const mesh = new THREE.Mesh(buffer, material);
      mesh.position.fromArray(geom.position);
      mesh.quaternion.set(
        geom.quaternion[1],
        geom.quaternion[2],
        geom.quaternion[3],
        geom.quaternion[0],
      );
      bodies.get(geom.body)?.add(mesh);
    }
    const bounds = new THREE.Box3().setFromObject(scene);
    const center = bounds.getCenter(new THREE.Vector3());
    const radius = bounds.getSize(new THREE.Vector3()).length() / 2;
    const direction = new THREE.Vector3(1.3, -2, .7).normalize();
    const tint = (id: string) => {
      const color = roster[id as RobotId]?.team === "red" ? "#e34d4e" : "#418ee8";
      for (const shell of shells) {
        shell.material.color.copy(duckColor(shell.geom, color));
      }
    };
    const frameCamera = (width: number, height: number) => {
      camera.aspect = width / height;
      camera.position.copy(center).addScaledVector(
        direction,
        radius / Math.sin(Math.PI / 12) * .91 / Math.min(1, camera.aspect),
      );
      camera.lookAt(center);
      camera.updateProjectionMatrix();
    };
    renderer.setSize(480, 480);
    frameCamera(480, 480);
    const portraits: Record<string, string> = {};
    for (const id of Object.keys(roster)) {
      tint(id);
      renderer.render(scene, camera);
      portraits[id] = canvas.toDataURL("image/png");
    }
    onPortraits(portraits);
    const reduced = matchMedia("(prefers-reduced-motion: reduce)");
    const nextQ = new THREE.Quaternion();
    let frame = 0, last = 0, start = 0, stopAt = 0;
    let target: HTMLElement | null = null;
    let targetId: string | null = null;
    let lastPose = asset.standing;
    const apply = (a: number[][], b: number[][], amount: number) => {
      for (const [i, id] of asset.model.bodyIds.entries()) {
        const p = a[i], q = b[i], body = bodies.get(id)!;
        body.position.set(
          p[0] + (q[0] - p[0]) * amount,
          p[1] + (q[1] - p[1]) * amount,
          p[2] + (q[2] - p[2]) * amount,
        );
        body.quaternion.set(p[4], p[5], p[6], p[3]);
        nextQ.set(q[4], q[5], q[6], q[3]);
        body.quaternion.slerp(nextQ, amount);
      }
    };
    const hide = () => {
      if (target) delete target.dataset.animated;
      canvas.style.display = "none";
      target = null;
      targetId = null;
      cancelAnimationFrame(frame);
      frame = 0;
    };
    const tick = (now: number) => {
      frame = 0;
      if (document.hidden || !target?.isConnected) {
        hide();
        return;
      }
      if (now - last >= 1000 / 30) {
        last = now;
        const rect = target.getBoundingClientRect();
        if (rect.bottom < 0 || rect.top > innerHeight) {
          hide();
          return;
        }
        const w = Math.max(1, Math.round(rect.width)),
          h = Math.max(1, Math.round(rect.height));
        if (canvas.width !== w || canvas.height !== h) {
          renderer.setSize(w, h);
          frameCamera(w, h);
        }
        if (stopAt) {
          const amount = reduced.matches ? 1 : Math.min(1, (now - stopAt) / 220);
          apply(lastPose, asset.standing, amount);
          if (amount === 1) {
            hide();
            return;
          }
        } else {
          const t = (now - start) / 1000 * asset.walk.fps;
          const i = Math.floor(t) % asset.walk.frames.length;
          const next = asset.walk.frames[(i + 1) % asset.walk.frames.length];
          lastPose = asset.walk.frames[i];
          apply(lastPose, next, t % 1);
          if (!reduced.matches && now - start < 180) {
            const moving = asset.model.bodyIds.map((id) => {
              const b = bodies.get(id)!;
              return [
                ...b.position.toArray(),
                b.quaternion.w,
                b.quaternion.x,
                b.quaternion.y,
                b.quaternion.z,
              ];
            });
            apply(asset.standing, moving, (now - start) / 180);
          }
        }
        renderer.render(scene, camera);
        canvas.style.display = "block";
        target.dataset.animated = "true";
        canvas.dataset.frames = String(Number(canvas.dataset.frames ?? 0) + 1);
      }
      frame = requestAnimationFrame(tick);
    };
    // Hover, keyboard focus and the preview button are deliberate requests to
    // see the gait. Reduced motion still removes the surrounding transitions.
    const change = (id: string | null) => {
      if (id && !document.hidden) {
        const element = document.querySelector<HTMLElement>(
          `[data-preview="${id}"]`,
        );
        if (!element) return;
        if (id === targetId && !stopAt) return;
        if (target) delete target.dataset.animated;
        target = element;
        targetId = id;
        target.appendChild(canvas);
        tint(id);
        start = performance.now();
        stopAt = 0;
        if (!frame) frame = requestAnimationFrame(tick);
      } else if (target) stopAt = performance.now();
    };
    const visibility = () => {
      if (document.hidden) hide();
      else change(current.current);
    };
    select.current = change;
    change(current.current);
    document.addEventListener("visibilitychange", visibility);
    return () => {
      select.current = null;
      hide();
      document.removeEventListener("visibilitychange", visibility);
      for (const g of geometries) g.dispose();
      for (const m of materials) m.dispose();
      renderer.dispose();
      renderer.forceContextLoss();
      canvas.remove();
    };
  }, [asset, onPortraits]);
  useEffect(() => {
    select.current?.(active);
  }, [active, asset]);
  return null;
}
