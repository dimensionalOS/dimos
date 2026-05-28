"use client";

import { SparkRenderer, SplatMesh } from "@sparkjsdev/spark";
import { useEffect, useRef, useState } from "react";
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";

type Status = "loading" | "ready" | "error";

// Imperative Three.js + Spark canvas. Deliberately NOT react-three-fiber: the
// vanilla path is Spark's documented quickstart and has the fewest moving parts
// (no extend()/JSX-element typing). Mounted only on the client via a parent
// dynamic import with ssr:false — Spark needs WebGL/WASM/window.
export default function SplatCanvas({
  url,
  flip,
}: {
  url: string;
  flip: boolean;
}) {
  const containerRef = useRef<HTMLDivElement>(null);
  const [status, setStatus] = useState<Status>("loading");
  const [progress, setProgress] = useState(0);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    const container = containerRef.current;
    if (!container) return;

    let disposed = false;
    setStatus("loading");
    setProgress(0);
    setError(null);

    const scene = new THREE.Scene();
    const camera = new THREE.PerspectiveCamera(
      60,
      container.clientWidth / container.clientHeight,
      0.01,
      1000,
    );
    camera.position.set(0, 0, 3);

    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.setSize(container.clientWidth, container.clientHeight);
    container.appendChild(renderer.domElement);

    const spark = new SparkRenderer({ renderer });
    scene.add(spark);

    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.1;

    const splat = new SplatMesh({
      url,
      onProgress: (event) => {
        if (!disposed && event.total > 0) {
          setProgress(event.loaded / event.total);
        }
      },
    });
    // Splat coordinate conventions vary by exporter; many real-world captures
    // come in Y-down relative to Three.js. Caller toggles a 180° X flip.
    if (flip) splat.rotation.x = Math.PI;
    scene.add(splat);

    splat.initialized
      .then(() => {
        if (!disposed) setStatus("ready");
      })
      .catch((e: unknown) => {
        if (!disposed) {
          setError(e instanceof Error ? e.message : String(e));
          setStatus("error");
        }
      });

    const onResize = () => {
      const w = container.clientWidth;
      const h = container.clientHeight;
      camera.aspect = w / h;
      camera.updateProjectionMatrix();
      renderer.setSize(w, h);
    };
    window.addEventListener("resize", onResize);

    renderer.setAnimationLoop(() => {
      controls.update();
      renderer.render(scene, camera);
    });

    return () => {
      disposed = true;
      window.removeEventListener("resize", onResize);
      renderer.setAnimationLoop(null);
      controls.dispose();
      splat.dispose();
      renderer.dispose();
      if (renderer.domElement.parentElement === container) {
        container.removeChild(renderer.domElement);
      }
    };
  }, [url, flip]);

  return (
    <div className="relative h-full w-full">
      <div className="h-full w-full" ref={containerRef} />
      {status !== "ready" ? (
        <div className="pointer-events-none absolute inset-0 flex items-center justify-center bg-black/30 text-sm text-white">
          {status === "error"
            ? `Failed to load splat: ${error}`
            : `Loading splat… ${Math.round(progress * 100)}%`}
        </div>
      ) : null}
      <div className="pointer-events-none absolute bottom-2 left-2 text-white/60 text-xs">
        drag to orbit · scroll to zoom · right-drag to pan
      </div>
    </div>
  );
}
