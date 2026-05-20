// scenes/apartment/index.js — entry for the apartment scene.
//
// First half loads the pre-authored apartment geometry (data/*.js) via the
// engine's level loader.  Second half is a live demonstration of the
// standard Three.js dev cycle Lesh asked for in dimos#1691: import
// primitives, add lights, load a GLB, attach physics colliders.  Save the
// file and the browser hot-reloads everything below `await loadLevel(...)`.

import { SKY }        from './data/sky.js';
import { TAGS }       from './data/tags.js';
import { GROUPS }     from './data/groups.js';
import { LIGHTS }     from './data/lights.js';
import { PRIMITIVES } from './data/structure.js';
import { ASSETS }     from './data/objects.js';

export default async function build(api) {
  const { scene, THREE, physics, loadLevel, loadGLTF } = api;

  // ── 1. Load the authored apartment ──────────────────────────────────
  await loadLevel({
    version: '2.0',
    worldKey: 'default',
    tags: TAGS,
    primitives: PRIMITIVES,
    assets: ASSETS,
    lights: LIGHTS,
    groups: GROUPS,
    sceneSettings: { sky: SKY },
  });

  // ── 2. Add a red ball with collisions (Lesh: "New elements") ────────
  // Edit `ballPos` or `ballRadius` and save — HMR re-runs this block and
  // the ball appears in the new spot.
  const ballPos    = { x: 1.5, y: 0.4, z: 0 };
  const ballRadius = 0.25;
  const ball = new THREE.Mesh(
    new THREE.SphereGeometry(ballRadius, 24, 24),
    new THREE.MeshPhysicalMaterial({ color: 0xe53935, roughness: 0.4, metalness: 0.1 }),
  );
  ball.position.set(ballPos.x, ballPos.y, ballPos.z);
  ball.castShadow = ball.receiveShadow = true;
  scene.add(ball);
  physics.staticCollider(ball, 'sphere');

  // ── 3. Add a wooden crate next to the couch (primitive + physics) ──
  const crate = new THREE.Mesh(
    new THREE.BoxGeometry(0.6, 0.6, 0.6),
    new THREE.MeshPhysicalMaterial({ color: 0x8b5a2b, roughness: 0.75 }),
  );
  crate.position.set(3.2, 0.3, 1.5);
  crate.castShadow = crate.receiveShadow = true;
  scene.add(crate);
  physics.staticCollider(crate, 'box');

  // ── 4. Load a GLB and drop it in (Lesh: "Model importing") ──────────
  // Using the shipped robot stub as a sample asset.  Replace with any GLB
  // path under /scenes/apartment/ or any absolute URL the bridge serves.
  try {
    const gltf = await loadGLTF('/agent-model/dimsim_unitree_stub.glb');
    const prop = gltf.scene;
    prop.position.set(-3, 0.3, 0);
    prop.scale.set(0.4, 0.4, 0.4);
    prop.traverse((o) => { if (o.isMesh) { o.castShadow = true; o.receiveShadow = true; } });
    scene.add(prop);
    physics.staticCollider(prop, 'trimesh');
  } catch (e) {
    console.warn('[scene] GLB load failed (ok if the file is missing):', e);
  }

  // ── 5. Add an extra accent light (Lesh: "add a light, reload") ──────
  const accent = new THREE.PointLight(0xff6b6b, 6, 4);
  accent.position.set(1.5, 1.8, 0);
  scene.add(accent);

  return {
    embodiment: null,
    spawnPoint: { x: 2, y: 0.5, z: 3 },
  };
}
