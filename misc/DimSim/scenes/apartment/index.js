import { SKY }        from './data/sky.js';
import { TAGS }       from './data/tags.js';
import { GROUPS }     from './data/groups.js';
import { LIGHTS }     from './data/lights.js';
import { PRIMITIVES } from './data/structure.js';
import { ASSETS }     from './data/objects.js';

export default async function build(api) {
  const { scene, THREE, physics, loadLevel, loadGLTF } = api;

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

  const crate = new THREE.Mesh(
    new THREE.BoxGeometry(0.6, 0.6, 0.6),
    new THREE.MeshPhysicalMaterial({ color: 0x8b5a2b, roughness: 0.75 }),
  );
  crate.position.set(3.2, 0.3, 1.5);
  crate.castShadow = crate.receiveShadow = true;
  scene.add(crate);
  physics.staticCollider(crate, 'box');

  try {
    const gltf = await loadGLTF('/agent-model/dimsim_unitree_stub.glb');
    const prop = gltf.scene;
    prop.position.set(-3, 0.3, 0);
    prop.scale.set(0.4, 0.4, 0.4);
    prop.traverse((o) => { if (o.isMesh) { o.castShadow = true; o.receiveShadow = true; } });
    scene.add(prop);
    physics.staticCollider(prop, 'trimesh');
  } catch (e) {
    console.warn('[scene] GLB load failed:', e);
  }

  const accent = new THREE.PointLight(0xff6b6b, 6, 4);
  accent.position.set(1.5, 1.8, 0);
  scene.add(accent);

  return {
    embodiment: null,
    spawnPoint: { x: 2, y: 0.5, z: 3 },
  };
}
