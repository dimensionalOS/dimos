// scenes/apartment/index.js — pure Three.js entry for the apartment.
//
// Static structure (walls, floor, ceiling, fixtures): ./structure.glb
// Interactive objects (fridge, cabinets, TV, etc.): ./objects/<asset>/<state>.glb
// keyed by ./objects/manifest.json. Sky, lights, tags inlined below.

const SKY = {
  topColor:     '#0b64f4',
  horizonColor: '#d9e5f7',
  bottomColor:  '#3b6cce',
  brightness:   0.9,
  softness:     0.75,
  sunStrength:  0.17,
  sunHeight:    0.34,
};

const TAGS = ['modern', 'apartment', 'interior', 'furnished'];

export default async function build({ scene, THREE, physics, setSky, loadGLTF, loadLevel }) {
  setSky(SKY);

  // ── Lights ─────────────────────────────────────────────────────────────────
  const lr1 = new THREE.PointLight(new THREE.Color('#FFFAF0'), 2, 8);
  lr1.position.set(2, 2.8, 2.5);
  scene.add(lr1);

  const lr2 = new THREE.PointLight(new THREE.Color('#FFFAF0'), 0, 8);
  lr2.position.set(4, 2.9, 1.5);
  scene.add(lr2);

  const kit1 = new THREE.PointLight(new THREE.Color('#F0FFFF'), 2.5, 8);
  kit1.position.set(-4, 2.8, 2.5);
  scene.add(kit1);

  const kit2 = new THREE.PointLight(new THREE.Color('#F0FFFF'), 2.5, 8);
  kit2.position.set(-4, 2.9, 3.5);
  scene.add(kit2);

  const bed1 = new THREE.PointLight(new THREE.Color('#FFE4B5'), 2, 8);
  bed1.position.set(-2.5, 2.8, -2.5);
  scene.add(bed1);

  const bed2 = new THREE.PointLight(new THREE.Color('#FFE4B5'), 1.5, 8);
  bed2.position.set(-2.5, 2.9, -3.5);
  scene.add(bed2);

  const path1 = new THREE.PointLight(new THREE.Color('#FFD700'), 1, 3);
  path1.position.set(-2, 0.25, 5.8);
  scene.add(path1);

  const path2 = new THREE.PointLight(new THREE.Color('#FFD700'), 1, 3);
  path2.position.set(0, 0.25, 5.8);
  scene.add(path2);

  const path3 = new THREE.PointLight(new THREE.Color('#FFD700'), 1, 3);
  path3.position.set(2, 0.25, 5.8);
  scene.add(path3);

  const bath1 = new THREE.PointLight(new THREE.Color('#F0FFFF'), 1.5, 6);
  bath1.position.set(3.5, 2.8, -2.5);
  scene.add(bath1);

  const sun = new THREE.DirectionalLight(new THREE.Color('#ffffff'), 1);
  sun.position.set(2.9245321131985382, 14.007232336425105, 16.10510431845022);
  sun.target.position.set(2.9990998365488326, 9.488242347246995, 13.966607385475479);
  sun.castShadow = true;
  sun.shadow.mapSize.set(2048, 2048);
  sun.shadow.camera.left   = -30;
  sun.shadow.camera.right  =  30;
  sun.shadow.camera.top    =  30;
  sun.shadow.camera.bottom = -30;
  scene.add(sun);
  scene.add(sun.target);

  // ── Static structure (walls/floor/ceiling/fixtures, baked from primitives) ──
  const sg = await loadGLTF('./structure.glb');
  sg.scene.traverse((o) => {
    if (o.isMesh) {
      o.castShadow = true;
      o.receiveShadow = true;
    }
  });
  scene.add(sg.scene);
  physics.staticCollider(sg.scene, 'trimesh');

  // ── Interactive objects (GLB-per-state, with actions/state machines) ───────
  const manifest = await fetch('/scenes/apartment/objects/manifest.json').then((r) => r.json());

  const assets = manifest.map((entry) => ({
    id: entry.id,
    title: entry.title,
    transform: entry.transform,
    currentStateId: entry.currentStateId,
    _shapePivotCenter: entry._shapePivotCenter,
    pickable: entry.pickable,
    bumpable: entry.bumpable,
    bumpResponse: entry.bumpResponse,
    bumpDamping: entry.bumpDamping,
    castShadow: entry.castShadow,
    receiveShadow: entry.receiveShadow,
    actions: entry.actions || [],
    states: (entry.states || []).map((s) => ({
      id: s.id,
      name: s.name,
      glbUrl: `/scenes/apartment/objects/${s.file}`,
    })),
  }));

  await loadLevel({
    version: '2.0',
    worldKey: 'default',
    tags: TAGS,
    assets,
  });

  return {
    embodiment: null,
    spawnPoint: { x: 2, y: 0.5, z: 3 },
  };
}
