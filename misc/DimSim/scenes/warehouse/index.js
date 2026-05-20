// scenes/warehouse/index.js — pure JS dev cycle.
//
// Everything is built directly from Three.js primitives (no JSON, no
// loadLevel).  Edit the constants at the top, save, and the engine
// re-imports this file on hot-reload.
//
// NOTE: meshes added with `scene.add` are NOT registered in the engine's
// interaction registry (assets[]), so E-key pickup / state toggles won't
// see them.  That gap is the "registerAsset API" item on the roadmap.
// For warehouse-style scenes (move around, look at stuff) this is fine.

const FLOOR  = { width: 30, depth: 40 };
const WALL_H = 6;
const SKY    = {
  topColor:    '#3a4654',
  horizonColor:'#b8c0c8',
  bottomColor: '#586470',
  brightness:  0.7,
  softness:    0.9,
  sunStrength: 0.25,
  sunHeight:   0.6,
};

export default async function build({ scene, THREE, physics, setSky, setEmbodiment, loadGLTF }) {
  setSky(SKY);

  setEmbodiment({
    embodimentType: 'ground',
    radius: 0.3,
    halfHeight: 0.85,
    lidarMountHeight: 1.6,
    gravity: -9.81,
    maxSpeed: 1.4,
    turnRate: 2.5,
  });

  // ── Floor ────────────────────────────────────────────────────────────────
  const floor = new THREE.Mesh(
    new THREE.BoxGeometry(FLOOR.width, 0.2, FLOOR.depth),
    new THREE.MeshPhysicalMaterial({ color: 0x6b6f74, roughness: 0.95, metalness: 0.05 }),
  );
  floor.position.y = -0.1;
  floor.receiveShadow = true;
  scene.add(floor);
  physics.staticCollider(floor, 'box');

  // ── Walls (north, south, east, west) ────────────────────────────────────
  const wallMat = new THREE.MeshPhysicalMaterial({ color: 0xc4c1b8, roughness: 0.8 });
  const wallSpecs = [
    // [w, h, d,  x,            y,         z]
    [FLOOR.width, WALL_H, 0.3,  0,            WALL_H / 2,  FLOOR.depth / 2],
    [FLOOR.width, WALL_H, 0.3,  0,            WALL_H / 2, -FLOOR.depth / 2],
    [0.3, WALL_H, FLOOR.depth,  FLOOR.width / 2,  WALL_H / 2, 0],
    [0.3, WALL_H, FLOOR.depth, -FLOOR.width / 2,  WALL_H / 2, 0],
  ];
  for (const [w, h, d, x, y, z] of wallSpecs) {
    const wall = new THREE.Mesh(new THREE.BoxGeometry(w, h, d), wallMat);
    wall.position.set(x, y, z);
    wall.castShadow = wall.receiveShadow = true;
    scene.add(wall);
    physics.staticCollider(wall, 'box');
  }

  // ── Pallet racks: 3 rows of vertical posts + shelves ─────────────────────
  // A rack = two side posts, 3 horizontal beams, optional crates on shelves.
  const rackMat   = new THREE.MeshPhysicalMaterial({ color: 0xd4823a, roughness: 0.5, metalness: 0.4 });
  const crateMat  = new THREE.MeshPhysicalMaterial({ color: 0x8b5a2b, roughness: 0.75 });
  const palletMat = new THREE.MeshPhysicalMaterial({ color: 0x6b4423, roughness: 0.9 });

  const rackBays   = 4;
  const bayWidth   = 2.4;
  const rackDepth  = 1.2;
  const rackHeight = 5.0;
  const shelfYs    = [1.5, 3.0, 4.5];

  function addRack(originX, originZ) {
    // Vertical posts at each bay edge
    const postGeo = new THREE.BoxGeometry(0.1, rackHeight, 0.1);
    for (let i = 0; i <= rackBays; i++) {
      for (const dz of [-rackDepth / 2, rackDepth / 2]) {
        const post = new THREE.Mesh(postGeo, rackMat);
        post.position.set(originX + i * bayWidth, rackHeight / 2, originZ + dz);
        post.castShadow = true;
        scene.add(post);
        physics.staticCollider(post, 'box');
      }
    }
    // Horizontal beams per shelf level
    const beamGeo = new THREE.BoxGeometry(rackBays * bayWidth, 0.08, 0.08);
    for (const y of shelfYs) {
      for (const dz of [-rackDepth / 2, rackDepth / 2]) {
        const beam = new THREE.Mesh(beamGeo, rackMat);
        beam.position.set(originX + (rackBays * bayWidth) / 2, y, originZ + dz);
        beam.castShadow = true;
        scene.add(beam);
        physics.staticCollider(beam, 'box');
      }
    }
    // Shelf decks
    const deckGeo = new THREE.BoxGeometry(rackBays * bayWidth, 0.04, rackDepth);
    for (const y of shelfYs) {
      const deck = new THREE.Mesh(deckGeo, palletMat);
      deck.position.set(originX + (rackBays * bayWidth) / 2, y - 0.05, originZ);
      deck.castShadow = deck.receiveShadow = true;
      scene.add(deck);
      physics.staticCollider(deck, 'box');
    }
    // Crates on a few of the shelves (skip some bays for variety)
    const occupied = [
      [0, 0], [0, 1], [0, 3],
      [1, 0], [1, 2],
      [2, 1], [2, 2], [2, 3],
    ];
    const crateGeo = new THREE.BoxGeometry(bayWidth * 0.85, 0.7, rackDepth * 0.85);
    for (const [shelfIdx, bayIdx] of occupied) {
      const crate = new THREE.Mesh(crateGeo, crateMat);
      crate.position.set(
        originX + bayIdx * bayWidth + bayWidth / 2,
        shelfYs[shelfIdx] + 0.35,
        originZ,
      );
      crate.castShadow = crate.receiveShadow = true;
      scene.add(crate);
      physics.staticCollider(crate, 'box');
    }
  }

  addRack(-FLOOR.width / 2 + 3, -10);
  addRack(-FLOOR.width / 2 + 3,   0);
  addRack(-FLOOR.width / 2 + 3,  10);

  // ── Loose pallet stacks near the dock ────────────────────────────────────
  for (let i = 0; i < 4; i++) {
    const stackX = 8 + (i % 2) * 1.6;
    const stackZ = -14 + Math.floor(i / 2) * 1.6;
    for (let h = 0; h < 3; h++) {
      const c = new THREE.Mesh(
        new THREE.BoxGeometry(1.2, 0.4, 1.2),
        crateMat,
      );
      c.position.set(stackX, 0.2 + h * 0.42, stackZ);
      c.castShadow = c.receiveShadow = true;
      scene.add(c);
      physics.staticCollider(c, 'box');
    }
  }

  // ── Loading-dock door (visual only — flat panel on east wall) ───────────
  const dockDoor = new THREE.Mesh(
    new THREE.BoxGeometry(0.05, 3.6, 3.6),
    new THREE.MeshPhysicalMaterial({ color: 0x3a3f47, roughness: 0.4, metalness: 0.6 }),
  );
  dockDoor.position.set(FLOOR.width / 2 - 0.1, 1.8, 12);
  scene.add(dockDoor);

  const ball = new THREE.Mesh(
    new THREE.SphereGeometry(0.4, 32, 32),
    new THREE.MeshPhysicalMaterial({ color: 0x2196f3, roughness: 0.35, metalness: 0.1 }),
  );
  ball.position.set(0, 3.0, -10);
  ball.castShadow = ball.receiveShadow = true;
  scene.add(ball);
  physics.dynamicCollider(ball, { shape: 'sphere', mass: 1.0, restitution: 0.6 });

  // ── Cross-scene GLB import test: pull the sectional out of apartment ─────
  // The dedup'd GLB references textures via `../_textures/<hash>.png` which
  // resolves relative to the GLB's URL — so loading it from any other scene
  // still resolves correctly into /scenes/apartment/objects/_textures/.
  const sectional = await loadGLTF('../apartment/objects/modern-l-shaped-sectional/state-default.glb');
  sectional.scene.traverse((o) => {
    if (o.isMesh) { o.castShadow = true; o.receiveShadow = true; }
  });
  sectional.scene.position.set(0, 0.5, -8);
  scene.add(sectional.scene);
  physics.staticCollider(sectional.scene, 'trimesh');

  // ── Lights ───────────────────────────────────────────────────────────────
  scene.add(new THREE.HemisphereLight(0xc8d4e0, 0x303030, 0.45));

  const sun = new THREE.DirectionalLight(0xffffff, 0.9);
  sun.position.set(15, 25, 10);
  sun.castShadow = true;
  sun.shadow.mapSize.set(2048, 2048);
  sun.shadow.camera.left   = -20;
  sun.shadow.camera.right  =  20;
  sun.shadow.camera.top    =  25;
  sun.shadow.camera.bottom = -25;
  scene.add(sun);

  // Pendant lights down the centerline
  for (let i = 0; i < 4; i++) {
    const z = -15 + i * 10;
    const housing = new THREE.Mesh(
      new THREE.CylinderGeometry(0.3, 0.45, 0.25, 16),
      new THREE.MeshPhysicalMaterial({ color: 0x202428, roughness: 0.5 }),
    );
    housing.position.set(0, WALL_H - 0.5, z);
    scene.add(housing);

    const bulb = new THREE.PointLight(0xfff2cf, 12, 14, 1.4);
    bulb.position.set(0, WALL_H - 0.8, z);
    bulb.castShadow = false;
    scene.add(bulb);
  }

  return {
    embodiment: null,
    spawnPoint: { x: 0, y: 1.0, z: -15 },
  };
}
