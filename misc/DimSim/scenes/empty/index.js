// scenes/empty/index.js — empty starter scene.
// Sky + ambient lighting + ground plane.  Use this as a canvas to author
// from scratch via the SceneClient SDK or by editing this file.

const GROUND_TEXTURE_DATA_URL =
  "data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAEAAAABACAIAAAAlC+aJAAAASklEQVR42u3PMQ0AAAgDsPk3DR4Iz5LWQRN4MlUEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBARaA3CyKBJG3poVNVYAAAAASUVORK5CYII=";

// Gradient sky dome — parity with the original JSON empty scene.
const SKY = {
  enabled: true,
  topColor: "#1a6be0",
  horizonColor: "#c8ddf5",
  bottomColor: "#4a7a4a",
  brightness: 1.0,
  softness: 0.6,
  sunStrength: 0.3,
  sunHeight: 0.45,
};

export default async function build({ scene, THREE, physics, setSky }) {
  setSky(SKY);

  scene.add(new THREE.AmbientLight(0xffffff, 0.6));

  const sun = new THREE.DirectionalLight(0xfff5e6, 1.5);
  sun.position.set(10, 20, 10);
  sun.castShadow = true;
  scene.add(sun);

  scene.add(new THREE.HemisphereLight(0x87ceeb, 0x4a7a4a, 0.4));

  const texture = new THREE.TextureLoader().load(GROUND_TEXTURE_DATA_URL);
  texture.wrapS = THREE.RepeatWrapping;
  texture.wrapT = THREE.RepeatWrapping;
  texture.repeat.set(100, 100);

  const ground = new THREE.Mesh(
    new THREE.BoxGeometry(100, 0.2, 100),
    new THREE.MeshStandardMaterial({
      color: 0xffffff,
      roughness: 0.8,
      metalness: 0.0,
      map: texture,
      side: THREE.DoubleSide,
    }),
  );
  ground.position.set(0, -0.1, 0);
  ground.receiveShadow = true;
  scene.add(ground);
  physics.staticCollider(ground, "box");

  return {
    embodiment: null,
    // Three.js coords (Y-up).  This maps to ROS/odom (x, y) = (z, x) = (3, 2)
    // via the bridge's (x,y,z)->(z,x,y) swap, which is the spawn point the
    // dimsim e2e tests assume (e.g. test_dimsim_path_replaning.py places its
    // walls around the robot starting at ROS (3, 2)).  Do not move this to the
    // origin: ROS (0, 0) lands behind the back wall and the robot gets stuck.
    spawnPoint: { x: 2, y: 0.5, z: 3 },
  };
}
