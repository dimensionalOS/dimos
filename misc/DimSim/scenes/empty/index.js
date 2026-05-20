// scenes/empty/index.js — empty starter scene.
// Sky + ambient lighting, no geometry, no embodiment.  Use this as a canvas
// to author from scratch via the SceneClient SDK or by editing this file.

export default async function build({ scene, THREE }) {
  scene.add(new THREE.AmbientLight(0xffffff, 0.6));

  const sun = new THREE.DirectionalLight(0xfff5e6, 1.5);
  sun.position.set(10, 20, 10);
  sun.castShadow = true;
  scene.add(sun);

  scene.add(new THREE.HemisphereLight(0x87ceeb, 0x4a7a4a, 0.4));

  return {
    embodiment: null,
    spawnPoint: { x: 0, y: 0.5, z: 0 },
  };
}
