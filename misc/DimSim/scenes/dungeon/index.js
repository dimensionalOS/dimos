const SKY = {
  topColor:    '#1a1a22',
  horizonColor:'#33323a',
  bottomColor: '#0c0c11',
  brightness:  0.4,
  softness:    1.2,
  sunStrength: 0.15,
  sunHeight:   0.7,
};

export default async function build({ scene, THREE, physics, setSky, setEmbodiment, loadGLTF }) {
  setSky(SKY);

  setEmbodiment({
    embodimentType: 'ground',
    radius:           0.25,
    halfHeight:       0.6,
    lidarMountHeight: 1.1,
    gravity:         -9.81,
    maxSpeed:         1.4,
    turnRate:         2.5,
  });

  scene.add(new THREE.AmbientLight(0xffeedd, 0.4));
  const torch = new THREE.DirectionalLight(0xffd28a, 1.4);
  torch.position.set(20, 30, 10);
  torch.castShadow = true;
  torch.shadow.mapSize.set(2048, 2048);
  torch.shadow.camera.left   = -40;
  torch.shadow.camera.right  =  40;
  torch.shadow.camera.top    =  40;
  torch.shadow.camera.bottom = -40;
  scene.add(torch);
  const fill = new THREE.HemisphereLight(0x4a6fa5, 0x1a1418, 0.35);
  scene.add(fill);

  console.log('[dungeon] loading dungeon.glb (~7.6 MB)');
  const gltf = await loadGLTF('./dungeon.glb');
  const dungeon = gltf.scene;
  dungeon.traverse((o) => {
    if (o.isMesh) {
      o.castShadow = true;
      o.receiveShadow = true;
    }
  });
  scene.add(dungeon);
  physics.staticCollider(dungeon, 'trimesh');
  console.log('[dungeon] loaded');

  return {
    embodiment: null,
    spawnPoint: { x: 0, y: 1.0, z: 0 },
  };
}
