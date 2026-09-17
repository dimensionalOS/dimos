import * as THREE from "three";

/** @typedef {{ id: string, title?: string }} AnnotatedAsset */
/** @typedef {{ id: string, label: string, box: THREE.Box3 }} ObjectAnnotation */

/** Structure nodes named "wall" or with "wall" as a dash/underscore/space-separated part. */
export const WALL_NAME_PATTERN = /(^|[-_ ])wall([-_ ]|$)/i;

/**
 * World-axis-aligned bounds of the visible mesh vertices under `root`,
 * excluding decorative blob shadows. Empty when nothing visible is found.
 * @param {THREE.Object3D} root
 * @returns {THREE.Box3}
 */
function visibleBounds(root) {
  const box = new THREE.Box3();
  const point = new THREE.Vector3();
  /** @param {THREE.Object3D} object */
  function visit(object) {
    if (!object.visible || object.userData.isBlobShadow) return;
    if (object.isMesh && object.geometry?.attributes.position) {
      const materials = Array.isArray(object.material)
        ? object.material
        : [object.material];
      if (
        materials.some((material) =>
          material.visible &&
          !(material.transparent && material.opacity === 0)
        )
      ) {
        for (let i = 0; i < object.geometry.attributes.position.count; i++) {
          object.getVertexPosition(i, point).applyMatrix4(object.matrixWorld);
          if (
            Number.isFinite(point.x) && Number.isFinite(point.y) &&
            Number.isFinite(point.z)
          ) {
            box.expandByPoint(point);
          }
        }
      }
    }
    object.children.forEach(visit);
  }
  visit(root);
  return box;
}

/**
 * Snapshot identified assets. Asset roots already contain the active GLB
 * state and the engine's pivot/placement transforms.
 * @param {AnnotatedAsset[]} assets
 * @param {THREE.Object3D} assetsGroup
 * @returns {ObjectAnnotation[]}
 */
export function collectObjectAnnotations(assets, assetsGroup) {
  assetsGroup.updateWorldMatrix(true, true);
  return assets.flatMap((asset) => {
    const root = assetsGroup.getObjectByName(`asset:${asset.id}`);
    if (!root) return [];
    const box = visibleBounds(root);
    return box.isEmpty()
      ? []
      : [{ id: asset.id, label: asset.title?.trim() || asset.id, box }];
  });
}

/**
 * Snapshot walls baked into the scene structure: every node outside
 * `assetsGroup` whose name matches WALL_NAME_PATTERN, bounded as a whole
 * (its children are not annotated separately). Names are authored in the
 * structure GLB or by SceneClient.add_wall, so they double as IDs;
 * duplicates get a "#n" suffix.
 * @param {THREE.Object3D} scene
 * @param {THREE.Object3D} assetsGroup
 * @returns {ObjectAnnotation[]}
 */
export function collectWallAnnotations(scene, assetsGroup) {
  scene.updateWorldMatrix(true, true);
  const annotations = [];
  const seen = new Map();
  /** @param {THREE.Object3D} object */
  function visit(object) {
    if (object === assetsGroup || !object.visible) return;
    if (WALL_NAME_PATTERN.test(object.name)) {
      const box = visibleBounds(object);
      if (!box.isEmpty()) {
        const count = (seen.get(object.name) ?? 0) + 1;
        seen.set(object.name, count);
        const id = count === 1 ? object.name : `${object.name}#${count}`;
        annotations.push({ id, label: object.name, box });
      }
      return;
    }
    object.children.forEach(visit);
  }
  visit(scene);
  return annotations;
}

/**
 * Everything one snapshot covers: assets first, then walls. `scene` may be
 * omitted for an asset-only snapshot.
 * @param {AnnotatedAsset[]} assets
 * @param {THREE.Object3D} assetsGroup
 * @param {THREE.Object3D} [scene]
 * @returns {ObjectAnnotation[]}
 */
export function collectAnnotations(assets, assetsGroup, scene) {
  return [
    ...collectObjectAnnotations(assets, assetsGroup),
    ...(scene ? collectWallAnnotations(scene, assetsGroup) : []),
  ];
}

/**
 * Plain-JSON form of a snapshot for the bridge control channel. Bounds stay in
 * Three.js Y-up world coordinates; consumers convert to their own frame.
 * @param {ObjectAnnotation[]} annotations
 * @param {number} capturedAt Epoch milliseconds when the bounds were measured.
 * @param {boolean} displayed Whether this is the snapshot currently shown.
 */
export function serializeObjectAnnotations(annotations, capturedAt, displayed) {
  return {
    capturedAt,
    displayed,
    objects: annotations.map(({ id, label, box }) => ({
      id,
      label,
      min: box.min.toArray(),
      max: box.max.toArray(),
    })),
  };
}

/** @param {string} label @param {THREE.Color} color @returns {THREE.Sprite} */
function makeLabel(label, color) {
  const canvas = document.createElement("canvas");
  canvas.width = 512;
  canvas.height = 128;
  const ctx = canvas.getContext("2d");
  ctx.fillStyle = "rgba(8, 15, 24, 0.9)";
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  ctx.strokeStyle = `#${color.getHexString()}`;
  ctx.lineWidth = 5;
  ctx.strokeRect(3, 3, canvas.width - 6, canvas.height - 6);
  ctx.fillStyle = "white";
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  ctx.font = "32px sans-serif";
  const words = label.split(/\s+/);
  const lines = [];
  let line = "";
  for (const word of words) {
    const next = line ? `${line} ${word}` : word;
    if (line && ctx.measureText(next).width > 480) {
      lines.push(line);
      line = word;
    } else line = next;
  }
  lines.push(line);
  const step = Math.min(34, 108 / lines.length);
  lines.forEach((text, i) =>
    ctx.fillText(text, 256, 64 + (i - (lines.length - 1) / 2) * step, 480)
  );
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  const sprite = new THREE.Sprite(
    new THREE.SpriteMaterial({
      map: texture,
      transparent: true,
      depthTest: true,
      depthWrite: false,
      toneMapped: false,
      sizeAttenuation: false,
    }),
  );
  sprite.scale.set(0.24, 0.24 * canvas.height / canvas.width, 1);
  return sprite;
}

export class ObjectAnnotations {
  constructor() {
    // Separate scene: never part of physics, raycasts, or robot sensor captures.
    this.scene = new THREE.Scene();
    this.enabled = false;
    /** @type {ObjectAnnotation[]} */
    this.annotations = [];
    /** @type {number | null} Epoch milliseconds of the displayed snapshot. */
    this.capturedAt = null;
  }

  clear() {
    this.scene.traverse((object) => {
      object.geometry?.dispose();
      object.material?.map?.dispose();
      object.material?.dispose();
    });
    this.scene.clear();
    this.enabled = false;
    this.annotations = [];
    this.capturedAt = null;
  }

  /**
   * @param {AnnotatedAsset[]} assets @param {THREE.Object3D} assetsGroup
   * @param {THREE.Object3D} [scene] Scene whose baked walls are annotated too.
   * @returns {number}
   */
  show(assets, assetsGroup, scene) {
    this.clear();
    const annotations = collectAnnotations(assets, assetsGroup, scene);
    const capturedAt = Date.now();
    annotations.forEach(({ id, label, box }, i) => {
      const color = new THREE.Color().setHSL((i * 0.618034) % 1, 0.8, 0.65);
      const wire = new THREE.Box3Helper(box, color);
      wire.material.depthTest = false;
      wire.material.depthWrite = false;
      wire.material.transparent = true;
      wire.material.opacity = 0.8;
      wire.material.toneMapped = false;
      wire.userData.assetId = id;
      const text = makeLabel(label, color);
      box.getCenter(text.position);
      text.position.y = box.max.y + 0.2;
      text.userData.assetId = id;
      this.scene.add(wire, text);
    });
    this.enabled = annotations.length > 0;
    this.annotations = annotations;
    this.capturedAt = capturedAt;
    return annotations.length;
  }

  /**
   * One-shot export for the bridge. While the overlay is shown this returns
   * the displayed snapshot with its original capture time, so exporting it
   * later does not claim a newer observation; otherwise it measures the
   * current geometry once without enabling the overlay.
   * @param {AnnotatedAsset[]} assets @param {THREE.Object3D} assetsGroup
   * @param {THREE.Object3D} [scene]
   */
  snapshot(assets, assetsGroup, scene) {
    if (this.enabled) {
      return serializeObjectAnnotations(this.annotations, this.capturedAt, true);
    }
    return serializeObjectAnnotations(
      collectAnnotations(assets, assetsGroup, scene),
      Date.now(),
      false,
    );
  }

  /** @param {THREE.WebGLRenderer} renderer @param {THREE.Camera} camera */
  render(renderer, camera) {
    if (!this.enabled) return;
    const autoClear = renderer.autoClear;
    renderer.autoClear = false;
    try {
      renderer.render(this.scene, camera);
    } finally {
      renderer.autoClear = autoClear;
    }
  }
}
