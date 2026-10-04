import assert from "node:assert/strict";
import test from "node:test";
import * as THREE from "three";
import {
  collectAnnotations,
  collectObjectAnnotations,
  collectWallAnnotations,
  ObjectAnnotations,
} from "./objectAnnotations.js";

/** @param {THREE.Box3} box @param {number[]} min @param {number[]} max */
function assertBox(box, min, max) {
  assert.ok(box.min.distanceTo(new THREE.Vector3(...min)) < 1e-6, `min ${box.min.toArray()}`);
  assert.ok(box.max.distanceTo(new THREE.Vector3(...max)) < 1e-6, `max ${box.max.toArray()}`);
}

test("bounds include nested world transforms but exclude hidden geometry and blob shadows", () => {
  const world = new THREE.Group();
  world.position.set(-5, 4, 2);
  const assetsGroup = new THREE.Group();
  world.add(assetsGroup);
  const root = new THREE.Group();
  root.name = "asset:sofa";
  root.position.set(3, 0.4, -5);
  root.rotation.y = 0.6;
  root.scale.set(2, 1, 1.5);
  const mesh = new THREE.Mesh(new THREE.BoxGeometry(2, 1, 3));
  mesh.position.set(1, 0.5, -2);
  mesh.rotation.y = 0.7;
  root.add(mesh);
  assetsGroup.add(root);
  world.updateMatrixWorld(true);
  const expected = new THREE.Box3().setFromObject(root, true);

  const shadow = new THREE.Mesh(new THREE.BoxGeometry(100, 100, 100));
  shadow.userData.isBlobShadow = true;
  root.add(shadow);
  const hidden = new THREE.Group();
  hidden.visible = false;
  hidden.add(new THREE.Mesh(new THREE.BoxGeometry(200, 200, 200)));
  root.add(hidden);
  const invisible = new THREE.Mesh(new THREE.BoxGeometry(300, 300, 300));
  invisible.material.visible = false;
  root.add(invisible);

  const [annotation] = collectObjectAnnotations([{
    id: "sofa",
    title: "  Sectional sofa  ",
  }], assetsGroup);
  assert.equal(annotation.label, "Sectional sofa");
  assert.equal(annotation.id, "sofa");
  assert.ok(annotation.box.min.distanceTo(expected.min) < 1e-6);
  assert.ok(annotation.box.max.distanceTo(expected.max) < 1e-6);
  root.position.x += 10;
  world.updateMatrixWorld(true);
  assert.deepEqual(
    annotation.box.min.toArray(),
    expected.min.toArray(),
    "v1 annotations are snapshots",
  );
});

test("skip missing, empty and hidden assets, without inventing a semantic label", () => {
  const group = new THREE.Group();
  const empty = new THREE.Group();
  empty.name = "asset:empty";
  group.add(empty);
  const mesh = new THREE.Mesh(new THREE.BoxGeometry(1, 1, 1));
  mesh.name = "asset:unnamed";
  group.add(mesh);
  const hidden = mesh.clone();
  hidden.name = "asset:hidden";
  hidden.visible = false;
  group.add(hidden);
  const annotations = collectObjectAnnotations([
    { id: "missing" },
    { id: "empty" },
    { id: "unnamed" },
    { id: "hidden" },
  ], group);
  assert.equal(annotations.length, 1);
  assert.equal(annotations[0].label, "unnamed");
});

test("viewer pass uses a separate scene and restores renderer state", () => {
  const overlay = new ObjectAnnotations();
  const camera = new THREE.PerspectiveCamera();
  let renders = 0;
  const renderer = {
    autoClear: true,
    render(scene, renderedCamera) {
      renders++;
      assert.equal(scene, overlay.scene);
      assert.equal(renderedCamera, camera);
      assert.equal(this.autoClear, false);
      throw new Error("render failed");
    },
  };
  overlay.render(renderer, camera);
  assert.equal(renders, 0);
  overlay.enabled = true;
  assert.throws(() => overlay.render(renderer, camera), /render failed/);
  assert.equal(renderer.autoClear, true);
  overlay.clear();
  assert.equal(overlay.enabled, false);
});

test("clearing snapshots releases helper geometry, label textures and materials", () => {
  const overlay = new ObjectAnnotations();
  const wire = new THREE.Box3Helper(
    new THREE.Box3(new THREE.Vector3(), new THREE.Vector3(1, 1, 1)),
  );
  const map = new THREE.Texture();
  const label = new THREE.Sprite(new THREE.SpriteMaterial({ map }));
  const disposed = [];
  for (
    const [name, resource] of [
      ["geometry", wire.geometry],
      ["wire", wire.material],
      ["texture", map],
      ["label", label.material],
    ]
  ) {
    resource.addEventListener("dispose", () => disposed.push(name));
  }
  overlay.scene.add(wire, label);
  overlay.enabled = true;
  overlay.clear();
  assert.equal(overlay.scene.children.length, 0);
  assert.deepEqual(disposed.sort(), ["geometry", "label", "texture", "wire"]);
});

test("snapshot reuses the displayed capture and measures once when hidden", (t) => {
  // makeLabel draws onto a canvas; stub the DOM surface it touches.
  const canvasContext = {
    fillRect() {},
    strokeRect() {},
    fillText() {},
    measureText: () => ({ width: 0 }),
  };
  globalThis.document = {
    createElement: () => ({ width: 0, height: 0, getContext: () => canvasContext }),
  };
  t.after(() => {
    delete globalThis.document;
  });
  const group = new THREE.Group();
  const root = new THREE.Group();
  root.name = "asset:crate";
  root.position.set(1, 2, 3);
  root.add(new THREE.Mesh(new THREE.BoxGeometry(2, 4, 6)));
  group.add(root);
  const assets = [{ id: "crate", title: "Crate" }];
  const overlay = new ObjectAnnotations();

  const hidden = overlay.snapshot(assets, group);
  assert.equal(hidden.displayed, false);
  assert.equal(overlay.enabled, false, "snapshot never enables the overlay");
  assert.ok(Number.isFinite(hidden.capturedAt));
  assert.deepEqual(hidden.objects, [
    { id: "crate", label: "Crate", min: [0, 0, 0], max: [2, 4, 6] },
  ]);

  assert.equal(overlay.show(assets, group), 1);
  const shown = overlay.snapshot(assets, group);
  root.position.x += 10;
  group.updateMatrixWorld(true);
  const stale = overlay.snapshot(assets, group);
  assert.equal(stale.displayed, true);
  assert.equal(stale.capturedAt, shown.capturedAt, "capture time is preserved");
  assert.deepEqual(stale.objects, shown.objects);
  assert.deepEqual(stale.objects[0].max, [2, 4, 6], "displayed snapshot is not re-measured");

  overlay.clear();
  assert.deepEqual(overlay.snapshot(assets, group).objects[0].max, [12, 4, 6]);
});

test("walls are collected by name from the structure, never from inside assets", () => {
  const scene = new THREE.Scene();
  const assetsGroup = new THREE.Group();
  scene.add(assetsGroup);
  const asset = new THREE.Group();
  asset.name = "asset:clock";
  const decor = new THREE.Mesh(new THREE.BoxGeometry(1, 1, 1));
  decor.name = "wall-clock"; // part of an asset, not a wall
  asset.add(decor);
  assetsGroup.add(asset);

  const structure = new THREE.Group();
  structure.position.set(10, 0, 0);
  const north = new THREE.Mesh(new THREE.BoxGeometry(4, 2.5, 0.2));
  north.name = "wall-north";
  north.position.set(0, 1.25, -2);
  const floor = new THREE.Mesh(new THREE.BoxGeometry(50, 0.1, 50));
  floor.name = "apartment-floor";
  const yard = new THREE.Group();
  yard.name = "yard-wall-east";
  const post = new THREE.Mesh(new THREE.BoxGeometry(1, 1, 1));
  post.name = "wall-post"; // nested under a wall: bounded with its parent, not twice
  post.position.set(0, 0.5, 3);
  const base = new THREE.Mesh(new THREE.BoxGeometry(1, 1, 1));
  base.position.set(0, 0.5, 0);
  yard.add(base, post);
  const hidden = new THREE.Mesh(new THREE.BoxGeometry(9, 9, 9));
  hidden.name = "wall-hidden";
  hidden.visible = false;
  const wallpaper = new THREE.Mesh(new THREE.BoxGeometry(9, 9, 9));
  wallpaper.name = "wallpaper-roll";
  const dupA = new THREE.Mesh(new THREE.BoxGeometry(1, 1, 1));
  const dupB = new THREE.Mesh(new THREE.BoxGeometry(1, 1, 1));
  dupA.name = dupB.name = "Wall_Extra";
  dupB.position.x = 5;
  structure.add(north, floor, yard, hidden, wallpaper, dupA, dupB);
  scene.add(structure);

  const walls = collectWallAnnotations(scene, assetsGroup);
  assert.deepEqual(
    walls.map((w) => [w.id, w.label]),
    [
      ["wall-north", "wall-north"],
      ["yard-wall-east", "yard-wall-east"],
      ["Wall_Extra", "Wall_Extra"],
      ["Wall_Extra#2", "Wall_Extra"],
    ],
  );
  assertBox(walls[0].box, [8, 0, -2.1], [12, 2.5, -1.9]);
  assertBox(walls[1].box, [9.5, 0, -0.5], [10.5, 1, 3.5]);
  assertBox(walls[3].box, [14.5, -0.5, -0.5], [15.5, 0.5, 0.5]);

  const all = collectAnnotations([{ id: "clock", title: "Clock" }], assetsGroup, scene);
  assert.deepEqual(all.map((a) => a.id), ["clock", "wall-north", "yard-wall-east", "Wall_Extra", "Wall_Extra#2"]);
  assert.deepEqual(collectAnnotations([{ id: "clock" }], assetsGroup).map((a) => a.id), ["clock"]);
});
