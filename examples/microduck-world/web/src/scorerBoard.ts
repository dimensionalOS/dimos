import * as THREE from "three";

/** Cosmetic wall display. All scores arrive from the authoritative DimOS server. */
export class ScorerBoard {
  readonly mesh: THREE.Mesh;
  private canvas = document.createElement("canvas");
  private texture: THREE.CanvasTexture;
  private last = "";
  constructor() {
    this.canvas.width = 1024; this.canvas.height = 640;
    this.texture = new THREE.CanvasTexture(this.canvas);
    this.texture.colorSpace = THREE.SRGBColorSpace;
    this.mesh = new THREE.Mesh(new THREE.PlaneGeometry(2.2, 1.15), new THREE.MeshBasicMaterial({ map: this.texture }));
    this.mesh.rotation.x = Math.PI / 2;
    this.mesh.position.set(2.08, 6.38, 0.7);
    this.update([]);
  }
  update(rows: { handle: string; goals: number }[]) {
    const key = JSON.stringify(rows);
    if (key === this.last) return;
    this.last = key;
    const ctx = this.canvas.getContext("2d")!;
    ctx.fillStyle = "#10272c"; ctx.fillRect(0, 0, 1024, 640);
    ctx.strokeStyle = "#96cde8"; ctx.lineWidth = 5; ctx.strokeRect(10, 10, 1004, 620);
    ctx.fillStyle = "#f2f6ed"; ctx.font = "bold 60px monospace"; ctx.fillText("GOAL SCORERS", 40, 70);
    ctx.fillStyle = "#96cde8"; ctx.font = "30px monospace"; ctx.fillText("ALL-TIME · GITHUB", 40, 112);
    ctx.font = "48px monospace";
    rows.slice(0, 8).forEach((row, index) => {
      ctx.fillStyle = "#f2f6ed"; ctx.textAlign = "left";
      ctx.fillText(`@${row.handle}`, 40, 178 + index * 57, 820);
      ctx.textAlign = "right"; ctx.fillText(String(row.goals), 975, 178 + index * 57);
    });
    ctx.textAlign = "left";
    if (!rows.length) { ctx.fillStyle = "#a9bec1"; ctx.fillText("First goal is yours.", 40, 190); }
    this.texture.needsUpdate = true;
  }
  dispose() { this.texture.dispose(); this.mesh.geometry.dispose(); (this.mesh.material as THREE.Material).dispose(); }
}
