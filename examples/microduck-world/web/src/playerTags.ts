import * as THREE from "three";
import type { PlayerIdentity } from "./players.ts";
import type { WorldDefinition, WorldSnapshot } from "./worldModel.ts";
import { type RobotId, roster } from "./roster.ts";
import styles from "./world.module.css";

/** Six HTML labels projected through the Three.js camera. No physics or agent input. */
export class PlayerTags {
  private readonly layer = document.createElement("div");
  private readonly point = new THREE.Vector3();
  private players = new Map<string, PlayerIdentity>();
  private actors: WorldSnapshot["actors"] = [];
  private enabled = true;
  private labels = new Map<
    string,
    { element: HTMLDivElement; name: HTMLSpanElement; index: number }
  >();

  constructor(host: HTMLElement) {
    this.layer.className = styles.nameTags;
    this.layer.setAttribute("aria-label", "Player names");
    host.appendChild(this.layer);
  }

  bind(model: WorldDefinition): void {
    this.clear();
    for (const actor of model.actors ?? []) {
      const player = roster[actor.id as RobotId];
      if (!player) continue;
      const element = document.createElement("div");
      element.className = styles.nameTag;
      element.dataset.testid = `player-tag-${actor.id}`;
      element.hidden = true;
      element.style.setProperty(
        "--team",
        player.team === "red" ? "#e34d4e" : "#418ee8",
      );
      const badge = document.createElement("span");
      badge.className = styles.nameBadge;
      badge.textContent = `${
        player.team === "red" ? "R" : "B"
      }${player.number}`;
      const name = document.createElement("span");
      element.append(badge, name);
      this.layer.appendChild(element);
      this.labels.set(actor.id, {
        element,
        name,
        index: model.bodyIds.indexOf(actor.focusBody),
      });
    }
  }

  setPlayers(players: PlayerIdentity[], enabled: boolean): void {
    this.players = new Map(players.map((player) => [player.id, player]));
    this.enabled = enabled;
    this.layer.hidden = !enabled;
  }

  push(snapshot: WorldSnapshot): void {
    this.actors = snapshot.actors ?? [];
  }

  draw(
    camera: THREE.Camera,
    bodies: { group: THREE.Group }[],
    width: number,
    height: number,
  ): void {
    if (!this.enabled) return;
    for (const [id, label] of this.labels) {
      const player = this.players.get(id);
      const actor = this.actors?.find((actor) => actor.id === id);
      // A stale lobby poll must never put the previous occupant's name on a new duck.
      if (
        !player?.occupied || !player.displayName || !actor?.active ||
        player.generation !== actor.generation || !bodies[label.index]
      ) {
        label.element.hidden = true;
        continue;
      }
      bodies[label.index].group.getWorldPosition(this.point);
      this.point.z += .34;
      this.point.project(camera);
      const visible = Math.abs(this.point.x) <= 1 &&
        Math.abs(this.point.y) <= 1 &&
        this.point.z >= -1 && this.point.z <= 1;
      label.element.hidden = !visible;
      if (!visible) continue;
      if (label.name.textContent !== player.displayName) {
        label.name.textContent = player.displayName;
      }
      const x = Math.max(
        65,
        Math.min(width - 65, (this.point.x + 1) * width / 2),
      );
      const y = (1 - this.point.y) * height / 2;
      label.element.style.transform =
        `translate(${x}px, ${y}px) translate(-50%, -100%)`;
    }
  }

  clear(): void {
    this.layer.replaceChildren();
    this.labels.clear();
    this.actors = [];
  }
  dispose(): void {
    this.clear();
    this.layer.remove();
  }
}
