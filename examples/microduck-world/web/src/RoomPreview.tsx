import places from "../../assets/scenes/apartment/places.json";
import appearance from "../../assets/scenes/apartment/viewer.json";
import roster from "../../assets/scenes/apartment/multiplayer.json";
import type { WorldDefinition, WorldSnapshot } from "./worldModel.ts";

export const roomNames = Object.keys(places.rooms);
export const objectCount = Object.keys(places.objects).length;
export const duckColors: Record<string, string> = roster.colors;
const rooms = Object.values(places.rooms);
const xmin = Math.min(...rooms.map((r) => r.bounds[0]));
const xmax = Math.max(...rooms.map((r) => r.bounds[1]));
const ymin = Math.min(...rooms.map((r) => r.bounds[2]));
const ymax = Math.max(...rooms.map((r) => r.bounds[3]));
const scale = 216 / Math.max(xmax - xmin, ymax - ymin);
const x = (value: number) => 126 + (value - (xmin + xmax) / 2) * scale;
const y = (value: number) => 126 - (value - (ymin + ymax) / 2) * scale;

/** Human spectator metadata; this is never published into a robot's sensor streams. */
export function RoomPreview({ model, snapshot }: {
  model: WorldDefinition | null;
  snapshot: WorldSnapshot | null;
}) {
  return (
    <svg
      viewBox="0 0 252 252"
      role="img"
      aria-label="Football club, team locker rooms, benchmark wing and active ducks"
    >
      <rect x="10" y="10" width="232" height="232" rx="9" fill="#0f203c" />
      {Object.entries(places.rooms).map(([name, room]) => {
        const [left, right, bottom, top] = room.bounds;
        return (
          <g key={name}>
            <rect
              x={x(left) + 2}
              y={y(top) + 2}
              width={(right - left) * scale - 4}
              height={(top - bottom) * scale - 4}
              rx="4"
              fill={name === "football" ? "#4c9a75" : appearance
                .colors[`floor_${name}` as keyof typeof appearance.colors] ??
                "#9aaeb9"}
              fillOpacity=".78"
            />
            <text
              x={x((left + right) / 2)}
              y={y((bottom + top) / 2) + 3}
              textAnchor="middle"
              fill="#11223c"
              fontSize={name in { living: 1, kitchen: 1, bathroom: 1, office: 1 } ? 6 : 10}
              fontWeight="700"
            >
              {name.includes("corridor") || name === "player_tunnel"
                ? ""
                : name === "red_lockers"
                ? "RED"
                : name === "blue_lockers"
                ? "BLUE"
                : name === "football"
                ? "PITCH"
                : name[0].toUpperCase() + name.slice(1)}
            </text>
          </g>
        );
      })}
      {Object.entries(places.objects).map(([name, point]) => (
        <rect
          key={name}
          x={x(point[0]) - 3}
          y={y(point[1]) - 3}
          width="6"
          height="6"
          rx="1"
          fill="#293654"
        >
          <title>{name.replaceAll("_", " ")}</title>
        </rect>
      ))}
      {(model?.actors ?? []).filter((actor) =>
        snapshot?.actors?.some((a) => a.id === actor.id && a.active)
      ).map((actor) => {
        const pose = snapshot?.poses[model!.bodyIds.indexOf(actor.focusBody)];
        if (!pose) return null;
        return (
          <g
            key={actor.id}
            transform={`translate(${x(pose[0])},${y(pose[1])})`}
          >
            <circle
              r="10"
              fill={duckColors[actor.id]}
              stroke="#142443"
              strokeWidth="2"
            />
            <text
              y="4"
              textAnchor="middle"
              fill="#142443"
              fontSize="11"
              fontWeight="800"
            >
              {actor.id.replace("duck", "")}
            </text>
            <title>{actor.id.replace("duck", "Duck ")}</title>
          </g>
        );
      })}
    </svg>
  );
}
