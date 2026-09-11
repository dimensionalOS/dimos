import settings from "../../assets/scenes/apartment/multiplayer.json";

export const roster = settings.robots;
export type RobotId = keyof typeof roster;
export const robotIds = Object.keys(roster) as RobotId[];
export const teams = ["red", "blue"] as const;
export const playerName = (id: string) => roster[id as RobotId]?.name ?? id;
