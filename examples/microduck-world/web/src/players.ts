import { createContext } from "react";

export interface PlayerIdentity {
  id: string;
  occupied: boolean;
  displayName: string | null;
  generation: string | null;
}

/** Lobby identity is human UI metadata. It is never added to robot sensor streams. */
export const PlayerDirectory = createContext<PlayerIdentity[]>([]);
