import type { EpochMs } from "../time.ts";
import type { Static } from "typebox";
import { Type } from "typebox";

/** Why a `dimos agent load` was asked for. */
type LoadTrigger = "gateway_start" | "new_run" | "tools_missing";

export type LoadStatus =
  | { readonly state: "idle" }
  /** No run file in the runs directory names a live process, so nothing is spawned until one does. */
  | { readonly state: "no_system" }
  | { readonly state: "running"; readonly trigger: LoadTrigger; readonly since: EpochMs }
  | { readonly state: "loaded"; readonly result: LoadResult; readonly at: EpochMs }
  | { readonly state: "failed"; readonly error: string; readonly attempt: number; readonly retryAt: EpochMs };

export const LoadResultSchema = Type.Object({ loaded: Type.Array(Type.String()), already: Type.Array(Type.String()) });

/** The one JSON line `dimos agent load` prints: module classes it started and classes already running. */
export type LoadResult = Static<typeof LoadResultSchema>;
