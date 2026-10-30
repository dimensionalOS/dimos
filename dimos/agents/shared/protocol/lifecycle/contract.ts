import type { EpochMs } from "../time.ts";
import { LIFECYCLE_ACTIONS } from "../actions/lifecycle.ts";
import { Type } from "typebox";
import { carried } from "../schema.ts";
import { hostNamed, remote } from "@earendil-works/chord";
import { methodsOf } from "../actions/methods.ts";

/** The running bundle; gateway/config/build.ts computes it. */
export type BuildInfo = {
  readonly version: string;
  /** SHA-256 prefix of the running bundle. */
  readonly build: string;
  readonly executable: string;
  readonly node: string;
};

export type GatewayProcess = {
  readonly build: BuildInfo;
  readonly pid: number;
  readonly startedAt: EpochMs;
};

export const LifecycleState = { process: carried<GatewayProcess>() } as const;

/**
 * The gateway process as every client of any build reaches it: `dimos agent status` and `stop` hold this object first.
 * Frozen at version 1; anything that may change goes on GatewayContract, so a contract mismatch there still leaves
 * status and stop working.
 */
export const LifecycleContract = remote("dimos.lifecycle", {
  version: 1,
  input: Type.Object({}),
  host: hostNamed("gateway"),
  variants: {
    owner: { state: LifecycleState, methods: methodsOf(LIFECYCLE_ACTIONS, "owner") },
    viewer: { state: LifecycleState },
  },
});
