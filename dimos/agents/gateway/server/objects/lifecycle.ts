import type { GatewayProcess } from "#dimos/agents/shared/protocol/lifecycle/contract.ts";
import type { Handlers } from "#dimos/agents/gateway/server/objects/lib/bind.ts";
import type { ServeObject } from "#dimos/agents/gateway/server/objects/base.ts";
import type { Standing } from "#dimos/agents/shared/protocol/actions/standing.ts";
import { LIFECYCLE_ACTIONS } from "#dimos/agents/shared/protocol/actions/lifecycle.ts";
import { LifecycleContract } from "#dimos/agents/shared/protocol/lifecycle/contract.ts";
import { bind } from "#dimos/agents/gateway/server/objects/lib/bind.ts";
import { carried } from "#dimos/agents/shared/protocol/schema.ts";
import { done } from "#dimos/agents/shared/protocol/outcome.ts";
import { grantOf } from "#dimos/agents/gateway/auth/principal.ts";

const STOP_AFTER_MS = 100;

/** The gateway process's identity and its one action, stop, which runs the shutdown after the result is written. */
export const serveLifecycle: ServeObject = (host, deps) =>
  host.serve(LifecycleContract, (_input, principal, self) => {
    const process = self.state("process", carried<GatewayProcess>(), deps.process);
    if (grantOf(principal) === "viewer") return { variant: "viewer", process };
    const standing = (): Standing => ({ role: "owner", link: deps.agent.connection.status.value, conversation: null });
    const handlers: Handlers<typeof LIFECYCLE_ACTIONS> = {
      stop: async () => {
        // A tick later than the result: closing the links at once can beat the result frame onto the wire when other
        // clients are attached, and the caller then sees `unknown_outcome` for a stop that happened.
        setTimeout(deps.shutdown, STOP_AFTER_MS);
        return done(null);
      },
    };
    return {
      variant: "owner",
      process,
      ...bind("lifecycle", LIFECYCLE_ACTIONS, handlers, standing, deps.replies, "owner"),
    };
  });
