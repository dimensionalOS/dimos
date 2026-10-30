import type { Agent } from "../../../harness/main.ts";
import type { ConfigStore } from "../../config/store.ts";
import type { GatewayProcess } from "../../../shared/protocol/lifecycle/contract.ts";
import type { Host } from "@earendil-works/chord";
import type { LoginFlows } from "../../auth/dialog.ts";
import type { ModelCatalog } from "../../auth/models.ts";
import type { Models } from "@earendil-works/pi-ai";
import type { Principal } from "../../auth/principal.ts";
import type { Replies } from "../replies.ts";

/** Everything an object's factory and its methods may use: the harness and the gateway's own stores. */
export type ObjectDeps = {
  readonly agent: Agent;
  readonly models: Models;
  readonly catalog: ModelCatalog;
  readonly logins: LoginFlows;
  readonly config: ConfigStore;
  readonly process: GatewayProcess;
  /** One per gateway process: the first outcome of each `by_request_id` call. */
  readonly replies: Replies;
  readonly report: (error: unknown) => void;
  readonly shutdown: () => void;
};

/** Every file of gateway/server/objects serves one Chord type on the gateway's host. server/host.ts calls each. */
export type ServeObject = (host: Host<Principal>, deps: ObjectDeps) => void;
