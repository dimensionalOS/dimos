import type { Accounts, LoginState } from "../auth.ts";
import type { StreamRef } from "../streams.ts";
import type { ConversationRow } from "../conversation/summary.ts";
import type { DimosRun } from "../runtime/runs.ts";
import type { JobRow } from "../jobs/state.ts";
import type { LinkStatus } from "./link.ts";
import type { LoadStatus } from "../runtime/load.ts";
import type { ModuleRow } from "../runtime/module.ts";
import type { Settings } from "../settings.ts";
import { GATEWAY_ACTIONS } from "../actions/gateway.ts";
import { Type } from "typebox";
import { carried } from "../schema.ts";
import { hostNamed, remote } from "@earendil-works/chord";
import { methodsOf } from "../actions/methods.ts";

/**
 * Everything shared across conversations, one record of schemas used by the gateway's host-lifetime store and by both
 * variants of the served view, so the members are listed once.
 */
export const GatewayState = {
  link: carried<LinkStatus>(),
  load: carried<LoadStatus>(),
  runs: carried<readonly DimosRun[]>(),
  conversations: carried<readonly ConversationRow[]>(),
  modules: carried<readonly ModuleRow[]>(),
  streams: carried<readonly StreamRef[]>(),
  jobs: carried<readonly JobRow[]>(),
  settings: carried<Settings>(),
  accounts: carried<Accounts>(),
} as const;

/** Per principal: only the terminal that started a login sees its prompts. */
export const GatewayPrincipalState = { loginFlow: carried<LoginState>() } as const;

/**
 * The gateway as one client sees it. `owner` is granted to every terminal on the 0600 socket and to browsers with an
 * operator token; `viewer` to browsers with a view token.
 */
export const GatewayContract = remote("dimos.gateway", {
  version: 1,
  input: Type.Object({}),
  host: hostNamed("gateway"),
  variants: {
    owner: { state: { ...GatewayState, ...GatewayPrincipalState }, methods: methodsOf(GATEWAY_ACTIONS, "owner") },
    viewer: { state: { ...GatewayState, ...GatewayPrincipalState } },
  },
});
