import type { Handlers } from "./lib/bind.ts";
import type { LoginState } from "../../../shared/protocol/auth.ts";
import type { ServeObject } from "./base.ts";
import type { Standing } from "../../../shared/protocol/actions/standing.ts";
import { GATEWAY_ACTIONS } from "../../../shared/protocol/actions/gateway.ts";
import { GatewayContract, GatewayState } from "../../../shared/protocol/gateway/contract.ts";
import { GatewayStore } from "./store.ts";
import { bind } from "./lib/bind.ts";
import { carried } from "../../../shared/protocol/schema.ts";
import { done, refused } from "../../../shared/protocol/outcome.ts";
import { feed } from "./lib/feed.ts";
import { forward } from "./lib/forward.ts";
import { fromJobChange, fromModuleOutcome } from "./lib/outcomes.ts";
import { grantOf } from "../../auth/principal.ts";

const SHARED = Object.keys(GatewayState) as (keyof typeof GatewayState)[];

/**
 * The gateway object of one principal: the store's members as its own, its own login flow, and, for owners, the
 * catalog's actions bound to their executors. Effects come back through the store's watches (a restarted module through
 * `harness_status`, a new job through the jobs document), never through the result. It lingers 30 s after its last
 * reference, so a terminal that reconnects keeps its login, and its disposal cancels a login nobody can answer now.
 */
export const serveGateway: ServeObject = (host, deps) =>
  host.serve(
    GatewayContract,
    (_input, principal, self) => {
      const store = self.get(GatewayStore, {});
      self.resource("login", { version: 1, open: () => principal, close: (owner) => deps.logins.cancel(owner) });
      const state = {
        ...forward(store, SHARED),
        loginFlow: feed(self, "loginFlow", carried<LoginState>(), deps.logins.of(principal)),
      };
      if (grantOf(principal) === "viewer") return { variant: "viewer", ...state };
      const { agent } = deps;
      const standing = (): Standing => ({ role: "owner", link: store.link.value, conversation: null });
      const handlers: Handlers<typeof GATEWAY_ACTIONS> = {
        createConversation: async ({ request, cwd }, call) =>
          done({ conversation: await agent.conversations.start(cwd, request, call.context) }),
        login: ({ provider, method }) => deps.logins.start(principal, provider, method),
        answerLogin: ({ dialog, value }) => deps.logins.answer(principal, dialog, value),
        logout: ({ provider }) => deps.catalog.logout(provider),
        updateSettings: async (change) => {
          const model = change.model;
          if (
            model !== undefined &&
            model !== null &&
            deps.models.getModel(model.provider, model.modelId) === undefined
          )
            return refused({ code: "not_found", what: "model", name: `${model.provider}/${model.modelId}` });
          return done(await deps.config.update(change));
        },
        createModule: async ({ name, path, owner }) =>
          fromModuleOutcome(await agent.runtime.createModule(name, path, owner)),
        restartModule: async ({ name }) => fromModuleOutcome(await agent.runtime.moduleOp("restart", name)),
        unloadModule: async ({ name }) => fromModuleOutcome(await agent.runtime.moduleOp("unload", name)),
        deleteModule: async ({ name }) => fromModuleOutcome(await agent.runtime.moduleOp("delete", name)),
        createJob: async (draft, call) => fromJobChange(await agent.jobs.create(draft, call.context), draft.name),
        updateJob: async (draft, call) => fromJobChange(await agent.jobs.update(draft, call.context), draft.name),
        deleteJob: async ({ name }, call) =>
          (await agent.jobs.remove(name, call.context)) === null
            ? refused({ code: "not_found", what: "job", name })
            : done(null),
        runJob: async ({ name, request }, call) => {
          const outcome = await agent.jobs.run(name, { kind: "requested", request }, call.context);
          return outcome === null ? refused({ code: "not_found", what: "job", name }) : done(outcome);
        },
      };
      return {
        variant: "owner",
        ...state,
        ...bind("gateway", GATEWAY_ACTIONS, handlers, standing, deps.replies, "owner"),
      };
    },
    { linger: 30_000 },
  );
