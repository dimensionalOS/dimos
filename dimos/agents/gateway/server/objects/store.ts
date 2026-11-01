import type { InventoryView } from "#dimos/agents/shared/protocol/inventory/contract.ts";
import type { ServeObject } from "#dimos/agents/gateway/server/objects/base.ts";
import { GatewayState } from "#dimos/agents/shared/protocol/gateway/contract.ts";
import { Type } from "typebox";
import { carried } from "#dimos/agents/shared/protocol/schema.ts";
import { feed } from "#dimos/agents/gateway/server/objects/lib/feed.ts";
import { conversationRows } from "#dimos/agents/gateway/server/objects/lib/conversations.ts";
import { jobRows } from "#dimos/agents/gateway/server/objects/lib/jobs.ts";
import { runtimeView } from "#dimos/agents/gateway/server/objects/lib/runtime.ts";
import { local } from "@earendil-works/chord";

/**
 * The gateway's shared cells, one object for the host's life: they keep collecting while no client is attached. Every
 * per-principal view reaches this one object with `self.get` and returns its members as its own.
 */
export const GatewayStore = local("dimos.gateway.store", {
  version: 1,
  input: Type.Object({}),
  members: { ...GatewayState, inventory: carried<InventoryView>() },
});

export const serveStore: ServeObject = (host, deps) =>
  host.serve(
    GatewayStore,
    (_input, _principal, self) => {
      const { agent } = deps;
      const link = feed(self, "link", GatewayState.link, agent.connection.status);
      const runtime = runtimeView(self, link, {
        connection: agent.connection,
        runtime: agent.runtime,
        stateDir: agent.stateDir,
        pollMs: 30_000,
        report: deps.report,
      });
      return {
        link,
        load: feed(self, "load", GatewayState.load, agent.loader.status),
        runs: feed(self, "runs", GatewayState.runs, agent.runs.status),
        conversations: conversationRows(self, agent.harness, agent.conversations),
        modules: self.derive([runtime], (view) => view.modules),
        streams: self.derive([runtime], (view) => view.streams),
        inventory: self.derive([runtime], (view) => view.inventory),
        jobs: jobRows(self, agent.harness),
        settings: feed(self, "settings", GatewayState.settings, deps.config.status),
        accounts: feed(self, "accounts", GatewayState.accounts, deps.catalog.status),
      };
    },
    { lifetime: "host" },
  );
