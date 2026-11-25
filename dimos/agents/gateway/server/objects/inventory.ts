import type { ServeObject } from "#dimos/agents/gateway/server/objects/base.ts";
import { GatewayStore } from "#dimos/agents/gateway/server/objects/store.ts";
import { InventoryContract } from "#dimos/agents/shared/protocol/inventory/contract.ts";

/** Runtime inventory shared by terminal stream discovery and the /runtime view. */
export const serveInventory: ServeObject = (host) =>
  host.serve(InventoryContract, (_input, _principal, self) => ({
    variant: "viewer",
    inventory: self.get(GatewayStore, {}).inventory,
  }));
