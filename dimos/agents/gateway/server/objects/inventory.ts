import type { ServeObject } from "./base.ts";
import { GatewayStore } from "./store.ts";
import { InventoryContract } from "../../../shared/protocol/inventory/contract.ts";

/** Runtime inventory shared by terminal stream discovery and the /runtime view. */
export const serveInventory: ServeObject = (host) =>
  host.serve(InventoryContract, (_input, _principal, self) => ({
    variant: "viewer",
    inventory: self.get(GatewayStore, {}).inventory,
  }));
