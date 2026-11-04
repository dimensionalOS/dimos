import type { EpochMs } from "#dimos/agents/shared/protocol/time.ts";
import type { Inventory } from "#dimos/agents/shared/protocol/runtime/inventory.ts";
import { Type } from "typebox";
import { carried } from "#dimos/agents/shared/protocol/schema.ts";
import { hostNamed, remote } from "@earendil-works/chord";

/** The whole running system as HarnessRuntime last described it; null before the first inventory since start. */
export type InventoryView = { readonly inventory: Inventory; readonly at: EpochMs; readonly current: boolean } | null;

/** The one member of the held inventory. */
export const InventoryState = { inventory: carried<InventoryView>() } as const;

/**
 * The runtime inventory, including the relay endpoint used by the terminal. Kept separate from the gateway state
 * because RPC docstrings make it the largest gateway value.
 */
export const InventoryContract = remote("dimos.inventory", {
  version: 1,
  input: Type.Object({}),
  host: hostNamed("gateway"),
  variants: { viewer: { state: InventoryState } },
});
