import type { Fact, FactKey, Facts } from "#dimos/agents/shared/protocol/runtime/context.ts";
import type { WireDecoder } from "#dimos/agents/harness/dimos/wire/base.ts";
import { InventorySchema } from "#dimos/agents/shared/protocol/runtime/inventory.ts";
import { Type } from "typebox";
import { WireError } from "#dimos/agents/harness/dimos/wire/base.ts";
import { wireDecoder } from "#dimos/agents/harness/dimos/wire/decoder.ts";

export const inventoryDecoder = wireDecoder("inventory result", InventorySchema);

const compactInventoryDecoder = wireDecoder(
  "compact inventory result",
  Type.Object({
    facts: Type.Array(
      Type.Object({
        section: Type.String(),
        key: Type.Unsafe<FactKey>(Type.String({ pattern: "^[^:]+:.+$" })),
        line: Type.String(),
      }),
    ),
  }),
);

/** `inventory compact=true`: the fact lines the runtime context compares and renders, keyed; a key sent twice is a WireError. */
export const inventoryFactsDecoder: WireDecoder<Facts> = {
  name: compactInventoryDecoder.name,
  decode(json: unknown): Facts {
    const { facts } = compactInventoryDecoder.decode(json);
    const keyed: { [key: FactKey]: Fact } = {};
    for (const { section, key, line } of facts) {
      if (key in keyed)
        throw new WireError(compactInventoryDecoder.name, [`duplicate fact key ${JSON.stringify(key)}`]);
      keyed[key] = { section, line };
    }
    return keyed;
  },
};
