import type { Cell, Self } from "@earendil-works/chord";
import type { Live } from "../../../../shared/protocol/status.ts";
import type { Principal } from "../../../auth/principal.ts";
import type { Static, TSchema } from "typebox";

/**
 * A cell that holds `source.value`, replaced on every change. A Live hands its current value to a new subscriber, so a
 * change between `init` and the listener's start is not lost. `replace` diffs structurally when the new value shares
 * containers with the old, so subscribers get ops; `diff` makes it diff every value, so a string that grows goes out
 * as an append.
 */
export function feed<Schema extends TSchema>(
  self: Self<Principal>,
  name: string,
  schema: Schema,
  source: Live<Static<Schema>>,
  options: { readonly diff?: boolean } = {},
): Cell<Static<Schema>> {
  const cell = self.state(name, schema, { init: () => source.value });
  self.on(name, source, (value, ctx) => cell.replace(ctx, value, options));
  return cell;
}
