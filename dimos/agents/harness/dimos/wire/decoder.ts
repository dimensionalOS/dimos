import type { Static, TSchema } from "typebox";
import type { WireDecoder } from "#dimos/agents/harness/dimos/wire/base.ts";
import { Value } from "typebox/value";
import { WireError } from "#dimos/agents/harness/dimos/wire/base.ts";

/** A copy of the JSON with Python's field names and only the fields the schema names, once it matches the schema. */
export function wireDecoder<S extends TSchema>(name: string, schema: S): WireDecoder<Static<S>> {
  return {
    name,
    decode(json: unknown): Static<S> {
      const named = Value.Clean(schema, Value.Clone(json));
      if (!Value.Check(schema, named)) {
        throw new WireError(
          name,
          Value.Errors(schema, named).map((error) => `${error.instancePath || "/"}: ${error.message}`),
        );
      }
      return named;
    },
  };
}
