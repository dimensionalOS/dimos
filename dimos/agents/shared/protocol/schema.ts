import type { TUnsafe } from "typebox";
import { Type } from "typebox";

/**
 * A state member or method result the gateway produces. Its one definition is the TypeScript type its owner already
 * declares (pi-durable or shared/protocol/); clients trust the gateway, so the wire carries no second schema to keep in
 * step. Method arguments, which clients produce, always get a full schema instead.
 */
export function carried<T>(): TUnsafe<T> {
  return Type.Unsafe<T>(Type.Unknown());
}

/** A field that may be null, for method argument schemas. */
export function nullable<S extends Parameters<typeof Type.Union>[0][number]>(schema: S) {
  return Type.Union([schema, Type.Null()]);
}
