/** A fact's identity, `<kind>:<name>`, e.g. `rpc:GO2Connection.move` or `job:patrol`. Facts are compared by key. */
export type FactKey = `${string}:${string}`;

/** One line of runtime knowledge, as HarnessRuntime's compact inventory or the job registry states it. */
export type Fact = {
  /** The heading the line is grouped under, e.g. "RPCs". */
  readonly section: string;
  readonly line: string;
};

export type Facts = { readonly [key: FactKey]: Fact };

/** What an input tells the model about the runtime. Stored in the input's own entry, so it commits with the message. */
export type RuntimeChange =
  | {
      /** Restates every fact; the model should forget earlier runtime messages. */
      readonly kind: "full";
      readonly facts: Facts;
      readonly summary: string;
    }
  | {
      readonly kind: "delta";
      readonly set: Facts;
      /** The facts that disappeared, as they were last stated, so the message can show their lines by section. */
      readonly removed: Facts;
      /** e.g. "DimOS reachable · Modules +2 · RPCs ~1 −3". */
      readonly summary: string;
    };
