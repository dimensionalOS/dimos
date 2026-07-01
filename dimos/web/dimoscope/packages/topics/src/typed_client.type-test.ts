// Compile-only type test for the generic client (no runtime; picked up by `deno task check`, NOT by
// `deno test` — the filename doesn't match Deno's test globs). Proves connect<TMap>() gives typed +
// autocompleting topic handles, keeps the explicit `topic<T>()` form, and leaves an untyped
// `connect()` behaving exactly as before. The `@ts-expect-error` lines fail the build if the types
// stop constraining (a positives-only test would pass even if everything were `any`).
import { connect } from "./client.ts";

type Fixture = { "/odom": { x: number }; "/map": { w: number } };

// deno-lint-ignore no-unused-vars
async function _typed() {
  const c = await connect<Fixture>({ url: "x" });

  // ① known key → autocompletes name, infers the mapped message type
  const odom = c.topic("/odom");
  const _pose: { x: number } | undefined = odom.getLatest();
  // @ts-expect-error — the mapped type is {x:number}, not {y:number}
  const _wrong: { y: number } | undefined = c.topic("/odom").getLatest();

  // ② runtime-discovered name → still allowed, Topic<unknown>
  const _u: unknown = c.topic("/not_in_map").getLatest();

  // explicit message-type form preserved
  const _z: { z: number } | undefined = c.topic<{ z: number }>("/z").getLatest();
}

// deno-lint-ignore no-unused-vars
async function _untyped() {
  const c = await connect({ url: "x" }); // no TMap → behaves as before
  const _u: unknown = c.topic("/anything").getLatest(); // Topic<unknown>
  const _t: { a: number } | undefined = c.topic<{ a: number }>("/x").getLatest();
}
