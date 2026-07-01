// Compile-only type test for the generic client (no runtime; picked up by `deno task check`, NOT by
// `deno test` — the filename doesn't match Deno's test globs). Proves connect<TMap>() gives typed +
// autocompleting topic handles for known names, yields Topic<unknown> for runtime names, and leaves
// an untyped `connect()` behaving as before. The `@ts-expect-error` lines fail the build if the types
// stop constraining (a positives-only test would pass even if everything were `any`). Functions are
// `_`-prefixed so deno-lint's no-unused-vars leaves them alone.
import { connect } from "./client.ts";

type Fixture = { "/odom": { x: number }; "/map": { w: number } };

async function _typed() {
  const c = await connect<Fixture>({ url: "x" });

  // known key → autocompletes name, infers the mapped message type
  const odom = c.topic("/odom");
  const _pose: { x: number } | undefined = odom.getLatest();
  // @ts-expect-error — the mapped type is {x:number}, not {y:number}
  const _wrong: { y: number } | undefined = c.topic("/odom").getLatest();

  // runtime-discovered name → still allowed, Topic<unknown>
  const _u: unknown = c.topic("/not_in_map").getLatest();

  // no hand-typed escape hatch: the explicit message-type form is gone (K must be a string key)
  // @ts-expect-error — `topic<Msg>()` removed; the type param is the name key, not the message type
  c.topic<{ z: number }>("/z");
}

async function _untyped() {
  const c = await connect({ url: "x" }); // no TMap → every topic is Topic<unknown>
  const _u: unknown = c.topic("/anything").getLatest();
}
