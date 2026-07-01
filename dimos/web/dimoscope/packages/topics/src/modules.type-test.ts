// Compile-only type test for the typed RPC `modules` surface (no runtime; picked up by `deno task check`,
// NOT by `deno test` — the filename doesn't match Deno's test globs). Companion to
// typed_client.type-test.ts (which covers the topics/read side). Proves the second generic bites: a real
// `DimosCommands`-shaped map makes `client.modules.<Target>.<method>` autocompleted + typo-checked and
// lifted to a callable, while an untyped client stays permissive (the `client.call` escape hatch). The
// `@ts-expect-error` lines fail the build if the types stop constraining. Functions are `_`-prefixed so
// deno-lint's no-unused-vars leaves them alone.
import { createDimosClient } from "./client.ts";

// Mirrors what `generateCommandTypes` (packages/topics/scripts/genTypes.ts) emits: target → method → RpcCall descriptor.
type Commands = {
  ScopeNav: {
    start: { args: unknown[]; ret: unknown };
    stop: { args: unknown[]; ret: unknown };
  };
};

async function _typedModules() {
  const c = createDimosClient<Record<never, never>, Commands>();
  await c.connect("x");

  // known target + method → lifted to a callable returning a Promise
  const _ret: Promise<unknown> = c.modules.ScopeNav.start();
  await c.modules.ScopeNav.stop();

  // @ts-expect-error — unknown method on a known target
  c.modules.ScopeNav.fly();
  // @ts-expect-error — unknown target
  c.modules.NotAModule.start();
}

async function _untypedModules() {
  const c = createDimosClient(); // no TCmds → permissive ModuleMap (escape hatch preserved)
  await c.connect("x");
  const _any: Promise<unknown> = c.modules.Anything.anything();
}
