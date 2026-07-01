#!/usr/bin/env -S deno run -A
// gen-types — emit the typed `DimosTopics` + `DimosCommands` maps from LIVE gateway discovery. The thin
// runnable wrapper around the pure codegen in genTypes.ts: it connects the SDK (default `ws()` transport),
// lets discovery settle, then feeds the discovered topic/type pairs + advertised RPC commands through
// `generateTypes` / `generateCommandTypes`. The app consumes the output as app/src/dimos.topics.gen.ts.
// Imports ../src/client.ts directly (not the index barrel) so the DOM-only media plane never enters the
// graph — this is headless Deno.
//
//   deno task gen-types --out app/src/dimos.topics.gen.ts   # write the app's map (needs a gateway + publisher)
//   deno task gen-types                                     # print to stdout
//   deno task gen-types --url ws://host:8080 --dur 3000
import { getTypeNames } from "@dimos/msgs";
import { createDimosClient, ws } from "../src/client.ts";
import { generateCommandTypes, generateTypes } from "./genTypes.ts";

const arg = (name: string, def?: string): string | undefined => {
  const i = Deno.args.indexOf(`--${name}`);
  return i >= 0 && i + 1 < Deno.args.length ? Deno.args[i + 1] : def;
};

const url = arg("url", "ws://localhost:8080")!;
const out = arg("out");
const dur = Number(arg("dur", "2500")); // discovery-settle window

// The set of message types @dimos/msgs can decode → anything else is user-defined → `unknown`.
const known = new Set<string>(getTypeNames());

const client = createDimosClient({ transport: ws({ reconnect: false }) });
await client.connect(url);
await new Promise((r) => setTimeout(r, dur)); // let discovery settle
const topics = client.listTopics().sort((a, b) => a.topic.localeCompare(b.topic));
const commands = client.commands; // advertised on the gateway hello (empty on read-only transports)
client.close();

if (!topics.length) {
  console.error(`gen-types: no topics discovered at ${url} — is a gateway + publisher running?`);
}
const src = generateTypes(topics, known) + "\n" + generateCommandTypes(commands);
if (out) {
  await Deno.writeTextFile(out, src);
  console.error(`gen-types: wrote ${topics.length} topics + ${commands.length} commands → ${out}`);
} else {
  console.log(src);
}
