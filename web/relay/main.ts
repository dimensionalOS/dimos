// Relay CLI. Run from web/:  deno task dev  (or --port/--host/--cockpit-dir).
// Prints a single JSON ready line on stdout for parent processes to parse;
// everything else logs to stderr-adjacent console lines prefixed [relay].
import { parseArgs } from "@std/cli";
import { PROTOCOL_VERSION } from "@dimos/shared";
import { loadAuthFile } from "./auth.ts";
import { loadRtcFile } from "./cloudflare.ts";
import { CERT_KEY_PAIR_ERROR, startRelay } from "./server.ts";
import { withoutFlag } from "./shards.ts";

const args = parseArgs(Deno.args, {
  string: [
    "host",
    "cockpit-dir",
    "sdk-dir",
    "serve-dir",
    "cert",
    "key",
    "auth-file",
    "rtc-file",
    "congestion",
    "shards",
  ],
  // Non-loopback binds without --cert, --key and --auth-file need this
  // explicit acknowledgment (see RelayOptions.unsafeNonLoopback).
  boolean: ["unsafe-non-loopback"],
  default: { port: 7780, host: "127.0.0.1" },
});

if ((args.cert === undefined) !== (args.key === undefined)) {
  throw new Error(CERT_KEY_PAIR_ERROR);
}
const congestion = args.congestion;
if (congestion !== undefined && congestion !== "throughput" && congestion !== "low-latency") {
  throw new Error(
    `--congestion must be throughput (Cubic) or low-latency (BBR), got ${congestion}`,
  );
}
const shards = args.shards === undefined ? 1 : Number(args.shards);
if (!Number.isInteger(shards) || shards < 1) {
  throw new Error(`--shards must be a whole number of relay processes, got ${args.shards}`);
}
// The other shards run this same command line without --shards and --port
// (they bind ephemeral ports) and may read what this process was given.
const shardChildArgs = withoutFlag(withoutFlag(Deno.args, "shards"), "port");
const shardReadScopes =
  (["cockpit-dir", "sdk-dir", "serve-dir", "cert", "key", "auth-file", "rtc-file"] as const)
    .map((name) => args[name])
    .filter((value): value is string => value !== undefined);
const host = args.host as string;
const tls = args.cert !== undefined;
if (host !== "127.0.0.1" && host !== "localhost" && !tls) {
  // serverCertificateHashes only works from secure contexts; http://<lan-ip>
  // pages are not one. A real certificate (--cert/--key) makes https://<host>
  // one, and the browser then verifies it instead of pinning a hash.
  console.log(
    `[relay] warning: binding ${host} - browsers will not treat http://${host} as a ` +
      "secure context, so WebTransport will be unavailable there; this is only useful " +
      "behind your own TLS setup",
  );
}

const relay = await startRelay({
  port: Number(args.port),
  host,
  cockpitDir: args["cockpit-dir"],
  sdkDir: args["sdk-dir"],
  serveDir: args["serve-dir"],
  unsafeNonLoopback: args["unsafe-non-loopback"],
  cert: args.cert === undefined ? undefined : await Deno.readTextFile(args.cert),
  key: args.key === undefined ? undefined : await Deno.readTextFile(args.key),
  auth: args["auth-file"] === undefined ? undefined : await loadAuthFile(args["auth-file"]),
  rtc: args["rtc-file"] === undefined ? undefined : await loadRtcFile(args["rtc-file"]),
  congestionControl: congestion,
  shards,
  shardChildArgs,
  shardReadScopes,
}).catch((e: unknown) => {
  // The unhandled-rejection guard must not turn a failed start into a
  // silent exit 0 (a shard that would not spawn, a port in use).
  console.error(`[relay] failed to start: ${e instanceof Error ? e.message : String(e)}`);
  Deno.exit(1);
});

console.log(JSON.stringify({
  event: "ready",
  httpPort: relay.httpPort,
  wtUrl: relay.wtUrl,
  certHash: relay.certHash,
  v: PROTOCOL_VERSION,
  shards: relay.shards,
}));
if (relay.shards > 1) {
  console.log(
    `[relay] shards: ${relay.shards}, this process is shard 0 (/api/info?robot=<id> picks one)`,
  );
}
const pageHost = host === "0.0.0.0" ? "127.0.0.1" : host;
const pageBase = `${tls ? "https" : "http"}://${pageHost}:${relay.httpPort}/`;
if (args["serve-dir"] !== undefined) {
  console.log(`[relay] serving ${args["serve-dir"]}: ${pageBase}`);
} else if (args["cockpit-dir"] !== undefined) {
  console.log(`[relay] cockpit: ${pageBase}`);
} else {
  console.log("[relay] no cockpit dist configured; serving /api only");
}
if (args["sdk-dir"] !== undefined) {
  console.log(`[relay] sdk: ${pageBase}sdk.js`);
}
if (args["rtc-file"] !== undefined) {
  console.log("[relay] WebRTC video through the Cloudflare Realtime SFU: on");
}
console.log(`[relay] congestion control: ${congestion === "throughput" ? "Cubic" : "BBR"}`);
for (const signal of ["SIGINT", "SIGTERM"] as const) {
  try {
    Deno.addSignalListener(signal, async () => {
      console.log(`[relay] ${signal}, shutting down`);
      await relay.shutdown();
      Deno.exit(0);
    });
  } catch {
    // not supported on this platform (e.g. SIGTERM on Windows)
  }
}
