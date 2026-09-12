// Relay CLI. Run from web/:  deno task dev  (or --port/--host/--cockpit-dir).
// Prints a single JSON ready line on stdout for parent processes to parse;
// everything else logs to stderr-adjacent console lines prefixed [relay].
import { parseArgs } from "@std/cli";
import { PROTOCOL_VERSION } from "@dimos/shared";
import { CERT_KEY_PAIR_ERROR, startRelay } from "./server.ts";

const args = parseArgs(Deno.args, {
  string: ["host", "cockpit-dir", "sdk-dir", "serve-dir", "cert", "key"],
  // Non-loopback binds need this explicit acknowledgment: the local relay
  // trusts every origin that can reach it (see RelayOptions.unsafeNonLoopback).
  boolean: ["unsafe-non-loopback"],
  default: { port: 7780, host: "127.0.0.1" },
});

if ((args.cert === undefined) !== (args.key === undefined)) {
  throw new Error(CERT_KEY_PAIR_ERROR);
}
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
});

console.log(JSON.stringify({
  event: "ready",
  httpPort: relay.httpPort,
  wtUrl: relay.wtUrl,
  certHash: relay.certHash,
  v: PROTOCOL_VERSION,
}));
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
