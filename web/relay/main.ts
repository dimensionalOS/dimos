// Relay CLI. Run from web/:  deno task dev  (or --port/--host/--cockpit-dir).
// Prints a single JSON ready line on stdout for parent processes to parse;
// everything else logs to stderr-adjacent console lines prefixed [relay].
import { parseArgs } from "@std/cli";
import { PROTOCOL_VERSION } from "@dimos/shared";
import { startRelay } from "./server.ts";

const args = parseArgs(Deno.args, {
  string: ["host", "cockpit-dir", "sdk-dir", "serve-dir"],
  // Non-loopback binds need this explicit acknowledgment: the local relay
  // trusts every origin that can reach it (see RelayOptions.unsafeNonLoopback).
  boolean: ["unsafe-non-loopback"],
  default: { port: 7780, host: "127.0.0.1" },
});

const host = args.host as string;
if (host !== "127.0.0.1" && host !== "localhost") {
  // serverCertificateHashes only works from secure contexts; http://<lan-ip>
  // pages are not one. Remote access needs the cloud relay + real TLS (T12).
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
});

console.log(JSON.stringify({
  event: "ready",
  httpPort: relay.httpPort,
  wtUrl: relay.wtUrl,
  certHash: relay.certHash,
  v: PROTOCOL_VERSION,
}));
const pageHost = host === "0.0.0.0" ? "127.0.0.1" : host;
if (args["serve-dir"] !== undefined) {
  console.log(`[relay] serving ${args["serve-dir"]}: http://${pageHost}:${relay.httpPort}/`);
} else if (args["cockpit-dir"] !== undefined) {
  console.log(`[relay] cockpit: http://${pageHost}:${relay.httpPort}/`);
} else {
  console.log("[relay] no cockpit dist configured; serving /api only");
}
if (args["sdk-dir"] !== undefined) {
  console.log(`[relay] sdk: http://${pageHost}:${relay.httpPort}/sdk.js`);
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
