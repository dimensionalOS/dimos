#!/usr/bin/env -S deno run --allow-net --allow-read --allow-run --allow-write --unstable-net

// WebSocket to LCM Bridge for Quest VR Teleop
// Forwards controller data from browser to LCM

import { LCM } from "jsr:@dimos/lcm";
import { dirname, fromFileUrl, join } from "jsr:@std/path";

const PORT = 8443;

// Resolve paths relative to script location
const scriptDir = dirname(fromFileUrl(import.meta.url));
const certsDir = join(scriptDir, "../../../../assets/teleop_certs");
const certPath = join(certsDir, "cert.pem");
const keyPath = join(certsDir, "key.pem");

// Auto-generate self-signed certificates if they don't exist
async function ensureCerts(): Promise<{ cert: string; key: string }> {
  try {
    const cert = await Deno.readTextFile(certPath);
    const key = await Deno.readTextFile(keyPath);
    return { cert, key };
  } catch {
    console.log("Generating self-signed certificates...");
    await Deno.mkdir(certsDir, { recursive: true });
    const cmd = new Deno.Command("openssl", {
      args: [
        "req", "-x509", "-newkey", "rsa:2048",
        "-keyout", keyPath, "-out", certPath,
        "-days", "365", "-nodes", "-subj", "/CN=localhost"
      ],
    });
    const { code } = await cmd.output();
    if (code !== 0) {
      throw new Error("Failed to generate certificates. Is openssl installed?");
    }
    console.log("Certificates generated in assets/teleop_certs/");
    return {
      cert: await Deno.readTextFile(certPath),
      key: await Deno.readTextFile(keyPath),
    };
  }
}

const { cert, key } = await ensureCerts();

const lcm = new LCM();
await lcm.start();

Deno.serve({ port: PORT, cert, key }, async (req) => {
  const url = new URL(req.url);

  if (req.headers.get("upgrade") === "websocket") {
    const { socket, response } = Deno.upgradeWebSocket(req);
    socket.onopen = () => console.log("Client connected");
    socket.onclose = () => console.log("Client disconnected");

    // Forward binary LCM packets from browser directly to UDP
    socket.binaryType = "arraybuffer";
    socket.onmessage = async (event) => {
      if (event.data instanceof ArrayBuffer) {
        const packet = new Uint8Array(event.data);
        try {
          await lcm.publishPacket(packet);
        } catch (e) {
          console.error("Forward error:", e);
        }
      }
    };

    return response;
  }

  if (url.pathname === "/" || url.pathname === "/index.html") {
    const html = await Deno.readTextFile(new URL("./static/index.html", import.meta.url));
    return new Response(html, { headers: { "content-type": "text/html" } });
  }

  return new Response("Not found", { status: 404 });
});

console.log(`Server: https://localhost:${PORT}`);

await lcm.run();
