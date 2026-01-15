#!/usr/bin/env -S deno run --allow-net --allow-read --unstable-net

// LCM to WebSocket Bridge for Robot Control
// Forwards robot pose to browser, receives twist commands from browser

import { LCM } from "jsr:@dimos/lcm";
import { decodePacket, geometry_msgs } from "jsr:@dimos/msgs";

const PORT = 8443;
const clients = new Set<WebSocket>();

// Auto-generate self-signed certificates if they don't exist
async function ensureCerts(): Promise<{ cert: string; key: string }> {
  const certPath = "./certs/cert.pem";
  const keyPath = "./certs/key.pem";

  try {
    const cert = await Deno.readTextFile(certPath);
    const key = await Deno.readTextFile(keyPath);
    return { cert, key };
  } catch {
    console.log("Generating self-signed certificates...");
    await Deno.mkdir("./certs", { recursive: true });
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
    console.log("Certificates generated in ./certs/");
    return {
      cert: await Deno.readTextFile(certPath),
      key: await Deno.readTextFile(keyPath),
    };
  }
}

const { cert, key } = await ensureCerts();

Deno.serve({ port: PORT, cert, key }, async (req) => {
  const url = new URL(req.url);

  if (req.headers.get("upgrade") === "websocket") {
    const { socket, response } = Deno.upgradeWebSocket(req);
    socket.onopen = () => { console.log("Client connected"); clients.add(socket); };
    socket.onclose = () => { console.log("Client disconnected"); clients.delete(socket); };
    socket.onerror = () => clients.delete(socket);

    // Forward binary LCM packets from browser directly to UDP
    socket.binaryType = "arraybuffer";
    let msgCount = 0;
    socket.onmessage = async (event) => {
      msgCount++;
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

const lcm = new LCM();
await lcm.start();

// Subscribe to pose and just log to show how server can decode messages for itself
// lcm.subscribe("/odom", geometry_msgs.PoseStamped, (msg) => {
//   const pos = msg.data.pose.position;
//   const ori = msg.data.pose.orientation;
//   console.log(`[pose] x=${pos.x.toFixed(2)} y=${pos.y.toFixed(2)} z=${pos.z.toFixed(2)}`);
// });

// Forward all raw packets to browser (we are decoding LCM directly in the browser)
lcm.subscribePacket((packet) => {
  for (const client of clients) {
    if (client.readyState === WebSocket.OPEN) {
      client.send(packet);
    }
  }
});

await lcm.run();
