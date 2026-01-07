#!/usr/bin/env -S deno run --allow-net --allow-read --unstable-net

// LCM to WebSocket Bridge for Robot Control
// Forwards robot pose to browser, receives twist commands from browser

import { LCM } from "jsr:@dimos/lcm";
import { decodePacket, geometry_msgs } from "jsr:@dimos/msgs";

const PORT = 8080;
const clients = new Set<WebSocket>();

Deno.serve({ port: PORT }, async (req) => {
  const url = new URL(req.url);

  if (req.headers.get("upgrade") === "websocket") {
    const { socket, response } = Deno.upgradeWebSocket(req);
    socket.onopen = () => { console.log("Client connected"); clients.add(socket); };
    socket.onclose = () => { console.log("Client disconnected"); clients.delete(socket); };
    socket.onerror = () => clients.delete(socket);

    // Forward binary LCM packets from browser directly to UDP
    socket.binaryType = "arraybuffer";
    socket.onmessage = async (event) => {
      if (event.data instanceof ArrayBuffer) {
        const packet = new Uint8Array(event.data);
        try {
          // we don't need to decode, just showing we can
          const { channel, data } = decodePacket(packet);
          console.log(`[ws->lcm] ${channel}`, data);
          await lcm.publishPacket(packet);
        } catch (e) {
          console.error("Forward error:", e);
        }
      }
    };

    return response;
  }

  if (url.pathname === "/" || url.pathname === "/index.html") {
    const html = await Deno.readTextFile(new URL("./index.html", import.meta.url));
    return new Response(html, { headers: { "content-type": "text/html" } });
  }

  return new Response("Not found", { status: 404 });
});

console.log(`Server: http://localhost:${PORT}`);

const lcm = new LCM();
await lcm.start();

// Subscribe to pose and just log to show how server can decode messages for itself
lcm.subscribe("/odom", geometry_msgs.PoseStamped, (msg) => {
  const pos = msg.data.pose.position;
  const ori = msg.data.pose.orientation;
  console.log(`[pose] x=${pos.x.toFixed(2)} y=${pos.y.toFixed(2)} z=${pos.z.toFixed(2)}`);
});

// Forward all raw packets to browser (we are decoding LCM directly in the browser)
lcm.subscribePacket((packet) => {
  for (const client of clients) {
    if (client.readyState === WebSocket.OPEN) {
      client.send(packet);
    }
  }
});

await lcm.run();
