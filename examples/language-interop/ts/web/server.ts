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

    // Handle twist commands from browser
    socket.onmessage = async (event) => {
      try {
        const cmd = JSON.parse(event.data);
        if (cmd.type === "twist") {
          const twist = new geometry_msgs.Twist({
            linear: new geometry_msgs.Vector3({ x: cmd.linear, y: 0, z: 0 }),
            angular: new geometry_msgs.Vector3({ x: 0, y: 0, z: cmd.angular }),
          });
          await lcm.publish("/cmd_vel", twist);
          console.log(`[cmd] linear=${cmd.linear.toFixed(2)} angular=${cmd.angular.toFixed(2)}`);
        }
      } catch (e) {
        console.error("Command error:", e);
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

// Subscribe to pose and forward to WebSocket clients
lcm.subscribe("/pose", geometry_msgs.PoseStamped, (msg) => {
  const pos = msg.data.pose.position;
  const ori = msg.data.pose.orientation;
  console.log(`[pose] x=${pos.x.toFixed(2)} y=${pos.y.toFixed(2)} z=${pos.z.toFixed(2)}`);
});

// Forward all raw packets to browser
lcm.subscribePacket((packet) => {
  for (const client of clients) {
    if (client.readyState === WebSocket.OPEN) {
      client.send(packet);
    }
  }
});

await lcm.run();
