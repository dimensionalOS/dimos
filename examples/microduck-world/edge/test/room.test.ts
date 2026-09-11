import { env } from "cloudflare:workers";
import { expect, it } from "vitest";
import {
  clientFrame,
  MAX_PENDING_DOWNLOADS,
  pack,
} from "../../shared/publicWire.ts";

function message(socket: WebSocket): Promise<string | ArrayBuffer> {
  return new Promise((resolve) =>
    socket.addEventListener("message", (e) => resolve(e.data), { once: true })
  );
}
async function room() {
  const room = env.MATCH.getByName(crypto.randomUUID());
  const response = await room.fetch("https://test/host", {
    headers: { upgrade: "websocket" },
  });
  const host = response.webSocket!;
  host.binaryType = "arraybuffer";
  host.accept();
  return { room, host };
}
async function viewer(room: ReturnType<typeof env.MATCH.getByName>) {
  const response = await room.fetch("https://test/viewer?ticket=example", {
    headers: {
      upgrade: "websocket",
      "x-user-id": "100",
      "x-user-login": "player",
      "x-session-expires": String(Date.now() + 60000),
    },
  });
  expect(response.status).toBe(101);
  const socket = response.webSocket!;
  socket.binaryType = "arraybuffer";
  socket.accept();
  return socket;
}
it("routes opaque DimOS frames, returns receive credit and disconnects slow viewers", async () => {
  const { room: match, host } = await room();
  const opened = message(host);
  const client = await viewer(match);
  const info = JSON.parse(await opened as string);
  expect(info.user).toEqual({ id: "100", login: "player" });
  const inbound = message(host);
  client.send(clientFrame(0, new Uint8Array([1, 2, 3])));
  expect(typeof await inbound).not.toBe("string");
  host.send(JSON.stringify({ t: "received", id: info.id }));
  const frame = message(client);
  host.send(pack({ to: [info.id], kind: 1 }, new Uint8Array([4, 5, 6])));
  expect([...new Uint8Array(await frame as ArrayBuffer).slice(5)]).toEqual([
    4,
    5,
    6,
  ]);
  client.send(JSON.stringify({ t: "ack", n: 1 }));
  const closed = new Promise((resolve) =>
    client.addEventListener("close", resolve, { once: true })
  );
  for (let i = 0; i < MAX_PENDING_DOWNLOADS + 2; i++) {
    host.send(pack({ to: [info.id], kind: 1 }, new Uint8Array([7])));
  }
  await closed;
  host.close();
});
it("allows a healthy cockpit burst while Internet acknowledgments are in flight", async () => {
  const { room: match, host } = await room();
  const opened = message(host);
  const client = await viewer(match);
  const info = JSON.parse(await opened as string);
  try {
    for (let batch = 0; batch < 2; batch++) {
      const sequences: number[] = [];
      const burst = new Promise<void>((resolve, reject) => {
        const onClose = () =>
          reject(new Error("Healthy client was disconnected"));
        const onMessage = (event: MessageEvent) => {
          sequences.push(new DataView(event.data as ArrayBuffer).getUint32(1));
          if (sequences.length === 40) {
            client.removeEventListener("message", onMessage);
            client.removeEventListener("close", onClose);
            resolve();
          }
        };
        client.addEventListener("message", onMessage);
        client.addEventListener("close", onClose, { once: true });
      });
      // About half a second of the multi-channel cockpit stream, before ACKs
      // make their return trip. The old 16-frame cap rejected this workload.
      for (let i = 0; i < 40; i++) {
        host.send(pack({ to: [info.id], kind: 1 }, new Uint8Array(1024)));
      }
      await burst;
      for (const n of sequences) client.send(JSON.stringify({ t: "ack", n }));
    }
  } finally {
    client.close();
    host.close();
  }
});

it("fails closed when the origin disconnects and does not retain old controls", async () => {
  const { room: match, host } = await room();
  const opened = message(host);
  const client = await viewer(match);
  await opened;
  const closed = new Promise((resolve) =>
    client.addEventListener("close", resolve, { once: true })
  );
  host.close();
  await closed;
  const response = await match.http(null, "/api/lobby");
  expect(response.status).toBe(503);
});
