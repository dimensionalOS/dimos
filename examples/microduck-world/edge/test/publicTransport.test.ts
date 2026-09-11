import { env } from "cloudflare:workers";
import { afterEach, expect, it, vi } from "vitest";
import { PublicTransport } from "../../web/src/publicTransport.ts";
import { pack, unpack } from "../../shared/publicWire.ts";

afterEach(() => vi.unstubAllGlobals());

it("carries SDK streams through the real edge relay and acknowledges received frames", async () => {
  const match = env.MATCH.getByName(crypto.randomUUID());
  const hostResponse = await match.fetch("https://test/host", {
    headers: { upgrade: "websocket" },
  });
  const host = hostResponse.webSocket!;
  host.binaryType = "arraybuffer";
  host.accept();
  const received: (string | ArrayBuffer)[] = [];
  let clientId = "";
  host.addEventListener("message", (event) => {
    received.push(event.data);
    if (typeof event.data === "string") {
      const message = JSON.parse(event.data);
      if (message.t === "open") clientId = message.id;
    } else {
      const { header } = unpack(new Uint8Array(event.data));
      host.send(JSON.stringify({ t: "received", id: header.to[0] }));
    }
  });

  // Workers returns an accepted socket instead of a browser constructor. This
  // shim changes only that API surface; both websocket endpoints and the DO
  // credit/account routing below run in the actual Workers test runtime.
  class BrowserSocket extends EventTarget {
    static OPEN = 1;
    readyState = 0;
    bufferedAmount = 0;
    binaryType = "arraybuffer";
    socket?: WebSocket;
    constructor() {
      super();
      void this.open();
    }
    async open() {
      const response = await match.fetch("https://test/viewer?ticket=example", {
        headers: {
          upgrade: "websocket",
          "x-user-id": "100",
          "x-user-login": "player",
          "x-session-expires": String(Date.now() + 60000),
        },
      });
      this.socket = response.webSocket!;
      this.socket.binaryType = "arraybuffer";
      this.socket.accept();
      this.socket.addEventListener(
        "message",
        (e) =>
          this.dispatchEvent(new MessageEvent("message", { data: e.data })),
      );
      this.socket.addEventListener("close", () => {
        this.readyState = 3;
        this.dispatchEvent(new CloseEvent("close"));
      });
      this.readyState = 1;
      this.dispatchEvent(new Event("open"));
    }
    send(data: string | Uint8Array) {
      this.socket!.send(data);
    }
    close(code?: number, reason?: string) {
      this.socket?.close(code, reason);
    }
  }
  vi.stubGlobal("WebSocket", BrowserSocket);
  const transport = new PublicTransport({
    wtUrl: "wss://test/connect",
    certHash: "",
    v: 5,
  });
  try {
    const control = await transport.createBidirectionalStream();
    const writer = control.writable.getWriter();
    await writer.write(new Uint8Array([1, 2, 3]));
    await vi.waitFor(() =>
      expect(received.some((m) => typeof m !== "string")).toBe(true)
    );
    const inbound = unpack(
      new Uint8Array(
        received.find((m): m is ArrayBuffer => typeof m !== "string")!,
      ),
    );
    expect(inbound.header.kind).toBe(0);
    expect([...inbound.payload]).toEqual([1, 2, 3]);

    const reader = control.readable.getReader();
    for (let n = 0; n < 100; n++) {
      host.send(pack({ to: [clientId], kind: 0 }, new Uint8Array([n])));
      expect([...(await reader.read()).value!]).toEqual([n]);
    }
    const streams = transport.incomingUnidirectionalStreams.getReader();
    host.send(pack({ to: [clientId], kind: 1 }, new Uint8Array([7, 8])));
    const dataReader = (await streams.read()).value!.getReader();
    expect([...(await dataReader.read()).value!]).toEqual([7, 8]);
    expect((await dataReader.read()).done).toBe(true);

    const datagrams = transport.datagrams.writable.getWriter();
    await datagrams.write(new Uint8Array([9]));
    await vi.waitFor(() =>
      expect(received.filter((m) => typeof m !== "string")).toHaveLength(2)
    );
    const last = received.filter((m): m is ArrayBuffer => typeof m !== "string")
      .at(-1)!;
    expect(unpack(new Uint8Array(last)).header.kind).toBe(2);
    transport.close();
    expect(await transport.closed).toEqual({ reason: "Session ended" });
  } finally {
    transport.close();
    host.close();
  }
});

it("reports replacement once without treating ordinary disconnects as ownership changes", async () => {
  let socket: EventTarget;
  class Socket extends EventTarget {
    binaryType = "arraybuffer";
    constructor() { super(); socket = this; queueMicrotask(() => this.dispatchEvent(new Event("open"))); }
    close() {}
  }
  vi.stubGlobal("WebSocket", Socket);
  const replaced = vi.fn();
  const transport = new PublicTransport({ wtUrl: "wss://test", certHash: "", v: 5 }, replaced);
  await transport.ready;
  socket!.dispatchEvent(new CloseEvent("close", { reason: "Replaced by your new connection" }));
  expect(replaced).toHaveBeenCalledTimes(1);
  expect((await transport.closed).reason).toBe("Replaced by your new connection");
  const ordinary = new PublicTransport({ wtUrl: "wss://test", certHash: "", v: 5 }, replaced);
  await ordinary.ready;
  socket!.dispatchEvent(new CloseEvent("close", { reason: "World reconnecting" }));
  expect(replaced).toHaveBeenCalledTimes(1);
  expect((await ordinary.closed).reason).toBe("World reconnecting");
});
