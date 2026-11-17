// @vitest-environment happy-dom
import { afterEach, describe, expect, it, vi } from "vitest";
import { decodeDataFrame, type Msg } from "@dimos/shared";
import { TopicClient } from "./topics.ts";

describe("provider-owned topic publishers", () => {
  const clients: TopicClient[] = [];
  afterEach(() => {
    for (const client of clients.splice(0)) client.close();
    vi.useRealTimers();
    vi.restoreAllMocks();
  });
  function setup(write = true, guardedPublishers = true) {
    const client = new TopicClient("workspace");
    clients.push(client);
    const sent: Msg[] = [];
    const raw: Uint8Array[] = [];
    const guarded: Uint8Array[] = [];
    client.connected((msg) => sent.push(msg), vi.fn(), (bytes) => {
      raw.push(bytes);
      return Promise.resolve();
    }, (bytes) => {
      guarded.push(bytes);
      return Promise.resolve();
    });
    const sources = (gen = 1) =>
      client.onMessage({
        t: "topic_sources",
        v: 1,
        sources: [{
          source: "workspace",
          gen,
          read: true,
          write,
          profiles: ["raw"],
          guardedPublishers,
        }],
      });
    sources();
    const open = (stop = new Uint8Array([0, 255])) =>
      client.publisher("robot/cmd", stop, { group: "drive", timeoutMs: 400 });
    const ready = () => {
      const id = decodeDataFrame(guarded[0]).header.meta!.id as string;
      client.onMessage({
        t: "topic_guard_ready",
        id,
        handle: "@p:1",
        gen: 1,
        ticket: 1,
        timeoutMs: 400,
      });
    };
    return { client, sent, raw, guarded, sources, open, ready };
  }

  it("requires separate write permission and an advertised provider watchdog", () => {
    expect(() => setup(false).open()).toThrow("guarded publishers unavailable");
    expect(() => setup(true, false).open()).toThrow("guarded publishers unavailable");
  });

  it("copies the safe stop, waits for provider readiness, and uses the independent command lane", async () => {
    const h = setup();
    const stop = new Uint8Array([0, 255]);
    const publisher = h.open(stop);
    stop.fill(7);
    expect(decodeDataFrame(h.guarded[0]).payload).toEqual(new Uint8Array([0, 255]));
    await expect(publisher.publish(new Uint8Array([1]))).rejects.toMatchObject({
      code: "guard_not_ready",
    });
    h.ready();
    await publisher.ready();
    const receipt = publisher.publish(new Uint8Array([1, 128, 0]));
    const command = decodeDataFrame(h.guarded[1]);
    expect(command.header).toMatchObject({
      ch: "@guard",
      seq: 1,
      meta: { action: "command", handle: "@p:1", gen: 1, ticket: 1 },
    });
    expect(command.payload).toEqual(new Uint8Array([1, 128, 0]));
    expect(h.raw).toEqual([]);
    h.client.onMessage({
      t: "topic_guard_result",
      handle: "@p:1",
      gen: 1,
      seq: 1,
      outcome: "accepted",
      code: "",
      providerTs: 123,
    });
    await expect(receipt).resolves.toEqual({ id: "@p:1:1", providerTs: 123 });
  });

  it("closes a late provider handle after the caller cancelled while opening", async () => {
    const h = setup();
    const publisher = h.open();
    publisher.close();
    h.ready();
    await expect(publisher.ready()).rejects.toMatchObject({ code: "released" });
    expect(h.sent.at(-1)).toEqual({ t: "topic_guard_stop", handle: "@p:1", gen: 1 });
    expect(publisher.status().phase).toBe("closed");
  });

  it("bounds commands until provider receipts and never queues more motion", async () => {
    const h = setup();
    const publisher = h.open();
    h.ready();
    const pending = Array.from(
      { length: 8 },
      () => publisher.publish(new Uint8Array([1])).catch((error) => error),
    );
    await expect(publisher.publish(new Uint8Array([2]))).rejects.toMatchObject({
      code: "guard_busy",
    });
    expect(h.guarded).toHaveLength(9);
    publisher.close();
    expect((await Promise.all(pending)).map((error) => error.outcome)).toEqual(
      Array(8).fill("unknown"),
    );
    await expect(publisher.publish(new Uint8Array([3]))).rejects.toMatchObject({
      code: "guard_not_ready",
    });
  });

  it("retires ownership on source restart without replaying or rearming", async () => {
    const h = setup();
    const publisher = h.open();
    h.ready();
    const pending = publisher.publish(new Uint8Array([1]));
    h.sources(2);
    await expect(pending).rejects.toMatchObject({ outcome: "unknown", code: "source_offline" });
    expect(publisher.status().phase).toBe("interrupted");
    expect(h.guarded).toHaveLength(2);
    await expect(publisher.publish(new Uint8Array([2]))).rejects.toMatchObject({
      code: "guard_not_ready",
    });
  });

  it("stops on focus loss and does not restore an armed handle on focus", async () => {
    const h = setup();
    const publisher = h.open();
    h.ready();
    globalThis.dispatchEvent(new Event("blur"));
    globalThis.dispatchEvent(new Event("focus"));
    expect(h.sent.at(-1)).toEqual({ t: "topic_guard_stop", handle: "@p:1", gen: 1 });
    expect(publisher.status()).toMatchObject({
      phase: "interrupted",
      error: { code: "focus_lost" },
    });
    await expect(publisher.publish(new Uint8Array([1]))).rejects.toMatchObject({
      code: "guard_not_ready",
    });
  });

  it("expires when provider heartbeat replies stop, including with pending commands", async () => {
    vi.useFakeTimers({
      toFake: ["setInterval", "clearInterval", "setTimeout", "clearTimeout", "performance"],
    });
    const h = setup();
    const publisher = h.open();
    h.ready();
    const pending = publisher.publish(new Uint8Array([1])).catch((error) => error);
    await vi.advanceTimersByTimeAsync(400);
    expect(publisher.status()).toMatchObject({
      phase: "interrupted",
      error: { code: "guard_heartbeat_timeout" },
    });
    expect((await pending).outcome).toBe("unknown");
    expect(h.sent.at(-1)?.t).toBe("topic_guard_stop");
  });

  it("retires an idle handle when the provider expires it despite healthy heartbeats", async () => {
    const h = setup();
    const publisher = h.open();
    h.ready();
    await publisher.ready();
    h.client.onMessage({ t: "topic_guard_ticket", handle: "@p:1", gen: 1, ticket: 2 });
    h.client.onMessage({ t: "topic_guard_closed", handle: "@p:1", gen: 1, code: "guard_expired" });
    expect(publisher.status()).toMatchObject({
      phase: "interrupted",
      error: { code: "guard_expired" },
    });
    await expect(publisher.publish(new Uint8Array([1]))).rejects.toMatchObject({
      code: "guard_not_ready",
    });
    expect(h.guarded).toHaveLength(1);
  });

  it("stops on visibility loss and does not rearm when visible again", async () => {
    const h = setup();
    const publisher = h.open();
    h.ready();
    const hidden = vi.spyOn(document, "hidden", "get").mockReturnValue(true);
    document.dispatchEvent(new Event("visibilitychange"));
    hidden.mockReturnValue(false);
    document.dispatchEvent(new Event("visibilitychange"));
    expect(h.sent.at(-1)).toEqual({ t: "topic_guard_stop", handle: "@p:1", gen: 1 });
    await expect(publisher.publish(new Uint8Array([1]))).rejects.toMatchObject({
      code: "guard_not_ready",
    });
  });

  it("refuses to open a handle while the page is hidden", () => {
    const h = setup();
    const hidden = vi.spyOn(document, "hidden", "get").mockReturnValue(true);
    document.dispatchEvent(new Event("visibilitychange"));
    expect(() => h.open()).toThrow("focus lost");
    expect(h.guarded).toEqual([]);
    hidden.mockReturnValue(false);
    document.dispatchEvent(new Event("visibilitychange"));
    h.open();
    expect(h.guarded).toHaveLength(1);
  });

  it("accepts only current-generation provider receipts", async () => {
    const h = setup();
    const publisher = h.open();
    h.ready();
    const completed = vi.fn();
    const receipt = publisher.publish(new Uint8Array([1]));
    receipt.then(completed);
    h.client.onMessage({
      t: "topic_guard_result",
      handle: "@p:1",
      gen: 0,
      seq: 1,
      outcome: "accepted",
      code: "",
      providerTs: 1,
    });
    await Promise.resolve();
    expect(completed).not.toHaveBeenCalled();
    h.client.onMessage({
      t: "topic_guard_result",
      handle: "@p:1",
      gen: 1,
      seq: 1,
      outcome: "accepted",
      code: "",
      providerTs: 2,
    });
    await expect(receipt).resolves.toMatchObject({ providerTs: 2 });
  });
});
