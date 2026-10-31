import assert from "node:assert/strict";
import type { HarnessEvent } from "#dimos/agents/shared/protocol/runtime/event.ts";
import type { InputDraft } from "#dimos/agents/harness/inputs/gate.ts";
import { BACKGROUND_CONTEXT } from "@earendil-works/chord/context";
import { EventDrafts } from "#dimos/agents/harness/events/message.ts";
import { EventIntake } from "#dimos/agents/harness/events/intake.ts";
import { FileRenderer } from "#dimos/agents/shared/render/render.ts";
import { WireError } from "#dimos/agents/harness/dimos/wire/base.ts";
import { createHash, randomUUID } from "node:crypto";
import { harnessEventDecoder } from "#dimos/agents/harness/dimos/wire/event.ts";
import { join } from "node:path";
import { mkdtemp, rm, writeFile } from "node:fs/promises";
import { requestIdFor } from "#dimos/agents/harness/inputs/source.ts";
import { test } from "node:test";
import { tmpdir } from "node:os";

function notice(overrides: Partial<HarnessEvent> = {}): HarnessEvent {
  return harnessEventDecoder.decode({
    id: randomUUID(),
    module: "beacon",
    conversation: 7,
    at: 1_760_000_000,
    text: "beat",
    data: { n: 1 },
    exports: [],
    suppressed: 0,
    ...overrides,
  });
}

function liveConnection() {
  const listeners = new Set<(event: HarnessEvent) => void>();
  return {
    onEvent(listener: (event: HarnessEvent) => void): () => void {
      listeners.add(listener);
      return () => {
        listeners.delete(listener);
      };
    },
    emit(event: HarnessEvent): void {
      for (const listener of listeners) listener(event);
    },
  };
}

function renderer(cacheDir: string): FileRenderer {
  return new FileRenderer({ cacheDir, maxCacheBytes: 10_000_000, maxSourceBytes: 1_000_000 });
}

test("wire notices require a UUID, and their request ID is stable without a log or sequence", () => {
  const event = notice();
  assert.equal(requestIdFor({ kind: "event", event }), `dimos-event:${event.id}`);
  assert.equal(requestIdFor({ kind: "event", event: { ...event, text: "changed" } }), `dimos-event:${event.id}`);
  assert.throws(() => harnessEventDecoder.decode({ ...event, id: "not-a-uuid" }), WireError);
  const { id: _id, ...legacy } = event;
  assert.throws(() => harnessEventDecoder.decode({ ...legacy, log: "old", seq: 1 }), WireError);
});

test("live notices keep arrival order and shutdown unsubscribes then drains received work", async () => {
  const connection = liveConnection();
  const drafts = new EventDrafts(renderer("unused"));
  const delivered: InputDraft[] = [];
  const reports: unknown[] = [];
  const first = notice();
  const second = notice({ text: "second" });
  let release = () => {};
  const held = new Promise<void>((resolve) => {
    release = resolve;
  });
  const intake = new EventIntake({
    connection,
    drafts: {
      forEvent: async (event) => {
        if (event.id === first.id) await held;
        return drafts.forEvent(event);
      },
    },
    inputs: {
      submit: async (conversation, draft, context) => {
        assert.equal(conversation, first.conversation);
        assert.equal(context, BACKGROUND_CONTEXT);
        delivered.push(draft);
        return { kind: "nothing_to_say" };
      },
    },
    report: (error) => reports.push(error),
  });
  connection.emit(notice());
  intake.start(BACKGROUND_CONTEXT);
  intake.start(BACKGROUND_CONTEXT);
  connection.emit(first);
  connection.emit(second);
  const stopped = intake.stop();
  connection.emit(notice());
  assert.equal(delivered.length, 0);
  release();
  await stopped;
  assert.deepEqual(
    delivered.map((draft) => draft.source),
    [
      { kind: "event", event: first },
      { kind: "event", event: second },
    ],
  );
  assert(delivered.every((draft) => draft.whenBusy === "followUp"));
  assert.deepEqual(reports, []);
});

test("a failed delivery and missing owners do not prevent the next live notice", async () => {
  const connection = liveConnection();
  const drafts = new EventDrafts(renderer("unused"));
  const attempted: string[] = [];
  const reports: unknown[] = [];
  const failed = notice({ text: "failed" });
  const missing = notice({ text: "missing" });
  const next = notice({ text: "next" });
  const intake = new EventIntake({
    connection,
    drafts,
    inputs: {
      submit: async (_conversation, draft) => {
        assert.equal(draft.source.kind, "event");
        if (draft.source.kind !== "event") throw new Error("expected event");
        const event = draft.source.event;
        attempted.push(event.id);
        if (event.id === failed.id) throw new Error("delivery failed");
        return { kind: event.id === missing.id ? "unknown_conversation" : "nothing_to_say" };
      },
    },
    report: (error) => reports.push(error),
  });
  intake.start(BACKGROUND_CONTEXT);
  connection.emit(notice({ conversation: null }));
  connection.emit(failed);
  connection.emit(missing);
  connection.emit(next);
  await intake.stop();
  assert.deepEqual(attempted, [failed.id, missing.id, next.id]);
  assert.equal(reports.length, 3);
  assert.match(String(reports[0]), /has no owning conversation; dropped/);
  assert.equal(String(reports[1]), "Error: delivery failed");
  assert.match(String(reports[2]), /names conversation 7, which does not exist; dropped/);
});

test("notice drafts preserve ownership, exported images, data, and suppression counts", async (t) => {
  const dir = await mkdtemp(join(tmpdir(), "dimos-live-notice-"));
  t.after(() => rm(dir, { recursive: true, force: true }));
  const path = join(dir, "frame.svg");
  const svg =
    '<svg xmlns="http://www.w3.org/2000/svg" width="16" height="16"><rect width="16" height="16" fill="red"/></svg>';
  await writeFile(path, svg);
  const event = harnessEventDecoder.decode({
    ...notice({ suppressed: 3 }),
    exports: [{ kind: "image", path, sha256: createHash("sha256").update(svg).digest("hex") }],
  });
  const draft = await new EventDrafts(renderer(join(dir, "cache"))).forEvent(event);
  assert.deepEqual(draft.source, { kind: "event", event });
  assert.equal(draft.images.length, 1);
  assert.equal(draft.render?.views[0]?.kind, "image");
  assert.match(draft.text, /beacon: beat/);
  assert.match(draft.text, /Data: \{"n":1\}/);
  assert.match(draft.text, /3 earlier notices/);
});
