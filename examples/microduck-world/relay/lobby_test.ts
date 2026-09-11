import { assert, assertEquals, assertThrows } from "@std/assert";
import { type Msg } from "../vendor/dimos/web/shared/protocol.ts";
import { type ViewerPeer } from "../vendor/dimos/web/relay/registry.ts";
import { Lobby, NameError, normalizeName, RECONNECT_MS, ROBOTS } from "./lobby.ts";

function viewer(id: number): ViewerPeer {
  return {
    id,
    watched: "world",
    subs: new Set(),
    policies: new Map(),
    greeted: true,
    sendMsg() {},
    sink: {
      sendFrame() {
        throw new Error("unexpected frame");
      },
      openStream() {
        throw new Error("unexpected stream");
      },
      kick() {},
    },
  };
}
Deno.test("six equal player slots and a seventh spectator", () => {
  const lobby = new Lobby();
  const participants = Array.from({ length: 7 }, () => lobby.create()!);
  for (const p of participants.slice(0, 6)) assert(lobby.change(p, "join"));
  assert(lobby.change(participants[0], "join"));
  assertEquals(lobby.change(participants[6], "join"), false);
  assertEquals(Object.keys(lobby.assignments()), ROBOTS);
  assertEquals(participants[6].role, "observe");
});
Deno.test("leaving releases a slot immediately with a new generation on reuse", () => {
  const lobby = new Lobby();
  const first = lobby.create()!, second = lobby.create()!;
  lobby.change(first, "join");
  const generation = first.generation;
  lobby.change(first, "observe");
  assertEquals(lobby.assignments(), {});
  lobby.change(second, "join");
  assertEquals(second.robot, "duck1");
  assert(second.generation !== generation);
});
Deno.test("disconnect reserves a duck for 60 seconds and expires without heartbeats", () => {
  let now = 0;
  const lobby = new Lobby(() => now), p = lobby.create()!, peer = viewer(1);
  lobby.change(p, "join");
  lobby.attach(peer, p, () => {});
  lobby.addViewer(peer);
  lobby.viewerClosed(peer);
  now = RECONNECT_MS - 1;
  assertEquals(Object.keys(lobby.assignments()), ["duck1"]);
  now++;
  assertEquals(lobby.assignments(), {});
  assertEquals(p.role, "observe");
});
Deno.test("reload retains the same assignment and revokes the old transport", () => {
  const lobby = new Lobby(),
    p = lobby.create()!,
    old = viewer(1),
    next = viewer(2);
  lobby.change(p, "join");
  const generation = p.generation;
  let closed = 0;
  lobby.attach(old, p, () => closed++);
  lobby.attach(next, p, () => {});
  assertEquals(closed, 1);
  assertEquals(p.generation, generation);
  assertEquals(
    lobby.onViewerMsg(old, { t: "ping", n: 1, ts: 0 }, () => {}),
    false,
  );
});
Deno.test("observer cannot send motion, policy, navigation, chat or private subscriptions", () => {
  const lobby = new Lobby(), p = lobby.create()!, peer = viewer(1);
  lobby.attach(peer, p, () => {});
  const messages: Msg[] = [
    { t: "teleop_start" },
    { t: "stop", seq: 1, ts: 0 },
    { t: "twist", vx: 1, vy: 0, wz: 0, seq: 1, ts: 0 },
    {
      t: "tx",
      ch: "ui_command",
      seq: 1,
      data: { command: "set_mode", mode: "agent" },
    },
    { t: "tx", ch: "goal_request", seq: 2, data: { x: 1, y: 1 } },
    { t: "tx", ch: "human_input", seq: 3, data: { text: "move" } },
    { t: "sub", ch: "agent" },
  ];
  for (const msg of messages) {
    const replies: Msg[] = [];
    lobby.onViewerMsg(peer, msg, (m) => replies.push(m));
    assertEquals(replies[0].t, "error");
    assertEquals((replies[0] as { code: string }).code, "forbidden");
  }
});
Deno.test("visitor cannot watch or control another duck", () => {
  const lobby = new Lobby(), p = lobby.create()!, peer = viewer(1);
  lobby.change(p, "join");
  lobby.attach(peer, p, () => {});
  const replies: Msg[] = [];
  lobby.onViewerMsg(
    peer,
    { t: "watch", robotId: "duck3" },
    (m) => replies.push(m),
  );
  lobby.onViewerMsg(peer, { t: "teleop_start" }, (m) => replies.push(m));
  assertEquals(replies.map((m) => (m as { code: string }).code), [
    "forbidden",
    "forbidden",
  ]);
});
Deno.test("participant storage is bounded and idle sessions expire", () => {
  let now = 0;
  const lobby = new Lobby(() => now);
  for (let i = 0; i < 128; i++) assert(lobby.create());
  assertEquals(lobby.create(), null);
  now = 3_600_001;
  assert(lobby.create());
  assertEquals(lobby.participants.size, 1);
});

Deno.test("new visitor cannot watch the previous visitor's cached runtime", () => {
  const lobby = new Lobby();
  const first = lobby.create()!, next = lobby.create()!, peer = viewer(1);
  lobby.change(first, "join");
  const oldRuntime = lobby.runtime(first);
  lobby.change(first, "observe");
  lobby.change(next, "join");
  lobby.attach(peer, next, () => {});
  const replies: Msg[] = [];
  lobby.onViewerMsg(
    peer,
    { t: "watch", robotId: oldRuntime },
    (m) => replies.push(m),
  );
  assertEquals((replies[0] as { code: string }).code, "forbidden");
  assert(oldRuntime !== lobby.runtime(next));
  assertEquals(next.robot, "duck1");
});

Deno.test("Duck 1 is an exclusive selectable slot, including the legacy host route", () => {
  const lobby = new Lobby(), first = lobby.create()!, second = lobby.create()!;
  assert(lobby.change(first, "join", "duck1"));
  assertEquals(first.role, "visitor");
  assertEquals(lobby.runtime(first), `duck1-${first.generation}`);
  const generation = first.generation;
  assert(lobby.change(first, "host"));
  assertEquals(first.generation, generation);
  assertEquals(lobby.change(second, "host"), false);
  assertEquals(lobby.change(second, "join", "duck1"), false);
  assertEquals(second.role, "observe");
  assertEquals(lobby.assignments(), { duck1: generation });
  assert(lobby.change(first, "observe"));
  assert(lobby.change(second, "host"));
});

Deno.test("explicit duck choice is respected and conflicts preserve the current assignment", () => {
  const lobby = new Lobby(), first = lobby.create()!, second = lobby.create()!;
  assert(lobby.change(first, "join", "duck3"));
  assertEquals(first.robot, "duck3");
  assert(lobby.change(second, "join", "duck2"));
  const generation = second.generation;
  assertEquals(lobby.change(second, "join", "duck3"), false);
  assertEquals(lobby.change(second, "join", "duck7"), false);
  assertEquals(second.robot, "duck2");
  assertEquals(second.generation, generation);
});

Deno.test("all six ducks fill independently and the seventh participant can watch", () => {
  const lobby = new Lobby();
  for (const robot of ROBOTS) {
    assert(lobby.change(lobby.create()!, "join", robot));
  }
  const observer = lobby.create()!;
  for (const robot of ROBOTS) {
    assertEquals(lobby.change(observer, "join", robot), false);
  }
  assert(lobby.change(observer, "observe"));
  assertEquals(
    lobby.state(observer).slots.map((s) => [s.id, s.occupied, s.mine]),
    ROBOTS.map((id) => [id, true, false]),
  );
});

Deno.test("host availability shows ownership and reconnect reservation before release", () => {
  let now = 0;
  const lobby = new Lobby(() => now), p = lobby.create()!, peer = viewer(1);
  lobby.change(p, "host");
  lobby.attach(peer, p, () => {});
  lobby.addViewer(peer);
  const slot = lobby.state(p).slots[0];
  assertEquals([
    slot.id,
    slot.occupied,
    slot.connected,
    slot.mine,
    slot.reconnectSeconds,
  ], ["duck1", true, true, true, 0]);
  assertEquals(lobby.state().slots[0].mine, false);
  lobby.viewerClosed(peer);
  now = 10_000;
  assertEquals(lobby.state(p).slots[0].reconnectSeconds, 50);
  now = RECONNECT_MS;
  assertEquals(lobby.state(p).slots[0].occupied, false);
  assertEquals(lobby.assignments(), {});
});

Deno.test("names stay attached to one session and never enter physics assignments", () => {
  const lobby = new Lobby(),
    first = lobby.create()!,
    replacement = lobby.create()!;
  lobby.change(first, "join", "duck4", "  Ada   Lovelace  ");
  const generation = first.generation;
  assertEquals(lobby.state().slots[3].displayName, "Ada Lovelace");
  assertEquals(lobby.assignments(), { duck4: generation });
  lobby.change(first, "join", "duck4", "Ada");
  assertEquals(first.generation, generation);
  assertEquals(lobby.state(first).displayName, "Ada");
  lobby.change(first, "observe");
  assertEquals(lobby.state().slots[3].displayName, null);
  lobby.change(replacement, "join", "duck4", "Grace");
  assertEquals(lobby.state().slots[3].displayName, "Grace");
  assert(replacement.generation !== generation);
});

Deno.test("name validation and conflicts preserve existing ownership", () => {
  const lobby = new Lobby(), first = lobby.create()!, next = lobby.create()!;
  lobby.change(first, "join", "duck1", "Tule");
  lobby.change(next, "join", "duck2", "Grace");
  const before = lobby.assignments();
  assertThrows(() => lobby.change(next, "join", "duck3", "tule"), NameError);
  assertThrows(
    () => lobby.change(next, "join", "duck3", "x".repeat(25)),
    NameError,
  );
  assertThrows(
    () => lobby.change(next, "join", "duck3", { name: "Alex" }),
    NameError,
  );
  assertEquals(lobby.assignments(), before);
  assertEquals(next.displayName, "Grace");
  assertEquals(normalizeName(" \u202e\u0000 ", "Blue 1"), "Blue 1");
  assertEquals(normalizeName("<b>Ada</b>", "Blue 1"), "<b>Ada</b>");
});

Deno.test("names survive reconnect grace and clear on expiry", () => {
  let now = 0;
  const lobby = new Lobby(() => now), p = lobby.create()!;
  lobby.change(p, "join", "duck6", "Río");
  now = RECONNECT_MS - 1;
  assertEquals(lobby.state().slots[5].displayName, "Río");
  now++;
  assertEquals(lobby.state().slots[5].displayName, null);
  assertEquals(p.displayName, "");
});
