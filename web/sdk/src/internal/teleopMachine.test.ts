import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import type { Msg } from "@dimos/shared";
import type { ChannelSpec } from "@dimos/shared/manifest";
import {
  TELEOP_DEFAULTS,
  teleopConfigFromChannel,
  TeleopMachine,
  type TeleopSnapshot,
} from "./teleopMachine.ts";

interface Sent {
  via: "control" | "datagram";
  msg: Msg;
}

function makeMachine(config = TELEOP_DEFAULTS): { machine: TeleopMachine; sent: Sent[] } {
  const sent: Sent[] = [];
  const machine = new TeleopMachine(config, {
    control: (msg) => sent.push({ via: "control", msg }),
    datagram: (msg) => sent.push({ via: "datagram", msg }),
  });
  return { machine, sent };
}

function armed(): { machine: TeleopMachine; sent: Sent[] } {
  const { machine, sent } = makeMachine();
  machine.arm();
  machine.onRelayMsg({ t: "teleop_started" });
  sent.length = 0;
  return { machine, sent };
}

function twists(sent: Sent[]): { vx: number; vy: number; wz: number; seq: number }[] {
  return sent.flatMap(({ msg }) => (msg.t === "twist" ? [msg] : []));
}

function snap(machine: TeleopMachine): TeleopSnapshot {
  return machine.getSnapshot();
}

describe("TeleopMachine", () => {
  beforeEach(() => vi.useFakeTimers());
  afterEach(() => vi.useRealTimers());

  it("arms only after the relay grants the lease", () => {
    const { machine, sent } = makeMachine();
    machine.arm();
    expect(sent).toEqual([{ via: "control", msg: { t: "teleop_start" } }]);
    expect(snap(machine).phase).toBe("arming");
    machine.keyDown("KeyW");
    expect(twists(sent)).toHaveLength(0); // not armed yet: nothing drives
    machine.onRelayMsg({ t: "teleop_started" });
    expect(snap(machine).phase).toBe("armed");
  });

  it("a refused lease disarms with the reason and sends nothing", () => {
    const { machine, sent } = makeMachine();
    machine.arm();
    machine.onRelayMsg({ t: "error", code: "teleop_held", message: "held" });
    expect(snap(machine).phase).toBe("disarmed");
    expect(snap(machine).reason).toBe("teleop held by another viewer");
    expect(sent).toHaveLength(1); // only the teleop_start
  });

  it("streams twists at publishHz while a motion key is held", () => {
    const { machine, sent } = armed();
    machine.keyDown("KeyW");
    expect(twists(sent)).toHaveLength(1); // immediate send on the edge
    vi.advanceTimersByTime(1000);
    const stream = twists(sent);
    // 1 edge send + ~15 interval ticks (exact count is timer float rounding).
    expect(stream.length).toBeGreaterThanOrEqual(15);
    expect(stream.length).toBeLessThanOrEqual(17);
    for (const t of stream) {
      expect(t).toMatchObject({ vx: TELEOP_DEFAULTS.maxLinear, vy: 0, wz: 0 });
    }
    const seqs = stream.map((t) => t.seq);
    expect(seqs).toEqual([...seqs].sort((a, b) => a - b));
    expect(new Set(seqs).size).toBe(seqs.length); // strictly increasing
  });

  it("applies the pygame key map with assignment semantics", () => {
    const { machine, sent } = armed();
    // S wins over W, E over Q, D over A (checked in that order).
    for (const code of ["KeyW", "KeyS", "KeyQ", "KeyE", "KeyA", "KeyD"]) machine.keyDown(code);
    const last = twists(sent).at(-1);
    expect(last).toMatchObject({
      vx: -TELEOP_DEFAULTS.maxLinear,
      vy: -TELEOP_DEFAULTS.maxLinear,
      wz: -TELEOP_DEFAULTS.maxAngular,
    });
  });

  it("Shift boosts and its release drops back", () => {
    const { machine, sent } = armed();
    machine.keyDown("KeyW");
    machine.keyDown("ShiftLeft");
    expect(twists(sent).at(-1)?.vx).toBe(TELEOP_DEFAULTS.maxLinear * TELEOP_DEFAULTS.boost);
    expect(snap(machine).boosted).toBe(true);
    machine.keyUp("ShiftLeft");
    expect(twists(sent).at(-1)?.vx).toBe(TELEOP_DEFAULTS.maxLinear);
  });

  it("the last release sends zero now plus two repeats, then goes silent", () => {
    const { machine, sent } = armed();
    machine.keyDown("KeyW");
    machine.keyUp("KeyW");
    const zeros = () => twists(sent).filter((t) => t.vx === 0 && t.vy === 0 && t.wz === 0);
    expect(zeros()).toHaveLength(1);
    vi.advanceTimersByTime(100);
    expect(zeros()).toHaveLength(2);
    vi.advanceTimersByTime(100);
    expect(zeros()).toHaveLength(3);
    const total = sent.length;
    vi.advanceTimersByTime(2000);
    expect(sent.length).toBe(total); // silence after the burst
  });

  it("a keydown during the release burst cancels it and resumes streaming", () => {
    const { machine, sent } = armed();
    machine.keyDown("KeyW");
    machine.keyUp("KeyW");
    vi.advanceTimersByTime(50);
    machine.keyDown("KeyW");
    sent.length = 0;
    vi.advanceTimersByTime(300);
    expect(twists(sent).every((t) => t.vx === TELEOP_DEFAULTS.maxLinear)).toBe(true);
  });

  it("Space clears keys, bursts stops on both paths, and stays armed", () => {
    const { machine, sent } = armed();
    machine.keyDown("KeyW");
    machine.keyDown("ShiftLeft");
    sent.length = 0;
    machine.estop();
    expect(sent.filter((s) => s.via === "control" && s.msg.t === "stop")).toHaveLength(1);
    const stops = () => sent.filter((s) => s.via === "datagram" && s.msg.t === "stop");
    expect(stops()).toHaveLength(1);
    vi.advanceTimersByTime(200);
    expect(stops()).toHaveLength(3);
    expect(snap(machine).phase).toBe("armed");
    expect(snap(machine).pressed.size).toBe(0);
    const total = sent.length;
    vi.advanceTimersByTime(1000);
    expect(sent.length).toBe(total); // the drive interval died with the keys
  });

  it("every disarm trigger zeroes once and releases the lease", () => {
    for (const reason of ["escape", "focus lost", "window blurred", "tab hidden", "unmounted"]) {
      const { machine, sent } = armed();
      machine.keyDown("KeyW");
      sent.length = 0;
      machine.disarm(reason);
      expect(twists(sent)).toEqual([
        { t: "twist", vx: 0, vy: 0, wz: 0, seq: expect.any(Number), ts: expect.any(Number) },
      ]);
      expect(sent.filter((s) => s.msg.t === "teleop_stop")).toHaveLength(1);
      expect(snap(machine)).toMatchObject({ phase: "disarmed", reason });
      const total = sent.length;
      vi.advanceTimersByTime(2000);
      expect(sent.length).toBe(total); // all timers stopped
    }
  });

  it("connection loss disarms locally without wire sends", () => {
    const { machine, sent } = armed();
    machine.keyDown("KeyW");
    sent.length = 0;
    machine.connectionChanged(false);
    expect(sent).toHaveLength(0);
    expect(snap(machine)).toMatchObject({ phase: "disarmed", reason: "connection lost" });
    vi.advanceTimersByTime(2000);
    expect(sent).toHaveLength(0);
  });

  it("never sends while disarmed", () => {
    const { machine, sent } = makeMachine();
    machine.keyDown("KeyW");
    machine.keyUp("KeyW");
    machine.estop();
    machine.disarm("noop");
    vi.advanceTimersByTime(1000);
    expect(sent).toHaveLength(0);
  });
});

describe("teleopConfigFromChannel", () => {
  const base: ChannelSpec = {
    ch: "tele_cmd_vel",
    dir: "tx",
    encoding: "twist.json.v1",
    delivery: "latest",
    maxHz: 10,
    params: { maxLinear: 0.5, maxAngular: 0.9, boost: 3 },
    publish: "none",
    requiredScope: null,
  };

  it("reads params and maxHz from the channel spec", () => {
    expect(teleopConfigFromChannel(base)).toEqual({
      maxLinear: 0.5,
      maxAngular: 0.9,
      boost: 3,
      publishHz: 10,
    });
  });

  it("falls back per-field on junk (params crossed the wire)", () => {
    const junk = {
      ...base,
      maxHz: -1,
      params: { maxLinear: "fast", maxAngular: NaN, boost: 0 },
    };
    expect(teleopConfigFromChannel(junk)).toEqual(TELEOP_DEFAULTS);
    expect(teleopConfigFromChannel(undefined)).toEqual(TELEOP_DEFAULTS);
  });
});
