import assert from "node:assert/strict";
import type { Facts } from "#dimos/agents/shared/protocol/runtime/context.ts";
import type { Observation } from "#dimos/agents/harness/context/facts.ts";
import { applyChange, diffFacts, observedFacts } from "#dimos/agents/harness/context/facts.ts";
import { changeMessage } from "#dimos/agents/harness/context/messages.ts";
import { test } from "node:test";

const GO2: Facts = {
  "env:dimos": { section: "Environment", line: "DimOS 0.4 @ 03bcca09 · transport zenoh" },
  "run:20261016-115933-983c-unitree-go2": {
    section: "Environment",
    line: "Run 20261016-115933-983c-unitree-go2 (blueprint unitree-go2)",
  },
  "module:GO2Connection": { section: "Modules", line: "GO2Connection [GO2Connection]" },
  "module:ReplanningAStarPlanner": { section: "Modules", line: "ReplanningAStarPlanner [ReplanningAStarPlanner]" },
  "rpc:ReplanningAStarPlanner.set_goal": { section: "RPCs", line: "ReplanningAStarPlanner.set_goal(goal) -> bool" },
  "rpc:GO2Connection.move": { section: "RPCs", line: "GO2Connection.move(twist, duration) -> bool" },
  "stream:ReplanningAStarPlanner.path": { section: "Streams", line: "ReplanningAStarPlanner.path out Path" },
  "job:patrol": { section: "Jobs", line: "patrol every 10m" },
};

function system(facts: Facts): Facts {
  return Object.fromEntries(Object.entries(facts).filter(([key]) => !/^(run|job):/.test(key)));
}

function observe(told: Facts, observation: Observation): { message: string | null; told: Facts } {
  const change = diffFacts(told, observedFacts(told, observation));
  return {
    message: change === null ? null : changeMessage(change, told),
    told: change === null ? told : applyChange(told, change),
  };
}

test("the first system is stated in full; an outage is one status line and keeps the system as last seen", () => {
  const up: Observation = {
    kind: "up",
    system: system(GO2),
    gateway: {
      "job:patrol": GO2["job:patrol"],
      "run:20261016-115933-983c-unitree-go2": GO2["run:20261016-115933-983c-unitree-go2"],
    },
  };
  const first = observe({}, up);
  assert.match(first.message ?? "", /^<dimos_runtime>\nCurrent DimOS runtime\./);
  assert.deepEqual(first.told, GO2);

  const down = observe(first.told, {
    kind: "down",
    status: {
      section: "Environment",
      line: "DimOS: not reachable, link reconnecting; any runtime stated earlier is as last seen",
    },
    gateway: { "job:patrol": GO2["job:patrol"] },
  });
  assert.equal(
    down.message,
    [
      "<dimos_runtime_change>",
      "DimOS: not reachable, link reconnecting; any runtime stated earlier is as last seen. Run 20261016-115933-983c-unitree-go2 ended.",
      "</dimos_runtime_change>",
    ].join("\n"),
  );
  assert.deepEqual(Object.keys(down.told).sort(), [
    "dimos:status",
    "env:dimos",
    "job:patrol",
    "module:GO2Connection",
    "module:ReplanningAStarPlanner",
    "rpc:GO2Connection.move",
    "rpc:ReplanningAStarPlanner.set_goal",
    "stream:ReplanningAStarPlanner.path",
  ]);
  assert.equal(
    observe(down.told, {
      kind: "down",
      status: down.told["dimos:status"],
      gateway: { "job:patrol": GO2["job:patrol"] },
    }).message,
    null,
  );
});

test("a restart on another blueprint is a notice and a delta against the last system, a removed module named once", () => {
  const { told } = observe(GO2, {
    kind: "down",
    status: {
      section: "Environment",
      line: "DimOS: not reachable, link down; any runtime stated earlier is as last seen",
    },
    gateway: { "job:patrol": GO2["job:patrol"] },
  });
  const basic: Observation = {
    kind: "up",
    system: {
      "env:dimos": GO2["env:dimos"],
      "module:GO2Connection": GO2["module:GO2Connection"],
      "rpc:GO2Connection.move": GO2["rpc:GO2Connection.move"],
    },
    gateway: {
      "job:patrol": GO2["job:patrol"],
      "run:20261016-155403-7dc5-unitree-go2-basic": {
        section: "Environment",
        line: "Run 20261016-155403-7dc5-unitree-go2-basic (blueprint unitree-go2-basic)",
      },
    },
  };
  assert.equal(
    observe(told, basic).message,
    [
      "<dimos_runtime_change>",
      "DimOS: reachable again. Run 20261016-155403-7dc5-unitree-go2-basic (blueprint unitree-go2-basic) started.",
      "DimOS runtime changes since the runtime state stated earlier; they come from the gateway, not from the user.",
      "Modules:",
      "  - ReplanningAStarPlanner (with its RPCs and streams)",
      "</dimos_runtime_change>",
    ].join("\n"),
  );
});
