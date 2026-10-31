import { describe, expect, it } from "vitest";
import { byRarity, moduleUsage, projectPath, topicOf } from "./model.ts";
import { liveRunOf } from "./run.ts";

describe("model", () => {
  it("names topics without their leading slash, preferring the wired name", () => {
    expect(topicOf({ name: "image", type: "", direction: "out", topic: "/color" })).toBe("color");
    expect(topicOf({ name: "image", type: "", direction: "out", wire: "head" })).toBe("head");
    expect(topicOf({ name: "image", type: "", direction: "out" })).toBe("image");
  });

  it("reads paths from the dimos checkout's root", () => {
    expect(projectPath("/home/me/dimos/dimos/core/module.py")).toBe("dimos/core/module.py");
  });

  it("lists the rarest modules first", () => {
    const uses = moduleUsage({ blueprints: [{ modules: ["Lidar", "Map"] }, { modules: ["Map"] }] });
    const modules = ["map", "lidar", "solo"].map((name) => ({ name, class: name, streams: [] }));
    expect(byRarity(modules, uses).map((module) => module.name)).toEqual(["solo", "lidar", "map"]);
  });

  it("finds the blueprint's live run however it was started, newest first", () => {
    const run = (run_id: string, blueprint: string) => ({
      run_id,
      blueprint,
      pid: 1,
      started_at: "",
      log_dir: "",
    });
    // GET /runs lists newest first; a tmux `dimos run` has no Desktop launch, only this entry
    const runs = [run("3-other", "other"), run("2-nav", "r1pro-nav"), run("1-nav", "r1pro-nav")];
    expect(liveRunOf(runs, "r1pro-nav", null)?.run_id).toBe("2-nav");
    expect(liveRunOf(runs, "r1pro-nav", "1-nav")?.run_id).toBe("1-nav");
    expect(liveRunOf(runs, "r1pro-nav", "gone")?.run_id).toBe("2-nav");
    expect(liveRunOf(runs, "absent", null)).toBeNull();
  });
});
