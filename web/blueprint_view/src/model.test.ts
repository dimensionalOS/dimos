import { describe, expect, it } from "vitest";
import { byRarity, moduleUsage, projectPath, topicOf } from "./model.ts";

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
    expect(byRarity(modules, uses).map((m) => m.name)).toEqual(["solo", "lidar", "map"]);
  });
});
