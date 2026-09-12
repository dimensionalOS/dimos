import assert from "node:assert/strict";
import { mkdtemp, readFile, rm, stat } from "node:fs/promises";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { test } from "node:test";
import { SettingsManager } from "@earendil-works/pi-coding-agent";
import { loadConfig, paths } from "../src/config.js";
import { needsSetup, setup } from "../src/setup.js";

for (const cancel of [false, true])
  test(
    "onboarding " +
      (cancel
        ? "can cancel and resume"
        : "saves provider, model and workspace with private credentials"),
    async (t) => {
      const home = await mkdtemp(join(tmpdir(), "dimcode-setup-"));
      t.after(() => rm(home, { recursive: true, force: true }));
      const p = paths({ DIMCODE_HOME: home });
      assert(await needsSetup(p));
      let asked = 0;
      const configure = setup(
        p,
        {},
        {
          prompt: async (question) => {
            asked++;
            if (cancel && asked === 2) throw new Error("Cancelled");
            if (question.type === "select") {
              if (question.options.some((item) => item.id === "openai"))
                return "openai";
              if (question.options.some((item) => item.id === "api_key"))
                return "api_key";
              if (question.options.some((item) => item.id === "gpt-5.6-luna"))
                return "gpt-5.6-luna";
              return question.options[0].id;
            }
            if (question.type === "secret") return "onboarding-fixture-key";
            return question.message.includes("Workspace") ? home : "";
          },
          notify: () => {},
        },
      );
      if (cancel) {
        await assert.rejects(configure, /Cancelled/);
        assert(await needsSetup(p));
        return;
      }
      await configure;
      assert(!(await needsSetup(p)));
      assert.equal((await loadConfig(p)).workspace, home);
      assert.equal(
        SettingsManager.create(home, p.config).getDefaultProvider(),
        "openai",
      );
      assert.equal(
        SettingsManager.create(home, p.config).getDefaultModel(),
        "gpt-5.6-luna",
      );
      assert.equal((await stat(join(home, "auth.json"))).mode & 0o777, 0o600);
      assert(
        !(await readFile(join(home, "config.json"), "utf8")).includes(
          "fixture-key",
        ),
      );
    },
  );
