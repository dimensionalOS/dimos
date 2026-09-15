// Copyright 2026 Dimensional Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

import { readFileSync, writeFileSync } from "node:fs";
import {
  createBashToolDefinition,
  createGrepToolDefinition,
} from "@earendil-works/pi-coding-agent";

/** @type {{command: string[], tools: string[]}} */
const { command, tools } = JSON.parse(
  readFileSync(new URL("./sandbox.json", import.meta.url), "utf8"),
);
/** @param {string} value */
const quote = (value) => "'" + value.replaceAll("'", "'\"'\"'") + "'";

/** @param {import("@earendil-works/pi-coding-agent").ExtensionAPI} pi */
export default function (pi) {
  const bash = createBashToolDefinition("/workspace", {
    shellPath: "/bin/bash",
    exposeSessionEnvironment: false,
    spawnHook: ({ command: code, cwd }) => ({
      command: [...command, code].map(quote).join(" "),
      cwd,
      env: { PATH: "/usr/bin:/bin", HOME: "/tmp" },
    }),
  });
  if (tools.includes("bash")) pi.registerTool(bash);
  if (tools.includes("grep")) {
    const grep = createGrepToolDefinition("/workspace");
    pi.registerTool({
      ...grep,
      description:
        "Search file contents with ripgrep in the isolated environment. Returns matching lines and optional context, truncated to the requested line limit.",
      execute: (id, args, signal, update, ctx) => {
        const argv = ["rg", "--line-number", "--color=never", "--hidden"];
        if (args.ignoreCase) argv.push("--ignore-case");
        if (args.literal) argv.push("--fixed-strings");
        if (args.glob) argv.push("--glob", args.glob);
        if (args.context) argv.push("--context", String(args.context));
        argv.push("--", args.pattern, args.path ?? "/workspace");
        const code =
          argv.map(quote).join(" ") +
          " | head -n " +
          Math.max(1, Math.floor(args.limit ?? 100)) +
          '\nstatus=${PIPESTATUS[0]}; if (( status > 1 && status != 141 )); then exit "$status"; fi';
        return bash.execute(id, { command: code }, signal, update, ctx);
      },
    });
  }
  writeFileSync(new URL("./sandbox-ready", import.meta.url), "ready");
}
