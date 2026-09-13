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

import { execFile } from "node:child_process";
import { readFileSync, writeFileSync } from "node:fs";
import { promisify } from "node:util";
import {
  createBashToolDefinition,
  createGrepToolDefinition,
} from "@earendil-works/pi-coding-agent";

const { command, tools } = JSON.parse(
  readFileSync(new URL("./sandbox.json", import.meta.url)),
);
const quote = (s) => "'" + s.replaceAll("'", "'\"'\"'") + "'";
const exec = promisify(execFile);
const shell = async (code) =>
  (await exec(command[0], [...command.slice(1), code])).stdout;

export default function (pi) {
  if (tools.includes("bash")) {
    pi.registerTool(
      createBashToolDefinition("/workspace", {
        shellPath: "/bin/bash",
        exposeSessionEnvironment: false,
        spawnHook: ({ command: code, cwd }) => ({
          command: [...command, code].map(quote).join(" "),
          cwd,
          env: { PATH: "/usr/bin:/bin", HOME: "/tmp" },
        }),
      }),
    );
  }
  if (tools.includes("grep")) {
    const grep = createGrepToolDefinition("/workspace", {
      operations: {
        isDirectory: async (path) => {
          const kind = (await shell("stat -L -c %F -- " + quote(path))).trim();
          return kind === "directory";
        },
        readFile: (path) => shell("cat -- " + quote(path)),
      },
    });
    pi.registerTool({
      ...grep,
      execute: (id, args, signal, update, ctx) =>
        grep.execute(id, args, signal, update, { ...ctx, cwd: "/workspace" }),
    });
  }
  writeFileSync(new URL("./sandbox-ready", import.meta.url), "ready");
}
