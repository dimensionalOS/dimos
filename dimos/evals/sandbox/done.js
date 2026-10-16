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

// The tool a sandboxed agent calls to claim the task is done. Pi validates its
// arguments against the case's JSON schema; a valid call ends the agent's turn. What
// happens next (another user message, or the end) is the case's human's call.

import { readFileSync } from "node:fs";

/** @type {{name: string, description: string, schema: object, accepted: string}} */
const config = JSON.parse(
  readFileSync(new URL("./done.json", import.meta.url), "utf8"),
);

/** @param {import("@earendil-works/pi-coding-agent").ExtensionAPI} pi */
export default function (pi) {
  pi.registerTool({
    name: config.name,
    label: "Done",
    description: config.description,
    parameters: config.schema,
    async execute() {
      return {
        content: [{ type: "text", text: config.accepted }],
        details: undefined,
        terminate: true,
      };
    },
  });
}
