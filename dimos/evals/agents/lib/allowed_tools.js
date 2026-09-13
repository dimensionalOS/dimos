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

const allowed = JSON.parse(
  readFileSync(new URL("./allowed-tools.json", import.meta.url)),
);
const ready = new URL("./allowed-tools-ready.json", import.meta.url);

export default function (pi) {
  const apply = () => {
    const names = new Set(pi.getAllTools().map((tool) => tool.name));
    const unknown = allowed.filter((name) => !names.has(name));
    pi.setActiveTools(unknown.length ? [] : allowed);
    writeFileSync(
      ready,
      JSON.stringify({ tools: pi.getActiveTools(), unknown }),
    );
  };
  pi.on("session_start", apply);
  pi.on("before_agent_start", apply);
  pi.on("tool_call", (event) => {
    if (!allowed.includes(event.toolName))
      return { block: true, reason: "Tool excluded by eval allowed_tools" };
  });
}
