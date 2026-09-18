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

/** @type {{provider: string, base_url: string, key_env: string, allowed_tools: string[] | null, max_output_tokens: number | null, excluded_keywords: string[], ignored_paths: string[], max_tool_seconds: number | null}} */
const config = JSON.parse(
  readFileSync(new URL("./runtime.json", import.meta.url), "utf8"),
);
const ready = new URL("./runtime-ready.json", import.meta.url);
/** @param {string} s */
const escape = (s) => s.replace(/[.*+?^${}()|[\]\\]/g, "\\$&");
// Whole-token match: "dimos", "import dimos", "dimos_lcm", "/dimos/" hit; "dimosaurus" does not.
const excluded = config.excluded_keywords.map((word) => ({
  word,
  re: new RegExp(`(^|[^a-z0-9])${escape(word)}([^a-z0-9]|$)`),
}));
let blocked = 0;

/** @param {import("@earendil-works/pi-coding-agent").ExtensionAPI} pi */
export default function (pi) {
  pi.registerProvider(config.provider, {
    baseUrl: config.base_url,
    apiKey: "$" + config.key_env,
  });
  /** @type {string[]} */
  let unknown = [];
  const writeState = () =>
    writeFileSync(
      ready,
      JSON.stringify({ tools: pi.getActiveTools(), unknown, blocked }),
    );
  const apply = () => {
    const allowed = config.allowed_tools;
    const names = new Set(pi.getAllTools().map((tool) => tool.name));
    unknown = allowed?.filter((name) => !names.has(name)) ?? [];
    if (allowed !== null) pi.setActiveTools(unknown.length ? [] : allowed);
    writeState();
  };
  pi.on("session_start", apply);
  pi.on("before_agent_start", apply);
  pi.on("tool_call", (event) => {
    if (
      config.allowed_tools !== null &&
      !config.allowed_tools.includes(event.toolName)
    )
      return { block: true, reason: "Tool excluded by eval allowed_tools" };
    const cap = config.max_tool_seconds;
    if (cap !== null && event.toolName === "bash" && event.input) {
      const requested = Number(event.input.timeout);
      event.input.timeout =
        Number.isFinite(requested) && requested > 0
          ? Math.min(requested, cap)
          : cap;
    }
    if (!excluded.length) return;
    let text = JSON.stringify(event.input ?? {}).toLowerCase();
    for (const path of config.ignored_paths)
      text = text.split(path.toLowerCase()).join("");
    const hit = excluded.find(({ re }) => re.test(text));
    if (!hit) return;
    blocked += 1;
    writeState();
    return {
      block: true,
      reason: `Tool call denied: its arguments mention the excluded keyword "${hit.word}". Using it is against the rules of this task; solve the task with any other tool, library or source.`,
    };
  });
  pi.on("before_provider_request", (event) => {
    const cap = config.max_output_tokens;
    if (cap === null) return;
    if (!event.payload || typeof event.payload !== "object")
      throw new Error("Invalid provider payload");
    const payload = /** @type {Record<string, unknown>} */ (event.payload);
    const field = [
      "max_output_tokens",
      "max_tokens",
      "max_completion_tokens",
    ].find((name) => name in payload);
    if (!field)
      throw new Error("Provider does not expose an output token limit");
    const requested = payload[field];
    return {
      ...payload,
      [field]: typeof requested === "number" ? Math.min(requested, cap) : cap,
    };
  });
}
