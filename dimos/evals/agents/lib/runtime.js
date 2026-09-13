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

/** @type {{provider: string, base_url: string, key_env: string, allowed_tools: string[] | null, max_output_tokens: number | null}} */
const config = JSON.parse(
  readFileSync(new URL("./runtime.json", import.meta.url), "utf8"),
);
const ready = new URL("./runtime-ready.json", import.meta.url);

/** @param {import("@earendil-works/pi-coding-agent").ExtensionAPI} pi */
export default function (pi) {
  pi.registerProvider(config.provider, {
    baseUrl: config.base_url,
    apiKey: "$" + config.key_env,
  });
  const apply = () => {
    const allowed = config.allowed_tools;
    const names = new Set(pi.getAllTools().map((tool) => tool.name));
    const unknown = allowed?.filter((name) => !names.has(name)) ?? [];
    if (allowed !== null) pi.setActiveTools(unknown.length ? [] : allowed);
    writeFileSync(
      ready,
      JSON.stringify({ tools: pi.getActiveTools(), unknown }),
    );
  };
  pi.on("session_start", apply);
  pi.on("before_agent_start", apply);
  pi.on("tool_call", (event) => {
    if (
      config.allowed_tools !== null &&
      !config.allowed_tools.includes(event.toolName)
    )
      return { block: true, reason: "Tool excluded by eval allowed_tools" };
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
