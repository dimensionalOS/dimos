// Frank-local provider registration; credentials come from loop.py's environment.
import type { ExtensionAPI } from "@earendil-works/pi-coding-agent";

export default function (pi: ExtensionAPI) {
  pi.registerProvider("cerebras", {
    name: "Cerebras",
    baseUrl: "https://api.cerebras.ai/v1",
    apiKey: "$CEREBRAS_API_KEY",
    api: "openai-completions",
    models: [
      {
        id: "gemma-4-31b",
        name: "Gemma 4 31B (Cerebras)",
        reasoning: true,
        cost: { input: 0.99, output: 1.49, cacheRead: 0.99, cacheWrite: 0.99 },
        input: ["text", "image"],
        contextWindow: 131072,
        maxTokens: 8192,
        compat: { supportsDeveloperRole: false, supportsReasoningEffort: false, supportsStore: false },
      },
    ],
  });
}
