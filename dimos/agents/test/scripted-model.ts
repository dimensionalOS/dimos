import type { FauxResponseStep, MutableModels } from "@earendil-works/pi-ai";
import type { ModelRef } from "@earendil-works/pi-durable";
import { createModels, fauxProvider } from "@earendil-works/pi-ai";

type ScriptedModel = {
  readonly models: MutableModels;
  readonly model: ModelRef;
  /** Appends responses; a factory step sees the request, which is how tests assert what the model was sent. */
  script(steps: readonly FauxResponseStep[]): void;
  /** Replaces whatever is still queued. */
  reset(steps: readonly FauxResponseStep[]): void;
};

/** pi-ai's faux provider registered in a fresh Models. Tests pass `{ id }` to `fauxToolCall` for deterministic call ids. */
export function scriptedModel(): ScriptedModel {
  const faux = fauxProvider({
    provider: "faux",
    api: "faux",
    models: [{ id: "scripted", input: ["text", "image"], contextWindow: 128_000, maxTokens: 16_384 }],
  });
  const models = createModels();
  models.setProvider(faux.provider);
  return {
    models,
    model: { provider: "faux", modelId: "scripted" },
    script: (steps) => faux.appendResponses([...steps]),
    reset: (steps) => faux.setResponses([...steps]),
  };
}
