import type { ConversationId, Extension, ModelRef } from "@earendil-works/pi-durable";
import type { Models } from "@earendil-works/pi-ai";
import { GenerationTask, defineExtension, hook } from "@earendil-works/pi-durable";

type ContextSize = {
  /** Prompt and answer tokens of the last response. */
  readonly tokens: number;
  readonly model: ModelRef;
};

/** Characters still free in each conversation's context window, from its last response. */
export class ContextBudget {
  private readonly models: Models;
  private readonly sizes = new Map<ConversationId, ContextSize>();

  constructor(models: Models) {
    this.models = models;
  }

  /** The extension whose `afterResponse` hook records each conversation's context size. */
  extension(): Extension {
    return defineExtension({
      name: "dimos-context-budget",
      hooks: [
        hook(GenerationTask, {
          afterResponse: (message, api) => {
            const usage = message.usage;
            this.sizes.set(api.conversationId, {
              tokens: usage.input + usage.cacheRead + usage.cacheWrite + usage.output,
              model: { provider: message.provider, modelId: message.model },
            });
          },
        }),
      ],
    });
  }

  /** Null when nothing is known yet; otherwise (contextWindow − reserve − used tokens) × 3 characters. */
  remainingChars(conversation: ConversationId): number | null {
    const size = this.sizes.get(conversation);
    if (size === undefined) return null;
    const model = this.models.getModel(size.model.provider, size.model.modelId);
    if (model === undefined) return null;
    const reserveTokens = 16_384;
    const charsPerToken = 3;
    return Math.max(0, model.contextWindow - reserveTokens - size.tokens) * charsPerToken;
  }
}
