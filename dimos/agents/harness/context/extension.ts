import type { ConversationId, Extension } from "@earendil-works/pi-durable";
import type { InputGate } from "#dimos/agents/harness/inputs/gate.ts";
import { GenerationTask, ToolTask, defineExtension, hook } from "@earendil-works/pi-durable";
import { baselineSection } from "#dimos/agents/harness/context/baseline.ts";

type ContextExtensionOptions = {
  readonly inputs: InputGate;
  /** Whether a tool may change the running system: every tool not declared replay-safe. From harness/tools/extension.ts. */
  readonly mayChangeSystem: (toolName: string) => boolean;
};

/** The `dimos_runtime` section, and a runtime steer after a tool round in which a tool that may change the system ran. */
export function contextExtension(options: ContextExtensionOptions): Extension {
  const changedIn = new Set<ConversationId>();
  return defineExtension({
    name: "dimos-runtime",
    sections: [baselineSection()],
    hooks: [
      hook(ToolTask, {
        afterTool: (call, _result, api) => {
          if (options.mayChangeSystem(call.name)) changedIn.add(api.conversationId);
          return undefined;
        },
      }),
      hook(GenerationTask, {
        afterTools: async (assistant, _results, api, context) => {
          if (!changedIn.delete(api.conversationId)) return;
          await options.inputs.submit(
            api.conversationId,
            { source: { kind: "runtime", after: assistant }, text: "", images: [], render: null, whenBusy: "steer" },
            context,
          );
        },
      }),
    ],
  });
}
