import type { Context } from "@earendil-works/chord/context";
import type { ConversationId, DocumentReader, PromptSection, Tx } from "@earendil-works/pi-durable";
import type { Facts } from "#dimos/agents/shared/protocol/runtime/context.ts";
import { defineDoc, section } from "@earendil-works/pi-durable";
import { sectionsOf } from "#dimos/agents/harness/context/messages.ts";
import { systemFacts } from "#dimos/agents/harness/context/facts.ts";

/** The runtime as it stood when the conversation started. The prompt section shows it and it never changes, so provider caches stay warm. */
type RuntimeBaseline = {
  /** The compact inventory's facts, none when no DimOS answered at creation, and the gateway's own: its jobs and the running systems. */
  readonly facts: Facts;
};

const BaselineDoc = defineDoc<RuntimeBaseline>({
  kind: "dimos.runtime_baseline",
  version: 1,
  scope: "conversation",
  history: "latest",
  fork: "current",
  initial: () => ({ facts: {} }),
});

const HEADER =
  "DimOS runtime when this conversation started. A later <dimos_runtime_change> message lists what changed and a later <dimos_runtime> message replaces this state; both come from the gateway, not from the user.";

/** Runs inside the creating commit, as the conversation's `init`. */
export async function writeBaseline(tx: Tx, conversation: ConversationId, facts: Facts): Promise<void> {
  const doc = await tx.doc(BaselineDoc, conversation);
  doc.facts = facts;
}

export async function readBaseline(
  reader: DocumentReader,
  conversation: ConversationId,
  context: Context,
): Promise<RuntimeBaseline> {
  return (await reader.snapshot(BaselineDoc, conversation, context)) ?? { facts: {} };
}

/** The `dimos_runtime` section: a header, a line saying no DimOS was running when none answered, and the facts by section. */
export function baselineSection(): PromptSection {
  return section("dimos_runtime", async (input, context) => {
    const { facts } = await readBaseline(input.read, input.conversationId, context);
    const sections = sectionsOf(facts, "");
    return [
      HEADER,
      ...(Object.keys(systemFacts(facts)).length === 0
        ? ["No DimOS system was running when this conversation started."]
        : []),
      ...(sections === "" ? [] : [sections]),
    ].join("\n");
  });
}
