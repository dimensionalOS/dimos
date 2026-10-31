import type { Context } from "@earendil-works/chord/context";
import type { Conversation, DocumentReader } from "@earendil-works/pi-durable";
import type { Facts } from "../../shared/protocol/runtime/context.ts";
import { DimosInputEntry } from "../inputs/source.ts";
import { applyChange } from "./facts.ts";
import { readBaseline } from "./baseline.ts";

/** The baseline folded with the change of every `dimos.input` entry in the active context, derived on each call. */
export async function toldFacts(conversation: Conversation, reader: DocumentReader, context: Context): Promise<Facts> {
  let told = (await readBaseline(reader, conversation.id, context)).facts;
  for (const entry of (await conversation.context(context)).entries) {
    if (DimosInputEntry.is(entry) && entry.data.change !== null) told = applyChange(told, entry.data.change);
  }
  return told;
}
