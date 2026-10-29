import type { DimosInput, InputSource } from "../../shared/protocol/conversation/input.ts";
import type { Entry } from "@earendil-works/pi-durable";
import { defineEntry } from "@earendil-works/pi-durable";

export const DimosInputEntry: Entry<DimosInput> = defineEntry<DimosInput>("dimos.input");

/**
 * The pi-durable request id, unique per conversation: `user:<uuid>`, `dimos-event:<uuid>`,
 * `job:<name>:<due>`, `job:<name>:call:<id>` or `job:<name>:request:<id>`, `runtime:<assistant entry>`,
 * `background:<task>`.
 */
export function requestIdFor(source: InputSource): string {
  switch (source.kind) {
    case "user":
      return `user:${source.request}`;
    case "event":
      return `dimos-event:${source.event.id}`;
    case "job":
      switch (source.run.kind) {
        case "scheduled":
          return `job:${source.job}:${source.run.due}`;
        case "manual":
          return `job:${source.job}:call:${source.run.toolCall}`;
        case "requested":
          return `job:${source.job}:request:${source.run.request}`;
      }
    case "runtime":
      return `runtime:${source.after}`;
    case "background":
      return `background:${source.task}`;
  }
}
