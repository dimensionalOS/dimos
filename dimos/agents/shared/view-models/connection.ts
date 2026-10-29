import type { Connection } from "#dimos/agents/sdk/main.ts";
import type { Segment } from "#dimos/agents/shared/view-models/levels.ts";
import { clockSeconds } from "#dimos/agents/shared/view-models/time.ts";
import { refusalText } from "#dimos/agents/shared/view-models/outcome.ts";

/** This client's own link to the gateway: "gateway ⟳ reconnecting (attempt 2) · showing state from 14:06:12". */
export function connectionSegment(connection: Connection): Segment {
  switch (connection.state) {
    case "connecting":
      return { text: `gateway ◌ connecting (attempt ${connection.attempt})`, level: "ok" };
    case "live":
      return { text: `gateway ● live since ${clockSeconds(connection.since)}`, level: "ok" };
    case "reconnecting":
      return {
        text: `gateway ⟳ reconnecting (attempt ${connection.attempt}) · showing state from ${clockSeconds(connection.since)}`,
        level: "warn",
      };
    case "refused":
      return { text: `gateway ✖ ${refusalText(connection.refusal)}`, level: "critical" };
  }
}
