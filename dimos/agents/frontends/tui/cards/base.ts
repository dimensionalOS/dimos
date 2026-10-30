import type { Component } from "@earendil-works/pi-tui";
import type { ToolCall } from "@earendil-works/pi-ai";
import type { ToolResult } from "../../../shared/protocol/conversation/state.ts";

/** One tool call in the transcript, from its start to its result. bash.ts and pi-tool.ts implement it; choose.ts picks one. */
export interface ToolCard extends Component {
  readonly call: ToolCall;
  /** The output window so far. */
  output(text: string): void;
  /** The finished result with its parsed details. The transcript draws a `render` in them under the card. */
  end(result: ToolResult): void;
  setExpanded(expanded: boolean): void;
  /** Stops timers; the card is not drawn again. */
  dispose(): void;
}
