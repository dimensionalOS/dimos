import type { TUI } from "@earendil-works/pi-tui";
import type { ToolCall } from "@earendil-works/pi-ai";
import type { ToolCard } from "./base.ts";
import type { ToolResult } from "../../../shared/protocol/conversation/state.ts";
import { Container, Spacer, Text } from "@earendil-works/pi-tui";
import {
  ToolExecutionComponent,
  createBashToolDefinition,
  createEditToolDefinition,
  createReadToolDefinition,
  createWriteToolDefinition,
} from "@earendil-works/pi-coding-agent";
import { accent, muted } from "../style.ts";
import { clean } from "../../../shared/view-models/text.ts";
import { movedToBackground } from "../../../shared/view-models/conversation/tools.ts";
import { needsImageSlices } from "../lib/terminal-image.ts";

/** Tools whose card is Pi's own renderer; the rest show a one-line summary until expanded. */
const PI_RENDERERS = {
  bash: createBashToolDefinition,
  read: createReadToolDefinition,
  edit: createEditToolDefinition,
  write: createWriteToolDefinition,
} as const;

function piRenderer(name: string, cwd: string) {
  return Object.hasOwn(PI_RENDERERS, name) ? PI_RENDERERS[name as keyof typeof PI_RENDERERS](cwd) : undefined;
}

function textOf(result: ToolResult): string {
  return result.content.flatMap((block) => (block.type === "text" ? [block.text] : [])).join("\n");
}

/**
 * Pi's ToolExecutionComponent with the read, edit and write renderers of pi-coding-agent, and a one-line card for the
 * rest. Expanded bash also uses it, with the `truncation` and `fullOutputPath` details bash's settle step stored.
 */
export class PiToolCard implements ToolCard {
  readonly call: ToolCall;
  private readonly component: ToolExecutionComponent;
  private readonly builtin: boolean;
  private readonly block = new Container();
  private result: ToolResult | null = null;
  private running = "";
  private expanded = false;

  constructor(call: ToolCall, tui: TUI, cwd: string) {
    this.call = call;
    const definition = piRenderer(call.name, cwd);
    this.builtin = definition !== undefined;
    this.component = new ToolExecutionComponent(
      call.name,
      call.id,
      call.arguments,
      { showImages: !needsImageSlices() },
      definition,
      tui,
      cwd,
    );
    this.component.markExecutionStarted();
    this.component.setArgsComplete();
    this.redraw();
  }

  output(text: string): void {
    this.running = text;
    this.component.updateResult({ content: [{ type: "text", text }], isError: false }, true);
    this.redraw();
  }

  end(result: ToolResult): void {
    this.result = result;
    this.component.updateResult(
      {
        content: result.content,
        details: result.details.tool === null ? undefined : result.details.details,
        isError: result.isError,
        durationMs: result.durationMs,
      },
      false,
    );
    this.redraw();
  }

  setExpanded(expanded: boolean): void {
    this.expanded = expanded;
    this.component.setExpanded(expanded);
    this.redraw();
  }

  render(width: number): string[] {
    return this.block.render(width);
  }

  invalidate(): void {
    this.block.invalidate();
  }

  dispose(): void {
    // Pi's renderers redraw a partial result every second until a final one arrives.
    if (this.result === null) this.component.updateResult({ content: [], isError: false }, false);
  }

  private redraw(): void {
    this.block.clear();
    if (this.builtin || this.expanded) {
      this.block.addChild(this.component);
      return;
    }
    const state =
      this.result === null
        ? " · running"
        : movedToBackground(this.result)
          ? " · moved to the background"
          : this.result.isError
            ? " · failed"
            : " · done";
    const summary = this.result === null ? this.running : textOf(this.result);
    this.block.addChild(new Spacer(1));
    this.block.addChild(new Text(accent(" ↳ ") + clean(this.call.name) + muted(state), 0, 0));
    const firstLine = summary.split("\n").find((line) => line.trim() !== "");
    if (firstLine !== undefined) this.block.addChild(new Text(muted(` ${clean(firstLine).slice(0, 220)}`), 0, 0));
  }
}
