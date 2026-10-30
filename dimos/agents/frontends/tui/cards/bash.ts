import type { TUI } from "@earendil-works/pi-tui";
import type { ToolCall } from "@earendil-works/pi-ai";
import type { ToolCard } from "./base.ts";
import type { ToolResult } from "../../../shared/protocol/conversation/state.ts";
import { Loader, Text, truncateToWidth } from "@earendil-works/pi-tui";
import { PiToolCard } from "./pi-tool.ts";
import { accent, amber, muted } from "../style.ts";
import { clean } from "../../../shared/view-models/text.ts";
import { movedToBackground } from "../../../shared/view-models/conversation/tools.ts";
import { stripVTControlCharacters } from "node:util";

const STAGE = /^(Resolved|Prepared|Installed|Uninstalled|Downloading|Building|Audited)\b/;

function commandOf(call: ToolCall): string {
  const command = call.arguments.command;
  return typeof command === "string" ? command : "bash";
}

/**
 * `$ command`, a loader with uv stage detection, the last 12 lines while running and 3 when done; `/expand` shows
 * Pi's full bash card. Moved from dimcode bash-log.ts.
 */
export class BashCard implements ToolCard {
  readonly call: ToolCall;
  private readonly loader: Loader;
  private readonly full: PiToolCard;
  private lines: string[] = [];
  private started: number | null = null;
  private elapsed: number | null = null;
  private running = false;
  private failed = false;
  private moved = false;
  private label = "";
  private expanded = false;

  constructor(call: ToolCall, tui: TUI, cwd: string) {
    this.call = call;
    this.loader = new Loader(tui, accent, muted, "Running");
    this.loader.stop();
    this.full = new PiToolCard(call, tui, cwd);
    this.update("", true, false);
  }

  output(text: string): void {
    this.full.output(text);
    this.update(text, true, false);
  }

  end(result: ToolResult): void {
    this.full.end(result);
    this.moved = movedToBackground(result);
    this.update(
      result.content.flatMap((block) => (block.type === "text" ? [block.text] : [])).join("\n"),
      false,
      !this.moved && (result.isError || (result.details.tool === "bash" && result.details.details.exitCode !== 0)),
    );
  }

  setExpanded(expanded: boolean): void {
    this.expanded = expanded;
    this.full.setExpanded(expanded);
  }

  render(width: number): string[] {
    if (this.expanded) return this.full.render(width);
    const elapsed = this.running && this.started !== null ? (Date.now() - this.started) / 1000 : this.elapsed;
    const tail = this.lines.slice(this.running ? -12 : -3);
    const stage = this.lines.findLast((line) => STAGE.test(line.trim()));
    const stageVisible = stage !== undefined && tail.includes(stage);
    const status = this.running
      ? stageVisible
        ? "Running"
        : (stage ?? "Waiting for command output")
      : this.moved
        ? "In the background"
        : this.failed
          ? "Failed"
          : `Done${stage === undefined || stageVisible ? "" : ` · ${stage}`}`;
    const label = status + (elapsed === null ? "" : ` · ${elapsed.toFixed(1)}s`);
    if (this.running && label !== this.label) {
      this.label = label;
      this.loader.setMessage(label);
    }
    const colour = (line: string) =>
      line.trimStart().startsWith("+") ? accent(line) : line.trimStart().startsWith("-") ? amber(line) : muted(line);
    return [
      "",
      ...new Text(accent("$ ") + clean(commandOf(this.call)), 0, 0).render(width),
      ...(this.running
        ? this.loader.render(width).filter(Boolean)
        : [truncateToWidth((this.failed ? amber : muted)(label), width)]),
      ...tail.map((line) => truncateToWidth(muted("│ ") + colour(line), width)),
      ...(this.lines.length > tail.length
        ? [
            truncateToWidth(
              muted(`  ${this.lines.length - tail.length} earlier lines · /expand for full output`),
              width,
            ),
          ]
        : []),
    ];
  }

  invalidate(): void {
    this.loader.invalidate();
    this.full.invalidate();
  }

  dispose(): void {
    this.loader.stop();
    this.full.dispose();
  }

  private update(output: string, running: boolean, failed: boolean): void {
    if (running && !this.running) {
      this.started = Date.now();
      this.loader.start();
    }
    if (!running) {
      this.loader.stop();
      if (this.running && this.started !== null) this.elapsed = (Date.now() - this.started) / 1000;
    }
    this.running = running;
    this.failed = failed;
    this.lines = stripVTControlCharacters(output)
      .split(/\r?\n|\r/)
      .map((line) => clean(line).trimEnd())
      .filter((line) => line !== "");
  }
}
