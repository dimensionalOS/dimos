import type { ContextBudget } from "./budget.ts";
import type { ConversationId } from "@earendil-works/pi-durable";
import type { LimitRecord } from "../../shared/protocol/tools/limit.ts";
import { createHash } from "node:crypto";
import { dirname, join } from "node:path";
import { mkdir, writeFile } from "node:fs/promises";

type LimitRequest = {
  readonly conversation: ConversationId;
  /** The call's `max_chars`; null takes the default, as every input does. */
  readonly requested: number | null;
  /**
   * Names the spill file: the same name gives the same file, so a hook that runs again rewrites it. Deterministic
   * (tool call id, request id); the file is a digest of it under the conversation's directory.
   */
  readonly spillName: string;
  /** A file that already holds the whole text, such as bash's `fullOutputPath` or the file `read` read; nothing is copied then. */
  readonly fullTextAt: string | null;
  /** The argument the notice tells the model to pass, or null for inputs, which take no arguments. */
  readonly argument: "max_chars" | null;
};

type Limited = {
  readonly text: string;
  readonly record: LimitRecord | null;
};

type LimiterOptions = {
  /** `~/.local/state/dimos/agent/tool-results/`. */
  readonly dir: string;
  readonly budget: ContextBudget;
  /** The limit of a call that passes no `max_chars`, and of every input; the context left never clamps below it. */
  readonly defaultMaxChars: number;
};

const count = new Intl.NumberFormat("en-US");

/**
 * The one `max_chars` cut, for tool results and inputs alike: clamps a limit above `defaultMaxChars` to the
 * conversation's context left, never below the default, keeps the start and end of text over it around a marker, e.g.
 * `[… 46,213 of 48,213 chars cut. Full: PATH. Pass max_chars for more. …]`, and saves the whole. A cut that would not
 * shorten the text is skipped.
 */
export class OutputLimiter {
  private readonly options: LimiterOptions;

  constructor(options: LimiterOptions) {
    this.options = options;
  }

  async limit(text: string, request: LimitRequest): Promise<Limited> {
    const requested = Math.max(1, request.requested ?? this.options.defaultMaxChars);
    const remaining = this.options.budget.remainingChars(request.conversation);
    const maxChars =
      remaining === null ? requested : Math.min(requested, Math.max(this.options.defaultMaxChars, remaining));
    if (text.length <= maxChars) return { text, record: null };
    const clampedFrom = maxChars < requested ? requested : null;
    const fullText = request.fullTextAt ?? this.spillPath(request.conversation, request.spillName);
    const head = Math.floor(maxChars / 2);
    const tail = maxChars - head;
    const marker = [
      `[… ${count.format(text.length - maxChars)} of ${count.format(text.length)} chars cut.`,
      `Full: ${fullText}.`,
      ...(clampedFrom === null ? [] : [`Only ${count.format(maxChars)} fit the context left.`]),
      ...(request.argument === null || clampedFrom !== null ? [] : [`Pass ${request.argument} for more.`]),
      "…]",
    ].join(" ");
    const cut = `${text.slice(0, head)}\n${marker}\n${tail === 0 ? "" : text.slice(-tail)}`;
    if (cut.length >= text.length) return { text, record: null };
    if (request.fullTextAt === null) await this.spill(fullText, text);
    return { text: cut, record: { shown: maxChars, total: text.length, fullText, maxChars, clampedFrom } };
  }

  private spillPath(conversation: ConversationId, name: string): string {
    const digest = createHash("sha256").update(name).digest("hex").slice(0, 12);
    return join(this.options.dir, String(conversation), `${digest}.txt`);
  }

  private async spill(path: string, text: string): Promise<void> {
    await mkdir(dirname(path), { recursive: true, mode: 0o700 });
    await writeFile(path, text, { mode: 0o600 });
  }
}
