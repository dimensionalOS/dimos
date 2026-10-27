import type { ConversationId, EntryId, LiveState, SubmissionId } from "@earendil-works/pi-durable";
import type { AssistantMessage } from "@earendil-works/pi-ai";
import type { Ref } from "@earendil-works/chord";
import type { Client } from "../../sdk/main.ts";
import { connect } from "../../sdk/main.ts";
import { unixSocket } from "../../sdk/node.ts";
import { owner, refusalOfFailure } from "../../sdk/failures.ts";
import { retryByRequestId } from "../../sdk/retry.ts";
import { ConversationContract } from "../../shared/protocol/conversation/contract.ts";
import { GatewayContract } from "../../shared/protocol/gateway/contract.ts";
import type { EpochMs } from "../../shared/protocol/time.ts";
import { until } from "../../sdk/until.ts";
import { newActionRequest, newUserRequest } from "../../shared/protocol/actions/request.ts";
import type { InputOutcome, Transcript, TranscriptItem } from "../../shared/protocol/conversation/state.ts";
import type { UserRequestId } from "../../shared/protocol/conversation/input.ts";
import { refusalText } from "../../shared/view-models/outcome.ts";
import type { Command, CommandContext } from "./base.ts";

function textOf(content: readonly { readonly type: string; readonly text?: string }[]): string {
  return content.flatMap((block) => (block.type === "text" && block.text !== undefined ? [block.text] : [])).join("");
}

/** The transcript items after one request's input, each once, so a busy conversation's earlier items stay out. */
class RequestItems {
  private readonly seen = new Set<EntryId>();
  private readonly request: UserRequestId;
  private placed = false;

  constructor(request: UserRequestId) {
    this.request = request;
  }

  next(transcript: Transcript): readonly TranscriptItem[] {
    const items: TranscriptItem[] = [];
    for (const item of transcript.items) {
      if (this.seen.has(item.entry)) continue;
      this.seen.add(item.entry);
      if (this.placed) items.push(item);
      else
        this.placed =
          item.kind === "input" && item.input.source.kind === "user" && item.input.source.request === this.request;
    }
    return items;
  }
}

/** What `run` prints while the input runs: the answer as text, or JSON lines with `--json`. */
type RunPrinter = {
  sent(submission: SubmissionId): void;
  live(live: LiveState): void;
  transcript(transcript: Transcript): void;
  /** Called once the outcome, and for an answer the item it names, have landed. */
  outcome(outcome: InputOutcome): void;
  finishLine(): void;
};

/**
 * Writes the answer to one request as it streams: `pi.live` only while the run carries the request's submission,
 * transcript items only after its input. The partial in `pi.live` is throttled, so the text printed so far is a prefix
 * of the assistant item that lands; each item's remainder and a newline close it.
 */
class AnswerPrinter implements RunPrinter {
  private printed = "";
  private readonly items: RequestItems;
  private readonly seenCalls = new Set<string>();
  private submission: SubmissionId | null = null;
  atLineStart = true;

  constructor(request: UserRequestId) {
    this.items = new RequestItems(request);
  }

  sent(submission: SubmissionId): void {
    this.submission = submission;
  }

  live(live: LiveState): void {
    if (this.submission === null || !(live.run?.inputs.includes(this.submission) ?? false)) return;
    for (const slot of live.tools ?? []) {
      if (this.seenCalls.has(slot.callId)) continue;
      this.seenCalls.add(slot.callId);
      process.stderr.write(`[${slot.name}]\n`);
    }
    const message = live.generation?.message;
    if (message === undefined) return;
    const text = textOf(message.content);
    if (text === "") return;
    if (!text.startsWith(this.printed)) {
      this.finishLine();
      this.printed = "";
    }
    this.write(text.slice(this.printed.length));
    this.printed = text;
  }

  transcript(transcript: Transcript): void {
    for (const item of this.items.next(transcript)) if (item.kind === "assistant") this.answer(item.message);
  }

  outcome(outcome: InputOutcome): void {
    this.finishLine();
    if (outcome.status === "unanswered") process.stderr.write(`Unanswered: ${outcome.reason}\n`);
  }

  finishLine(): void {
    if (this.atLineStart) return;
    process.stdout.write("\n");
    this.atLineStart = true;
  }

  private answer(message: AssistantMessage): void {
    const full = textOf(message.content);
    if (full === "") return;
    this.write(full.startsWith(this.printed) ? full.slice(this.printed.length) : full);
    this.printed = "";
    this.finishLine();
  }

  private write(text: string): void {
    if (text === "") return;
    process.stdout.write(text);
    this.atLineStart = text.endsWith("\n");
  }
}

/** One `{"item": ...}` line per transcript item after the request's input, as it lands, then one `{"outcome": ...}`. */
class JsonLinesPrinter implements RunPrinter {
  private readonly items: RequestItems;

  constructor(request: UserRequestId) {
    this.items = new RequestItems(request);
  }

  sent(): void {}

  live(): void {}

  transcript(transcript: Transcript): void {
    for (const item of this.items.next(transcript)) process.stdout.write(`${JSON.stringify({ item })}\n`);
  }

  outcome(outcome: InputOutcome): void {
    process.stdout.write(`${JSON.stringify({ outcome })}\n`);
  }

  finishLine(): void {}
}

async function shownConversation(
  client: Client,
  gateway: Ref<typeof GatewayContract>,
  options: CommandContext,
): Promise<{ readonly conversation: Ref<typeof ConversationContract>; readonly id: ConversationId } | string> {
  let id = options.conversation;
  if (id === null) {
    const created = await retryByRequestId(client, { request: newActionRequest(), cwd: options.cwd }, (sameArgs) =>
      owner(gateway).createConversation(sameArgs),
    );
    if (!created.ok) return refusalText(created.refusal);
    id = created.value.conversation;
  }
  const conversation = await Promise.race([
    client.host.get(ConversationContract, { conversation: id, role: "owner" }),
    client.ended,
  ]);
  if (conversation.variant !== "owner") {
    conversation.release();
    return refusalText({ code: "viewing", conversation: id });
  }
  return { conversation, id };
}

/**
 * `dimos agent run PROMPT`: one follow-up into a new or given conversation, its streamed text on stdout, `[tool]`
 * lines on stderr, and its outcome as the exit code: 0 when answered, 1 when unanswered or refused. With `--json`,
 * stdout carries JsonLinesPrinter's lines instead. A reconnect resends `outcomes` whole, so the outcome cannot be
 * missed.
 */
export const printRun: Command = async (options) => {
  const prompt = options.args.join(" ").trim();
  if (prompt === "") throw new Error("dimos agent run needs a PROMPT");
  await options.startGateway();
  try {
    const client = await connect(unixSocket({ path: options.paths.socket, label: "print" }));
    let gateway: Ref<typeof GatewayContract> | undefined;
    try {
      gateway = await Promise.race([client.host.get(GatewayContract, {}), client.ended]);
      const shown = await shownConversation(client, gateway, options);
      if (typeof shown === "string") {
        process.stderr.write(`${shown}\n`);
        return 1;
      }
      const { conversation, id } = shown;
      try {
        process.stderr.write(`Conversation ${id}\n`);
        const request = newUserRequest();
        const printer: RunPrinter = options.json ? new JsonLinesPrinter(request) : new AnswerPrinter(request);
        const stops = [
          conversation.live.subscribe(() => printer.live(conversation.live.value)),
          conversation.transcript.subscribe(() => printer.transcript(conversation.transcript.value)),
        ];
        try {
          const sent = await retryByRequestId(
            client,
            { request, text: prompt, images: [], whenBusy: "followUp" as const },
            (sameArgs) => owner(conversation).send(sameArgs),
          );
          if (!sent.ok) {
            process.stderr.write(`${refusalText(sent.refusal)}\n`);
            return 1;
          }
          printer.sent(sent.value.submission);
          printer.live(conversation.live.value);
          let ended: InputOutcome;
          try {
            ended = await Promise.race([
              until(conversation.outcomes, (rows) => rows.find((row) => row.submission === sent.value.submission)),
              client.ended,
            ]);
            if (ended.status === "done") {
              // Members arrive independently: the outcome can land before the transcript carries the answer it names.
              const answer = ended.answer;
              await Promise.race([
                until(conversation.transcript, (transcript) =>
                  transcript.items.some((item) => item.entry === answer) ? transcript : undefined,
                ),
                client.ended,
              ]);
            }
          } catch (error) {
            printer.finishLine();
            process.stderr.write(
              `The gateway disconnected before the answer finished: ${refusalText(refusalOfFailure(error, Date.now() as EpochMs))}\n`,
            );
            return 1;
          }
          printer.transcript(conversation.transcript.value);
          printer.outcome(ended);
          return ended.status === "done" ? 0 : 1;
        } finally {
          for (const stop of stops) stop();
          printer.finishLine();
        }
      } finally {
        conversation.release();
      }
    } finally {
      gateway?.release();
      await client.close();
    }
  } catch (error) {
    process.stderr.write(`${refusalText(refusalOfFailure(error, Date.now() as EpochMs))}\n`);
    return 1;
  }
};
