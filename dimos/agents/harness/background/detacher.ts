import type { AgentTool } from "../tools/base.ts";
import type { AgentToolName, Background } from "../../shared/protocol/tools/details.ts";
import type { CallId, Detachable } from "../../shared/protocol/conversation/calls.ts";
import type { Context } from "@earendil-works/chord/context";
import type {
  ConversationId,
  Extension,
  JsonObject,
  Task,
  TaskId,
  ToolDiagnostic,
  ToolExecutionApi,
  ToolExecutionResult,
  ToolRegistration,
} from "@earendil-works/pi-durable";
import type { ImageContent } from "@earendil-works/pi-ai";
import type { InputGate } from "../inputs/gate.ts";
import type { Live } from "../../shared/protocol/status.ts";
import type { RenderDetails } from "../../shared/protocol/media.ts";
import { MovedSchema } from "../../shared/protocol/tools/details.ts";
import { Status } from "../../shared/protocol/status.ts";
import { Type } from "typebox";
import { awaitWithContext, withAbortSignal, withoutAbortSignal } from "@earendil-works/chord/context";
import { defineExtension, defineTask } from "@earendil-works/pi-durable";
import { epochMs } from "../../shared/protocol/time.ts";
import { renderOf } from "../tools/details.ts";

export type DetachResult = { readonly kind: "detached"; readonly task: TaskId } | { readonly kind: "not_running" };

/** How background work ended, as its notice tells the model. */
type Ended =
  | {
      readonly end: "finished" | "failed";
      readonly text: string;
      readonly images: readonly ImageContent[];
      readonly render: RenderDetails | null;
    }
  /** The gateway restarted while the work ran, and the work died with it. */
  | { readonly end: "interrupted" };

type BackgroundInput = { readonly toolTask: TaskId; readonly call: CallId; readonly tool: string };

type BackgroundTask = Task<BackgroundInput, { readonly phase: "waiting" }, { readonly end: Ended["end"] }, object>;

/** A running call `detach` can move. `letGo` answers the task that took its work, or null when the work finished first. */
type Running = { readonly detachable: Detachable; readonly letGo: () => Promise<TaskId | null> };

/** Moved work, kept in memory only: a restarted gateway finds none and reports the call `interrupted`. */
type Adopted = { readonly ended: Promise<Ended>; readonly cancel: () => void };

/**
 * Hands running tool calls to background tasks, which pi-durable has no detach for. A wrapped call's work runs on its
 * own abort controller with an `api` it can keep using after the call returns. Asked to detach, the call creates a
 * `dimos.background` task owned by the conversation that adopts the work, and returns at once; the task then submits
 * the work's result through InputGate as a `background` input.
 */
export class Detacher {
  private readonly inputs: InputGate;
  /** Where a notice that could not be submitted goes; its task still ends. */
  private readonly report: (error: unknown) => void;
  private readonly task: BackgroundTask;
  private readonly running = new Map<ConversationId, Map<CallId, Running>>();
  /** Only conversations someone asked `of`; the others have nobody to tell. */
  private readonly statuses = new Map<ConversationId, Status<readonly Detachable[]>>();
  private readonly adopted = new Map<TaskId, Adopted>();

  constructor(inputs: InputGate, report: (error: unknown) => void) {
    this.inputs = inputs;
    this.report = report;
    this.task = defineTask<BackgroundInput, { readonly phase: "waiting" }, { readonly end: Ended["end"] }>({
      name: "dimos.background",
      version: 1,
      initial: () => ({ phase: "waiting" }),
      phases: {
        waiting: async (task, runtime, context) => {
          const { toolTask, call, tool } = task.input;
          const work = this.adopted.get(toolTask);
          const ended: Ended = work === undefined ? { end: "interrupted" } : await this.until(work, context);
          this.adopted.delete(toolTask);
          try {
            await this.inputs.submit(
              runtime.conversationId,
              {
                source: { kind: "background", task: runtime.taskId, call, tool, end: ended.end },
                text: noticeText(tool, call, ended),
                images: ended.end === "interrupted" ? [] : ended.images,
                render: ended.end === "interrupted" ? null : ended.render,
                whenBusy: "followUp",
              },
              context,
            );
          } catch (error) {
            this.report(new Error(`The notice of ${tool} call ${call} was not submitted: ${messageOf(error)}`));
          }
          await runtime.commit(
            () => ({ status: "terminal", outcome: { status: "completed", result: { end: ended.end } } }),
            context,
          );
        },
      },
      abort: async (task, runtime, context) => {
        this.adopted.get(task.input.toolTask)?.cancel();
        this.adopted.delete(task.input.toolTask);
        await runtime.commit(() => ({ status: "terminal", outcome: { status: "aborted" } }), context);
      },
    });
  }

  /** The conversation's calls that can move now, oldest first. */
  of(conversation: ConversationId): Live<readonly Detachable[]> {
    const known = this.statuses.get(conversation);
    if (known !== undefined) return known;
    const created = new Status(this.detachables(conversation), (error) => {
      throw error;
    });
    this.statuses.set(conversation, created);
    return created;
  }

  async detach(conversation: ConversationId, call: CallId, context: Context): Promise<DetachResult> {
    const running = this.running.get(conversation)?.get(call);
    if (running === undefined) return { kind: "not_running" };
    this.release(conversation, call);
    const task = await awaitWithContext(running.letGo(), context);
    return task === null ? { kind: "not_running" } : { kind: "detached", task };
  }

  /**
   * The tool's registration with `execute` wrapped so a running call can move; `toolsExtension` applies it per tool
   * policy. A moved call's structured output is its background handle, so a declared schema admits that too. Its work's
   * result goes through the tool's settle step before the notice, as a call that stays does in the afterTool hook.
   */
  wrap(tool: AgentTool<AgentToolName>): ToolRegistration {
    const schema = tool.registration.structuredOutputSchema;
    return {
      ...tool.registration,
      ...(schema === undefined ? {} : { structuredOutputSchema: Type.Union([schema, MovedSchema]) }),
      execute: (args, api, context) => this.execute(tool, args, api, context),
    };
  }

  /** The `dimos.background` task kind, installed before `resume()` so a restart finds its tasks. */
  extension(): Extension {
    return defineExtension({ name: "dimos-background", tasks: [this.task] });
  }

  private async execute(
    tool: AgentTool<AgentToolName>,
    args: unknown,
    api: ToolExecutionApi,
    context: Context,
  ): Promise<ToolExecutionResult> {
    const { registration, settle } = tool;
    const conversation = api.conversationId;
    const call = api.callId as CallId;
    const own = new AbortController();
    const abortWork = (): void => own.abort();
    context.abortSignal?.addEventListener("abort", abortWork, { once: true });
    const held = new HeldOutput(api);
    const workContext = withAbortSignal(own.signal, withoutAbortSignal(context));
    const work = registration.execute(args, held.api, workContext);
    let answer: (task: TaskId | null) => void = () => undefined;
    const asked = new Promise<void>((resolve) => {
      this.hold(conversation, call, {
        detachable: { call, tool: registration.name, since: epochMs(Date.now()) },
        letGo: () =>
          new Promise<TaskId | null>((reply) => {
            answer = reply;
            resolve();
          }),
      });
    });
    const first = await Promise.race([
      work.then(
        () => "finished" as const,
        () => "finished" as const,
      ),
      asked.then(() => "asked" as const),
    ]);
    this.release(conversation, call);
    context.abortSignal?.removeEventListener("abort", abortWork);
    if (first === "finished") {
      answer(null);
      return work;
    }
    held.move();
    const hookCall = {
      type: "toolCall" as const,
      id: api.callId,
      name: registration.name,
      arguments: args as JsonObject,
    };
    const ended = work
      .then((result) => (settle === null ? held.final(result) : settle(held.final(result), hookCall, workContext)))
      .then(endedOf, (error: unknown) => held.failed(error));
    this.adopted.set(api.taskId, { ended, cancel: () => own.abort() });
    try {
      const task = await api.commit(
        (tx) =>
          tx.createTask(
            this.task,
            { toolTask: api.taskId, call, tool: registration.name },
            { ownership: { kind: "conversation" }, conversationId: conversation, background: true },
          ),
        context,
      );
      answer(task);
      const background: Background = { task };
      return {
        output: [{ type: "text", text: "Moved to the background. Its result arrives later as a message." }],
        ...(registration.structuredOutputSchema === undefined ? {} : { structuredOutput: { background } }),
        details: { background },
      };
    } catch (error) {
      answer(null);
      this.adopted.delete(api.taskId);
      own.abort();
      throw error;
    }
  }

  /** The adopted work's end; stopping the task, as the Harness does when it closes, stops the work. */
  private async until(work: Adopted, context: Context): Promise<Ended> {
    context.abortSignal?.addEventListener("abort", work.cancel, { once: true });
    try {
      return await awaitWithContext(work.ended, context);
    } finally {
      context.abortSignal?.removeEventListener("abort", work.cancel);
    }
  }

  private detachables(conversation: ConversationId): readonly Detachable[] {
    return [...(this.running.get(conversation)?.values() ?? [])].map((running) => running.detachable);
  }

  private hold(conversation: ConversationId, call: CallId, running: Running): void {
    const calls = this.running.get(conversation) ?? new Map<CallId, Running>();
    calls.set(call, running);
    this.running.set(conversation, calls);
    this.notify(conversation);
  }

  private release(conversation: ConversationId, call: CallId): void {
    const calls = this.running.get(conversation);
    if (calls === undefined || !calls.delete(call)) return;
    if (calls.size === 0) this.running.delete(conversation);
    this.notify(conversation);
  }

  private notify(conversation: ConversationId): void {
    this.statuses.get(conversation)?.set(this.detachables(conversation));
  }
}

/** The tool's `api`: the call's own until the work moves, then a local tail of its output and its remarks. */
class HeldOutput {
  readonly api: ToolExecutionApi;
  private readonly call: ToolExecutionApi;
  private readonly decoder = new TextDecoder();
  private readonly diagnostics: ToolDiagnostic[] = [];
  private moved = false;
  private tail = "";
  private truncated = false;

  constructor(call: ToolExecutionApi) {
    this.call = call;
    this.api = {
      ...call,
      output: (chunk, skipped) => {
        if (this.moved) this.keep(chunk, skipped !== undefined);
        else call.output(chunk, skipped);
      },
      retainedOutput: () => (this.moved ? { text: this.tail, truncated: this.truncated } : call.retainedOutput()),
      diagnostic: (diagnostic) => {
        if (this.moved) this.diagnostics.push(diagnostic);
        else call.diagnostic(diagnostic);
      },
      details: async (value, context) => {
        if (!this.moved) await call.details(value, context);
      },
    };
  }

  /** From now on the call is over: output and remarks stay here. */
  move(): void {
    const retained = this.call.retainedOutput();
    this.tail = retained.text;
    this.truncated = retained.truncated;
    this.moved = true;
  }

  /** The result as the Harness gives it to settle: the kept tail when it has no output, the remarks before its own. */
  final(result: ToolExecutionResult): ToolExecutionResult {
    return {
      ...result,
      output: result.output ?? (this.tail === "" ? [] : [{ type: "text", text: this.tail }]),
      diagnostics: [...this.diagnostics, ...(result.diagnostics ?? [])],
    };
  }

  failed(error: unknown): Ended {
    const lines = [this.tail, ...this.diagnostics.map((diagnostic) => diagnostic.message), messageOf(error)];
    return { end: "failed", text: lines.filter((line) => line !== "").join("\n"), images: [], render: null };
  }

  /** The tail the call would have kept: at most its window's size when it has one, as the Harness keeps it. */
  private keep(chunk: string | Uint8Array, skipped: boolean): void {
    this.tail += typeof chunk === "string" ? chunk : this.decoder.decode(chunk, { stream: true });
    if (skipped) this.truncated = true;
    const window = this.call.outputWindow;
    if (window !== undefined && this.tail.length > window.maxBytes) {
      this.tail = this.tail.slice(-window.maxBytes);
      this.truncated = true;
    }
  }
}

function endedOf(result: ToolExecutionResult): Ended {
  const content = result.output ?? [];
  const remarks = (result.diagnostics ?? []).map((diagnostic) => diagnostic.message);
  return {
    end: result.isError === true ? "failed" : "finished",
    text: [...content.flatMap((block) => (block.type === "text" ? [block.text] : [])), ...remarks].join("\n"),
    images: content.flatMap((block) => (block.type === "image" ? [block] : [])),
    render: renderOf(result.details),
  };
}

function noticeText(tool: string, call: CallId, ended: Ended): string {
  const which = `[Background] ${tool} call ${call}`;
  switch (ended.end) {
    case "interrupted":
      return `${which} was lost: the gateway restarted while it ran.`;
    case "finished":
    case "failed":
      return ended.text === "" ? `${which} ${ended.end} with no output.` : `${which} ${ended.end}:\n${ended.text}`;
  }
}

function messageOf(error: unknown): string {
  return error instanceof Error ? error.message : String(error);
}
