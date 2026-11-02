import type { Agent } from "#dimos/agents/harness/main.ts";
import type { ChildCommand } from "#dimos/agents/test/child-driver.ts";
import type { ConversationId, SubmissionId, TaskId } from "@earendil-works/pi-durable";
import type { FauxResponseStep, Message } from "@earendil-works/pi-ai";
import type { LinkStatus } from "#dimos/agents/shared/protocol/gateway/link.ts";
import type { LoadStatus } from "#dimos/agents/shared/protocol/runtime/load.ts";
import type { UserRequestId } from "#dimos/agents/shared/protocol/conversation/input.ts";
import { AssistantEntry, ToolResultEntry } from "@earendil-works/pi-durable";
import { BACKGROUND_CONTEXT } from "@earendil-works/chord/context";
import { DimosInputEntry, requestIdFor } from "#dimos/agents/harness/inputs/source.ts";
import { existsSync, readFileSync, renameSync, writeFileSync } from "node:fs";
import { fauxAssistantMessage, fauxToolCall } from "@earendil-works/pi-ai";
import { join } from "node:path";
import { listJobs } from "#dimos/agents/harness/jobs/registry.ts";
import { openAgent } from "#dimos/agents/harness/main.ts";
import { parseDimosInstall } from "#dimos/agents/gateway/config/environment.ts";
import { randomUUID } from "node:crypto";
import { scriptedModel } from "#dimos/agents/test/scripted-model.ts";

/**
 * A process that runs `openAgent` over the store named by the environment, for tests that kill it mid-turn.
 * AGENT_CHILD_PHASE picks what the faux model answers; AGENT_CHILD_SCRATCH is where markers and results go;
 * AGENT_CHILD_CLOCK, when set, is the Harness clock in epoch milliseconds, frozen.
 */
const scratch = process.env["AGENT_CHILD_SCRATCH"] ?? "";

const phase = process.env["AGENT_CHILD_PHASE"] ?? "";

const clock = process.env["AGENT_CHILD_CLOCK"];

const context = BACKGROUND_CONTEXT;

const model = scriptedModel();

function summarize(message: Message): { role: string; text: string } {
  switch (message.role) {
    case "user":
      return {
        role: "user",
        text:
          typeof message.content === "string"
            ? message.content
            : message.content.map((block) => (block.type === "text" ? block.text : "[image]")).join("\n"),
      };
    case "assistant":
      return {
        role: "assistant",
        text: message.content
          .map((block) =>
            block.type === "toolCall"
              ? `${block.name}(${JSON.stringify(block.arguments)})`
              : block.type === "text"
                ? block.text
                : "",
          )
          .join("\n"),
      };
    case "toolResult":
      return {
        role: `toolResult:${message.toolName}`,
        text: message.content.map((block) => (block.type === "text" ? block.text : "[image]")).join("\n"),
      };
    case "system":
      return {
        role: "system",
        text:
          typeof message.content === "string" ? message.content : message.content.map((block) => block.text).join("\n"),
      };
  }
}

/** Written whole, then renamed, so a reader never sees a partial file. */
function marker(name: string, value: unknown): void {
  const path = join(scratch, name);
  writeFileSync(`${path}.tmp`, JSON.stringify(value, null, 1));
  renameSync(`${path}.tmp`, path);
}

async function open(): Promise<Agent> {
  const agent = await openAgent(
    {
      install: parseDimosInstall(process.env),
      models: model.models,
      storePath: join(scratch, "agent.sqlite"),
      toolResultsDir: join(scratch, "tool-results"),
      renderCacheDir: join(scratch, "render"),
      skillRoots: [],
      defaults: () => ({ model: model.model, thinkingLevel: null, workspace: join(scratch, "root") }),
      gatewayEnv: process.env,
      report: (error) => console.error("[agent-child report]", error),
      now: clock === undefined ? undefined : () => Number(clock),
    },
    context,
  );
  agent.connection.status.subscribe((link) => console.error("[agent-child link]", JSON.stringify(link)));
  agent.loader.status.subscribe((load) => console.error("[agent-child load]", JSON.stringify(load)));
  return agent;
}

/** The `status` command's shape, which the tests read: the DimOS link and the HarnessRuntime load. */
function runtimeStatus(agent: Agent): { readonly link: LinkStatus; readonly load: LoadStatus } {
  return { link: agent.connection.status.value, load: agent.loader.status.value };
}

const forever = (): Promise<never> => new Promise<never>(() => undefined);

const sleep = (ms: number): Promise<void> => new Promise((resolve) => setTimeout(resolve, ms));

type JobMarker = {
  readonly conversation: ConversationId;
  readonly name: string;
  readonly task: number;
  readonly due: number;
};

async function submitAndWait(agent: Agent, conversation: ConversationId, text: string): Promise<string> {
  const result = await agent.inputs.submit(
    conversation,
    {
      source: { kind: "user", request: `req-${text}` as UserRequestId },
      text,
      images: [],
      render: null,
      whenBusy: "reject",
    },
    context,
  );
  if (result.kind !== "submitted") throw new Error(`input not accepted: ${result.kind}`);
  const handle = await agent.harness.submission(result.submission, context);
  if (handle === undefined) throw new Error("submission vanished");
  return (await handle.wait(context)).status;
}

const PHASES: { [name: string]: () => Promise<void> } = {
  /** The shell writes `bash-started` before sleeping; the test kills us once that command has started. */
  "turn-1": async () => {
    model.script([
      fauxAssistantMessage(
        [fauxToolCall("bash", { command: `touch ${join(scratch, "bash-started")}; sleep 60` }, { id: "call_bash" })],
        { stopReason: "toolUse" },
      ),
    ]);
    const agent = await open();
    const conversation = await agent.conversations.start(null, "start-turn-1", context);
    const result = await agent.inputs.submit(
      conversation,
      {
        source: { kind: "user", request: "req-1" as UserRequestId },
        text: "go",
        images: [],
        render: null,
        whenBusy: "reject",
      },
      context,
    );
    marker("conversation.json", { conversation, submission: result.kind === "submitted" ? result.submission : null });
    await forever();
  },
  /** The turn resumes; the model request that follows the tool round records what it was sent, then never answers. */
  "turn-2": async () => {
    model.script([
      async (request) => {
        marker("seen-1.json", request.messages.map(summarize));
        return forever();
      },
    ]);
    await open();
    await forever();
  },
  /** The request is sent again; the model asks for a cut read and an environment probe, then answers. */
  "turn-3": async () => {
    const steps: FauxResponseStep[] = [
      (request) => {
        marker("seen-2.json", request.messages.map(summarize));
        return fauxAssistantMessage(
          [
            fauxToolCall("read", { path: join(scratch, "long-file"), max_chars: 50 }, { id: "call_read_cut" }),
            fauxToolCall(
              "bash",
              { command: 'echo "$DIMOS_ROOT|$FAKE_API_KEY|$(command -v python)|$FROM_DOTENV"' },
              { id: "call_env" },
            ),
          ],
          { stopReason: "toolUse" },
        );
      },
      fauxAssistantMessage("all done"),
    ];
    model.script(steps);
    const agent = await open();
    const { conversation, submission } = JSON.parse(readFileSync(join(scratch, "conversation.json"), "utf8")) as {
      conversation: ConversationId;
      submission: SubmissionId;
    };
    const handle = await agent.harness.submission(submission, context);
    if (handle === undefined) throw new Error("submission not found after reopen");
    const settled = await handle.wait(context);
    const answer =
      settled.status === "done" && settled.answer !== undefined
        ? await agent.harness.commit(
            async (tx) => (await tx.entry(AssistantEntry, settled.answer))?.model?.[0],
            context,
          )
        : undefined;
    const entries = await agent.harness.commit(
      async (tx) => (await tx.scanEntries({ conversationId: conversation, order: "ascending" }, 200)).items,
      context,
    );
    marker("result.json", {
      status: settled.status,
      reason: settled.status === "unanswered" ? settled.reason : null,
      answer: answer === undefined ? null : summarize(answer).text,
      toolResults: entries
        .filter((entry) => ToolResultEntry.is(entry))
        .map((entry) => {
          const message = entry.model?.[0];
          return {
            summary: message === undefined ? null : summarize(message),
            toolCallId: message?.role === "toolResult" ? message.toolCallId : null,
            isError: message?.role === "toolResult" ? message.isError : null,
            details: message?.role === "toolResult" ? message.details : null,
          };
        }),
    });
    await agent.close(context);
  },
  /** The model creates a skill job through the job tool; `job.json` names its task and first due time. */
  "job-create": async () => {
    model.script([
      fauxAssistantMessage(
        [
          fauxToolCall(
            "job",
            {
              op: "create",
              name: "ping",
              cron: "0 0 * * *",
              timezone: "UTC",
              run: { skill: { name: "echo", arguments: { text: "hi" } } },
            },
            { id: "call_job" },
          ),
        ],
        { stopReason: "toolUse" },
      ),
      fauxAssistantMessage("created"),
    ]);
    const agent = await open();
    const conversation = await agent.conversations.start(null, "start-job-create", context);
    const status = await submitAndWait(agent, conversation, "set up the job");
    const job = (await listJobs(agent.harness, context))[0];
    if (status !== "done" || job === undefined) throw new Error(`job not created: ${status}`);
    const task = await agent.harness.getTask(job.task, context);
    const checkpoint = task?.state.checkpoint;
    const due = typeof checkpoint === "object" && checkpoint !== null && "due" in checkpoint ? checkpoint.due : null;
    const record: JobMarker = { conversation, name: job.name, task: job.task, due: typeof due === "number" ? due : -1 };
    marker("job.json", record);
    await agent.close(context);
  },
  /** The job is due: the task commits `attempted` and then waits on the skill call, which the test's silent listener never answers. */
  "job-attempted": async () => {
    const job = JSON.parse(readFileSync(join(scratch, "job.json"), "utf8")) as JobMarker;
    const agent = await open();
    for (;;) {
      const task = await agent.harness.getTask(job.task as TaskId, context);
      const checkpoint = task?.state.checkpoint;
      if (
        typeof checkpoint === "object" &&
        checkpoint !== null &&
        "attempted" in checkpoint &&
        checkpoint.attempted === true
      ) {
        marker("attempted.json", checkpoint);
        break;
      }
      await sleep(20);
    }
    await forever();
  },
  /** After the kill, the firing phase reruns with `attempted` set and records the run without calling the skill. */
  "job-after": async () => {
    const job = JSON.parse(readFileSync(join(scratch, "job.json"), "utf8")) as JobMarker;
    const agent = await open();
    for (;;) {
      const current = (await listJobs(agent.harness, context)).find((item) => item.name === job.name);
      if (current !== undefined && current.runs.length > 0) {
        const task = await agent.harness.getTask(current.task, context);
        marker("runs.json", { runs: current.runs, checkpoint: task?.state.checkpoint ?? null });
        break;
      }
      await sleep(20);
    }
    await agent.close(context);
  },
};

/** Carries out one command of the "serve" phase; a throw becomes the command's error result, never the process's end. */
async function serve(agent: Agent, command: ChildCommand): Promise<unknown> {
  switch (command.op) {
    case "script":
      if (command.replace === true) model.reset(command.messages);
      else model.script(command.messages);
      return { scripted: command.messages.length };
    case "start":
      return { conversation: await agent.conversations.start(null, randomUUID(), context) };
    case "submit": {
      const conversation = command.conversation as ConversationId;
      const result = await agent.inputs.submit(
        conversation,
        {
          source: { kind: "user", request: command.request as UserRequestId },
          text: command.text,
          images: [],
          render: null,
          whenBusy: "reject",
        },
        context,
      );
      if (result.kind !== "submitted") return { status: result.kind, answer: null };
      const handle = await agent.harness.submission(result.submission, context);
      const settled = await handle?.wait(context);
      const answer =
        settled?.status === "done" && settled.answer !== undefined
          ? await agent.harness.commit(
              async (tx) => (await tx.entry(AssistantEntry, settled.answer))?.model?.[0],
              context,
            )
          : undefined;
      return {
        status: settled?.status ?? "lost",
        reason: settled?.status === "unanswered" ? settled.reason : null,
        answer: answer === undefined ? null : summarize(answer).text,
      };
    }
    case "events": {
      const conversation = command.conversation as ConversationId;
      const entries = await agent.harness.commit(
        async (tx) => (await tx.scanEntries({ conversationId: conversation, order: "ascending" }, 500)).items,
        context,
      );
      const inputs = [];
      for (const entry of entries) {
        if (!DimosInputEntry.is(entry) || entry.data.source.kind !== "event") continue;
        const { event } = entry.data.source;
        const submission = await agent.harness.commit(
          (tx) => tx.submissionByRequest(conversation, requestIdFor(entry.data.source)),
          context,
        );
        inputs.push({
          id: event.id,
          module: event.module,
          views: entry.data.render?.views.map((view) => view.kind) ?? [],
          status: submission?.status ?? "missing",
        });
      }
      return { inputs, status: runtimeStatus(agent) };
    }
    case "toolResults": {
      const conversation = command.conversation as ConversationId;
      const entries = await agent.harness.commit(
        async (tx) => (await tx.scanEntries({ conversationId: conversation, order: "ascending" }, 500)).items,
        context,
      );
      return entries
        .filter((entry) => ToolResultEntry.is(entry))
        .map((entry) => {
          const message = entry.model?.[0];
          return message?.role === "toolResult"
            ? {
                toolCallId: message.toolCallId,
                toolName: message.toolName,
                isError: message.isError,
                text: summarize(message).text,
              }
            : null;
        });
    }
    case "status":
      return runtimeStatus(agent);
    case "close":
      await agent.close(context);
      return { closed: true };
  }
}

PHASES["serve"] = async () => {
  process.on("unhandledRejection", (reason) => {
    marker("crash.json", { kind: "unhandledRejection", reason: String(reason) });
    process.exit(1);
  });
  process.on("uncaughtException", (error) => {
    marker("crash.json", { kind: "uncaughtException", reason: String(error) });
    process.exit(1);
  });
  const agent = await open();
  for (let n = 1; ; n += 1) {
    const path = join(scratch, `cmd-${n}.json`);
    while (!existsSync(path)) await sleep(50);
    const command = JSON.parse(readFileSync(path, "utf8")) as ChildCommand;
    let result: unknown;
    try {
      result = await serve(agent, command);
    } catch (error) {
      result = { error: error instanceof Error ? `${error.name}: ${error.message}` : String(error) };
    }
    marker(`cmd-${n}.done.json`, result);
    if (command.op === "close") process.exit(0);
  }
};

const run = PHASES[phase];

if (run === undefined) throw new Error(`unknown phase ${JSON.stringify(phase)}`);

await run();
