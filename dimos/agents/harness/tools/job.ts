import type { AgentTool } from "#dimos/agents/harness/tools/base.ts";
import type { EpochMs } from "#dimos/agents/shared/protocol/time.ts";
import type { JobDefinition, JobName } from "#dimos/agents/shared/protocol/jobs/state.ts";
import type { JobOps } from "#dimos/agents/harness/jobs/ops.ts";
import type { JsonObject, ToolExecutionResult } from "@earendil-works/pi-durable";
import type { Static } from "typebox";
import { Type } from "typebox";
import { agentTool } from "#dimos/agents/harness/tools/lib/tool.ts";
import { defineTool } from "@earendil-works/pi-durable";
import { jobIn, listJobs, parseJobName } from "#dimos/agents/harness/jobs/registry.ts";

const JobParameters = Type.Object({
  op: Type.Union([
    Type.Literal("list"),
    Type.Literal("get"),
    Type.Literal("create"),
    Type.Literal("update"),
    Type.Literal("delete"),
    Type.Literal("run"),
  ]),
  name: Type.Optional(
    Type.String({ description: "Lowercase letters, digits, '-' and '_'. Every op but list needs it." }),
  ),
  cron: Type.Optional(Type.String({ description: "minute hour day-of-month month day-of-week" })),
  timezone: Type.Optional(Type.String({ description: "IANA name; this machine's zone when omitted" })),
  run: Type.Optional(
    Type.Union([
      Type.Object({
        notify: Type.String({ description: "Text that starts a turn in this conversation when the job fires." }),
      }),
      Type.Object({
        skill: Type.Object({
          name: Type.String({ description: "A DimOS skill, as McpServer lists it." }),
          arguments: Type.Optional(Type.Record(Type.String(), Type.Unknown())),
        }),
      }),
    ]),
  ),
});

type JobArgs = Static<typeof JobParameters>;

function text(value: string, isError = false): ToolExecutionResult {
  return { output: [{ type: "text", text: value }], isError, details: {} };
}

function describe(job: JobDefinition, nextDue: EpochMs | null): string {
  const action =
    job.action.kind === "notify"
      ? `notify ${JSON.stringify(job.action.text)}`
      : `skill ${job.action.skill}(${JSON.stringify(job.action.arguments)})`;
  const last = job.runs.at(-1);
  return [
    `${job.name}: ${JSON.stringify(job.cron)} ${job.timeZone}, ${action}`,
    `  next: ${nextDue === null ? "none (task ended)" : new Date(nextDue).toISOString()}`,
    last === undefined
      ? "  runs: none"
      : `  last run: ${new Date(last.firedAt).toISOString()} ${JSON.stringify(last.outcome)}`,
  ].join("\n");
}

function actionOf(run: JobArgs["run"]): JobDefinition["action"] | null {
  if (run === undefined) return null;
  if ("notify" in run) return { kind: "notify", text: run.notify };
  return { kind: "skill", skill: run.skill.name, arguments: (run.skill.arguments ?? {}) as JsonObject };
}

/** The `job` tool over JobOps, which the gateway's job methods share; `run` fires inside the call. */
export function jobTool(ops: JobOps): AgentTool<"job"> {
  const tool = defineTool({
    name: "job",
    description:
      "Create, change, delete, list and run cron jobs. A job fires on its cron schedule and either notifies this conversation (starting a turn) or calls a DimOS skill through McpServer without a turn. Jobs survive gateway restarts; a run missed while the gateway was down fires once, late.",
    parameters: JobParameters,
    execute: async (args, api, context) => {
      if (args.op === "list") {
        const jobs = await listJobs(api, context);
        if (jobs.length === 0) return text("No jobs.");
        const lines = await Promise.all(jobs.map(async (job) => describe(job, await ops.nextDue(job, context))));
        return text(lines.join("\n"));
      }
      let name: JobName;
      try {
        name = parseJobName(args.name ?? "");
      } catch (error) {
        return text(error instanceof Error ? error.message : String(error), true);
      }
      switch (args.op) {
        case "get": {
          const job = await api.commit((tx) => jobIn(tx, name), context);
          return job === null ? text(`No job ${name}.`, true) : text(describe(job, await ops.nextDue(job, context)));
        }
        case "create":
        case "update": {
          const action = actionOf(args.run);
          if (args.cron === undefined || action === null) return text(`${args.op} needs cron and run.`, true);
          const draft = {
            name,
            conversation: api.conversationId,
            cron: args.cron,
            timeZone: args.timezone === undefined ? null : args.timezone,
            action,
          };
          const change = args.op === "create" ? await ops.create(draft, context) : await ops.update(draft, context);
          switch (change.kind) {
            case "invalid":
              return text(change.problem, true);
            case "exists":
              return text(`Job ${name} exists; use update.`, true);
            case "missing":
              return text(`No job ${name}; use create.`, true);
            case "done":
              return text(
                `${args.op === "create" ? "Created" : "Updated"} ${name}; next run ${new Date(change.firstDue).toISOString()}.`,
              );
          }
        }
        case "delete": {
          const removed = await ops.remove(name, context);
          return removed === null ? text(`No job ${name}.`, true) : text(`Deleted ${name}.`);
        }
        case "run": {
          const outcome = await ops.run(name, { kind: "manual", toolCall: api.callId }, context);
          if (outcome === null) return text(`No job ${name}.`, true);
          return text(`Ran ${name}: ${JSON.stringify(outcome)}`, outcome.kind === "error");
        }
      }
    },
  });
  return agentTool("job", tool, {
    replay: "unsafe",
    settle: null,
    fullTextAt: null,
    wholeResult: null,
    background: null,
  });
}
