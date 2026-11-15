import type { Context } from "@earendil-works/chord/context";
import type { DimosConnection } from "#dimos/agents/harness/dimos/connection.ts";
import type { InputGate } from "#dimos/agents/harness/inputs/gate.ts";
import type { JobDefinition, JobOutcome, JobRun } from "#dimos/agents/shared/protocol/jobs/state.ts";

export type JobFiringDeps = {
  readonly inputs: InputGate;
  readonly connection: DimosConnection;
  readonly skillTimeoutMs: number;
};

/**
 * Runs a job's action once. A notify run is an input whose request id names the job and the run, so submitting
 * it again does nothing. A skill call cannot be made idempotent here; its caller records the run before firing, so a
 * crash never repeats it. Every failure becomes an `error` outcome: a phase that throws faults its task, and the job
 * would never run again.
 */
export async function fireJob(
  job: JobDefinition,
  run: JobRun,
  deps: JobFiringDeps,
  context: Context,
): Promise<JobOutcome> {
  try {
    if (job.action.kind === "notify") {
      const result = await deps.inputs.submit(
        job.conversation,
        {
          source: { kind: "job", job: job.name, run },
          text: `[Job ${job.name}] ${job.action.text}`,
          images: [],
          render: null,
          whenBusy: "followUp",
        },
        context,
      );
      return result.kind === "submitted"
        ? { kind: "submitted" }
        : { kind: "error", text: `input not accepted: ${result.kind}` };
    }
    const outcome = await deps.connection.callTool(job.action.skill, job.action.arguments, {
      caller: job.conversation,
      timeoutMs: deps.skillTimeoutMs,
      signal: context.abortSignal,
    });
    const text = outcome.content.flatMap((block) => (block.type === "text" ? [block.text] : [])).join("\n");
    return { kind: outcome.isError ? "error" : "ok", text };
  } catch (error) {
    return { kind: "error", text: error instanceof Error ? error.message : String(error) };
  }
}
