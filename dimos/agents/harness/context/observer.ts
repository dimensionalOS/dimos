import type { Context } from "@earendil-works/chord/context";
import type { Conversation, Harness } from "@earendil-works/pi-durable";
import type { Fact, Facts, RuntimeChange } from "#dimos/agents/shared/protocol/runtime/context.ts";
import type { LinkStatus } from "#dimos/agents/shared/protocol/gateway/link.ts";
import type { Observation } from "#dimos/agents/harness/context/facts.ts";
import type { RuntimeClient } from "#dimos/agents/harness/dimos/client.ts";
import { DimosUnavailable } from "#dimos/agents/harness/dimos/connection.ts";
import { diffFacts, observedFacts } from "#dimos/agents/harness/context/facts.ts";
import { runFacts } from "#dimos/agents/harness/context/runs.ts";
import { toldFacts } from "#dimos/agents/harness/context/told.ts";

type ObserverOptions = {
  readonly harness: Harness;
  readonly runtime: RuntimeClient;
  /** The gateway's own facts: one `job:<name>` line per cron job. */
  readonly jobFacts: (context: Context) => Promise<Facts>;
  /** `~/.local/state/dimos/runs/`, whose files name the running systems. */
  readonly runsDir: string;
  /** How long one compact inventory serves every caller. */
  readonly reuseForMs: number;
  /** How long an observation may take before the input goes out without a change. */
  readonly timeoutMs: number;
  /**
   * An observation that failed for another reason than DimOS being down or slow, or gateway facts that could not be
   * read; the input still goes out.
   */
  readonly report: (error: unknown) => void;
};

function unreachable(link: LinkStatus): Fact {
  return {
    section: "Environment",
    line: `DimOS: not reachable, link ${link.state}; any runtime stated earlier is as last seen`,
  };
}

/** Compares the runtime now with what a conversation's transcript has told the model. */
export class RuntimeObserver {
  private readonly options: ObserverOptions;
  private inFlight: { readonly at: number; readonly observation: Promise<Observation | null> } | null = null;

  constructor(options: ObserverOptions) {
    this.options = options;
  }

  /**
   * The change the next input should carry, null when the model already knows the current runtime or the observation
   * timed out, and the facts the conversation was told before it.
   */
  async changeFor(
    conversation: Conversation,
    context: Context,
  ): Promise<{ readonly change: RuntimeChange | null; readonly told: Facts }> {
    const [observation, told] = await Promise.all([
      this.current(context),
      toldFacts(conversation, this.options.harness, context),
    ]);
    return { change: observation === null ? null : diffFacts(told, observedFacts(told, observation)), told };
  }

  /** One `job:<name>` fact per cron job and one `run:<run id>` per running system; none when they cannot be read. */
  gatewayFacts(context: Context): Promise<Facts> {
    return Promise.all([this.options.jobFacts(context), runFacts(this.options.runsDir)]).then(
      ([jobs, runs]): Facts => ({ ...runs, ...jobs }),
      (error: unknown): Facts => {
        this.options.report(error);
        return {};
      },
    );
  }

  /** The compact inventory or why there is none, with job and run facts. One call in flight, reused for `reuseForMs`. */
  private current(context: Context): Promise<Observation | null> {
    const at = Date.now();
    if (this.inFlight !== null && at - this.inFlight.at < this.options.reuseForMs) return this.inFlight.observation;
    const observation = this.observe(context);
    this.inFlight = { at, observation };
    return observation;
  }

  private async observe(context: Context): Promise<Observation | null> {
    const gateway = this.gatewayFacts(context);
    const abort = new AbortController();
    const timer = setTimeout(() => abort.abort(), this.options.timeoutMs);
    try {
      const system = await this.options.runtime.snapshot(abort.signal);
      return { kind: "up", system, gateway: await gateway };
    } catch (error) {
      if (error instanceof DimosUnavailable)
        return { kind: "down", status: unreachable(error.status), gateway: await gateway };
      if (!abort.signal.aborted) this.options.report(error);
      return null;
    } finally {
      clearTimeout(timer);
    }
  }
}
