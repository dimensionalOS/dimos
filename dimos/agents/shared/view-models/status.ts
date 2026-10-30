import type { Ref } from "@earendil-works/chord";
import type { GatewayContract } from "../protocol/gateway/contract.ts";
import type { LifecycleContract } from "../protocol/lifecycle/contract.ts";
import type { ModuleRow } from "../protocol/runtime/module.ts";
import type { EpochMs } from "../protocol/time.ts";
import { linkSegments, runSegment } from "./gateway/link.ts";
import { switcherRows } from "./gateway/switcher.ts";
import type { Segment } from "./levels.ts";
import { clock, duration } from "./time.ts";

const LABEL_WIDTH = 11;

function labelled(label: string, text: string): string {
  return `${label.padEnd(LABEL_WIDTH)}${text}`;
}

/** "follower #2 r12 running", "greeter #3 r4 stopped (ImportError)". */
function moduleText(module: ModuleRow): string {
  const owner = module.owner === null ? "" : ` #${module.owner}`;
  const revision = module.revision === null ? "" : ` r${module.revision}`;
  const error = module.lastError === null ? "" : ` (${module.lastError.split(":")[0] ?? module.lastError})`;
  return `${module.name}${owner}${revision} ${module.state}${error}`;
}

/**
 * `dimos agent status` as lines, from the derivations the TUI draws, without colour. A line with nothing to say is
 * left out: nothing busy, no modules, no jobs, no streams.
 */
export function statusText(
  refs: { gateway: Ref<typeof GatewayContract>; lifecycle: Ref<typeof LifecycleContract> },
  now: EpochMs,
): readonly string[] {
  const process = refs.lifecycle.process.value;
  const gateway = refs.gateway;
  const runs = gateway.runs.value;
  const lines = [
    labelled(
      "gateway",
      `running · build ${process.build.build} · pid ${process.pid} · up ${duration(now - process.startedAt)}`,
    ),
  ];
  const run = runSegment(runs);
  const dimos: Segment[] = [...linkSegments(gateway.link.value, gateway.load.value, runs, now)];
  if (run !== null) dimos.push(run);
  lines.push(labelled("DimOS", dimos.map((segment) => segment.text.replace(/^DimOS /, "")).join(" · ")));
  const busy = switcherRows(gateway.conversations.value, null, now)
    .filter((row) => row.state !== "idle")
    .map((row) => `#${row.id} ${row.state}`);
  if (busy.length > 0) lines.push(labelled("busy", busy.join(" · ")));
  const modules = gateway.modules.value;
  if (modules.length > 0) lines.push(labelled("modules", modules.map(moduleText).join(" · ")));
  const jobs = gateway.jobs.value.map(
    (job) => `${job.name} #${job.conversation} ${job.nextDue === null ? "ended" : `next ${clock(job.nextDue)}`}`,
  );
  if (jobs.length > 0) lines.push(labelled("jobs", jobs.join(" · ")));
  const streams = gateway.streams.value.length;
  if (streams > 0) lines.push(labelled("streams", String(streams)));
  return lines;
}
