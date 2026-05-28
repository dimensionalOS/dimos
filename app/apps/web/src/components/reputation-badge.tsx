"use client";

import type { Agent } from "@robomoo/shared";
import { ReputationStars } from "@/components/reputation-stars";
import { Skeleton } from "@/components/ui/skeleton";
import { getReputationSummary } from "@/lib/chain";
import { rpcClient } from "@/lib/orpc";
import { useLiveQuery } from "@/lib/use-live-query";

type Rep =
  | { kind: "onchain"; avg: number; count: number }
  | { kind: "offchain"; avg: number; count: number }
  | { kind: "none" };

// Prefers a live ERC-8004 on-chain summary (when the agent is registered + the
// reputation registry is configured); otherwise falls back to the job-derived
// average. Polls so the counter ticks up live after a job is rated.
async function loadRep(agent: Agent): Promise<Rep> {
  if (agent.agentId) {
    const onchain = await getReputationSummary(agent.agentId);
    if (onchain && onchain.count > 0) {
      return { kind: "onchain", avg: onchain.avg, count: onchain.count };
    }
  }
  const stats = await rpcClient.agents.stats({ slug: agent.slug });
  if (stats.rated > 0 && stats.avgRating !== null) {
    return { kind: "offchain", avg: stats.avgRating, count: stats.completed };
  }
  return { kind: "none" };
}

export function ReputationBadge({ agent }: { agent: Agent }) {
  const { data: rep, loading } = useLiveQuery<Rep>(() => loadRep(agent), 10000, [
    agent.agentId,
    agent.slug,
  ]);

  if (loading && !rep) {
    return <Skeleton className="h-4 w-28" />;
  }
  if (!rep || rep.kind === "none") {
    return <span className="text-muted-foreground text-xs">No ratings yet</span>;
  }

  return <ReputationStars count={rep.count} value={rep.avg} />;
}
