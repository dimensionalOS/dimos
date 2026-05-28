"use client";

import type { Agent } from "@robomoo/shared";
import { useEffect, useState } from "react";
import { getReputationSummary } from "@/lib/chain";
import { rpcClient } from "@/lib/orpc";

function Stars({ value }: { value: number }) {
  const full = Math.round(value);
  return (
    <span className="text-amber-500" title={`${value.toFixed(2)} / 5`}>
      {"★".repeat(Math.max(0, Math.min(5, full)))}
      <span className="text-muted-foreground">
        {"★".repeat(Math.max(0, 5 - full))}
      </span>
    </span>
  );
}

type Rep =
  | { kind: "loading" }
  | { kind: "onchain"; avg: number; count: number }
  | { kind: "offchain"; avg: number; count: number }
  | { kind: "none" };

// Reputation badge. Prefers a live ERC-8004 on-chain summary (when the agent is
// registered and the reputation registry is configured); otherwise falls back to
// the job-derived average from completed+rated jobs. Polls so the counter ticks
// up live after a job is rated.
export function ReputationBadge({ agent }: { agent: Agent }) {
  const [rep, setRep] = useState<Rep>({ kind: "loading" });

  useEffect(() => {
    let active = true;
    const load = async () => {
      if (agent.agentId) {
        const onchain = await getReputationSummary(agent.agentId);
        if (!active) return;
        if (onchain && onchain.count > 0) {
          setRep({ kind: "onchain", avg: onchain.avg, count: onchain.count });
          return;
        }
      }
      try {
        const stats = await rpcClient.agents.stats({ slug: agent.slug });
        if (!active) return;
        if (stats.rated > 0 && stats.avgRating !== null) {
          setRep({
            kind: "offchain",
            avg: stats.avgRating,
            count: stats.completed,
          });
        } else {
          setRep({ kind: "none" });
        }
      } catch {
        if (active) setRep({ kind: "none" });
      }
    };
    load();
    const t = setInterval(load, 10000);
    return () => {
      active = false;
      clearInterval(t);
    };
  }, [agent.agentId, agent.slug]);

  if (rep.kind === "loading") {
    return <span className="text-muted-foreground text-xs">…</span>;
  }
  if (rep.kind === "none") {
    return <span className="text-muted-foreground text-xs">No jobs yet</span>;
  }
  return (
    <span className="flex items-center gap-1.5 text-xs">
      <Stars value={rep.avg} />
      <span className="font-medium">{rep.avg.toFixed(1)}</span>
      <span className="text-muted-foreground">
        · {rep.count} job{rep.count === 1 ? "" : "s"}
      </span>
      {rep.kind === "onchain" ? (
        <span className="rounded bg-emerald-500/15 px-1.5 py-0.5 text-[10px] text-emerald-600">
          on-chain
        </span>
      ) : null}
    </span>
  );
}
