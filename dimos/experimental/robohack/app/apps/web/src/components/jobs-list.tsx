"use client";

import type { Job } from "@robomoo/shared";
import { Star } from "lucide-react";
import Link from "next/link";
import { useState } from "react";
import { StatusPill } from "@/components/status-pill";
import { Skeleton } from "@/components/ui/skeleton";
import { rpcClient } from "@/lib/orpc";
import { useLiveQuery } from "@/lib/use-live-query";
import { cn } from "@/lib/utils";
import { useWallet } from "@/lib/wallet";

// "My jobs" — defaults to the connected wallet's jobs, with a toggle to show
// every recent job (useful for a shared demo machine).
export function JobsList() {
  const { address } = useWallet();
  const [mineOnly, setMineOnly] = useState(true);
  const { data: jobs, loading } = useLiveQuery<Job[]>(
    () =>
      rpcClient.jobs.list(
        mineOnly && address ? { address } : { address: null },
      ),
    6000,
    [address, mineOnly],
  );

  return (
    <div className="flex flex-col gap-4">
      <div className="flex items-center gap-3 text-sm">
        <label
          className={cn(
            "inline-flex items-center gap-2",
            !address && "opacity-50",
          )}
        >
          <input
            checked={mineOnly}
            className="accent-signal"
            disabled={!address}
            onChange={(e) => setMineOnly(e.target.checked)}
            type="checkbox"
          />
          My wallet only
        </label>
        {!address ? (
          <span className="text-muted-foreground text-xs">
            Connect a wallet to filter to your jobs.
          </span>
        ) : null}
      </div>

      {loading && !jobs ? (
        <div className="flex flex-col gap-2">
          {[0, 1, 2].map((i) => (
            <Skeleton className="h-16 w-full rounded-lg" key={i} />
          ))}
        </div>
      ) : !jobs || jobs.length === 0 ? (
        <div className="flex flex-col items-center gap-2 rounded-xl border border-dashed py-12 text-center">
          <p className="text-muted-foreground text-sm">No jobs yet.</p>
          <Link className="font-medium text-signal text-sm hover:underline" href="/">
            Hire an agent →
          </Link>
        </div>
      ) : (
        <div className="flex flex-col gap-2">
          {jobs.map((j) => (
            <Link
              className="flex items-center justify-between gap-4 rounded-lg border bg-card px-4 py-3 transition-colors hover:border-signal/40"
              href={`/jobs/${j.id}`}
              key={j.id}
            >
              <span className="flex min-w-0 flex-col gap-0.5">
                <span className="font-display font-medium">{j.agentSlug}</span>
                <span className="font-mono text-muted-foreground text-xs">
                  {j.id}
                </span>
              </span>
              <span className="flex shrink-0 items-center gap-3">
                <span className="font-mono text-muted-foreground text-xs">
                  ${j.priceUsd}
                </span>
                {j.rating ? (
                  <span className="inline-flex items-center gap-0.5 text-amber-400 text-xs">
                    <Star fill="currentColor" size={12} strokeWidth={0} />
                    {j.rating}
                  </span>
                ) : null}
                <StatusPill status={j.status} />
              </span>
            </Link>
          ))}
        </div>
      )}
    </div>
  );
}
