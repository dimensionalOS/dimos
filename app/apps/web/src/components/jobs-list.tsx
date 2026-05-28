"use client";

import type { Job } from "@robomoo/shared";
import Link from "next/link";
import { useEffect, useState } from "react";
import { rpcClient } from "@/lib/orpc";
import { useWallet } from "@/lib/wallet";

// "My jobs" — defaults to the connected wallet's jobs, with a toggle to show
// every recent job (useful for a shared demo machine).
export function JobsList() {
  const { address } = useWallet();
  const [jobs, setJobs] = useState<Job[]>([]);
  const [mineOnly, setMineOnly] = useState(true);

  useEffect(() => {
    let active = true;
    const load = async () => {
      try {
        const rows = await rpcClient.jobs.list(
          mineOnly && address ? { address } : { address: null },
        );
        if (active) setJobs(rows);
      } catch {
        /* ignore */
      }
    };
    load();
    const t = setInterval(load, 6000);
    return () => {
      active = false;
      clearInterval(t);
    };
  }, [address, mineOnly]);

  return (
    <div className="flex flex-col gap-4">
      <div className="flex items-center gap-3 text-sm">
        <label className="flex items-center gap-2">
          <input
            checked={mineOnly}
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

      {jobs.length === 0 ? (
        <p className="text-muted-foreground text-sm">
          No jobs yet.{" "}
          <Link className="underline" href="/">
            Hire an agent →
          </Link>
        </p>
      ) : (
        <div className="flex flex-col gap-2">
          {jobs.map((j) => (
            <Link
              className="flex items-center justify-between rounded-lg border bg-card px-4 py-3 text-sm hover:border-foreground/30"
              href={`/jobs/${j.id}`}
              key={j.id}
            >
              <span className="flex flex-col gap-0.5">
                <span className="font-medium">{j.agentSlug}</span>
                <span className="font-mono text-muted-foreground text-xs">
                  {j.id}
                </span>
              </span>
              <span className="flex items-center gap-3">
                <span className="text-muted-foreground text-xs">
                  ${j.priceUsd}
                </span>
                {j.rating ? (
                  <span className="text-amber-500 text-xs">
                    {"★".repeat(j.rating)}
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

function StatusPill({ status }: { status: Job["status"] }) {
  const color =
    status === "done"
      ? "bg-emerald-500/15 text-emerald-600"
      : status === "failed" || status === "cancelled"
        ? "bg-destructive/15 text-destructive"
        : "bg-amber-500/15 text-amber-600";
  return (
    <span className={`rounded px-2 py-0.5 text-[10px] uppercase ${color}`}>
      {status}
    </span>
  );
}
