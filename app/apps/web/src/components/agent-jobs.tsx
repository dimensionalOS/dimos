"use client";

import type { Job } from "@robomoo/shared";
import Link from "next/link";
import { useEffect, useState } from "react";
import { rpcClient } from "@/lib/orpc";

// "Past work" for an agent profile — this agent's recent jobs with links to the
// deliverable. Filters the recent jobs list client-side by agent slug.
export function AgentJobs({ slug }: { slug: string }) {
  const [jobs, setJobs] = useState<Job[]>([]);

  useEffect(() => {
    let active = true;
    const load = async () => {
      try {
        const all = await rpcClient.jobs.list();
        if (active) setJobs(all.filter((j) => j.agentSlug === slug));
      } catch {
        /* ignore */
      }
    };
    load();
    const t = setInterval(load, 8000);
    return () => {
      active = false;
      clearInterval(t);
    };
  }, [slug]);

  if (jobs.length === 0) {
    return (
      <p className="text-muted-foreground text-sm">No jobs for this agent yet.</p>
    );
  }

  return (
    <div className="flex flex-col gap-2">
      {jobs.map((j) => (
        <Link
          className="flex items-center justify-between rounded-lg border bg-card px-4 py-2 text-sm hover:border-foreground/30"
          href={`/jobs/${j.id}`}
          key={j.id}
        >
          <span className="font-mono text-xs">{j.id}</span>
          <span className="flex items-center gap-3">
            <span className="text-muted-foreground">{j.service}</span>
            <StatusPill status={j.status} />
          </span>
        </Link>
      ))}
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
