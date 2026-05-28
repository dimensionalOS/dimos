"use client";

import type { Job } from "@robomoo/shared";
import Link from "next/link";
import { StatusPill } from "@/components/status-pill";
import { Skeleton } from "@/components/ui/skeleton";
import { rpcClient } from "@/lib/orpc";
import { useLiveQuery } from "@/lib/use-live-query";

// "Past work" for an agent profile — this agent's recent jobs with links to the
// deliverable. Filters the recent jobs list client-side by agent slug.
export function AgentJobs({ slug }: { slug: string }) {
  const { data: jobs, loading } = useLiveQuery<Job[]>(
    async () => (await rpcClient.jobs.list()).filter((j) => j.agentSlug === slug),
    8000,
    [slug],
  );

  if (loading && !jobs) {
    return (
      <div className="flex flex-col gap-2">
        {[0, 1].map((i) => (
          <Skeleton className="h-11 w-full rounded-lg" key={i} />
        ))}
      </div>
    );
  }

  if (!jobs || jobs.length === 0) {
    return (
      <p className="rounded-lg border border-dashed px-4 py-3 text-muted-foreground text-sm">
        No jobs for this agent yet.
      </p>
    );
  }

  return (
    <div className="flex flex-col gap-2">
      {jobs.map((j) => (
        <Link
          className="flex items-center justify-between gap-3 rounded-lg border bg-card px-4 py-2.5 text-sm transition-colors hover:border-signal/40"
          href={`/jobs/${j.id}`}
          key={j.id}
        >
          <span className="font-mono text-muted-foreground text-xs">{j.id}</span>
          <span className="flex items-center gap-3">
            <span className="text-muted-foreground">{j.service}</span>
            <StatusPill status={j.status} />
          </span>
        </Link>
      ))}
    </div>
  );
}
