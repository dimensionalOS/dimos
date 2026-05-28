"use client";

import type { Agent, Job, JobStatus } from "@robomoo/shared";
import {
  AlertTriangle,
  ArrowLeft,
  Check,
  Loader2,
  Sparkles,
  Star,
} from "lucide-react";
import Link from "next/link";
import { useEffect, useState } from "react";
import { ConsolePanel } from "@/components/console-panel";
import { JobScans } from "@/components/job-scans";
import { MapView } from "@/components/map-view";
import { SplatViewer } from "@/components/splat-viewer";
import { Button } from "@/components/ui/button";
import { Skeleton } from "@/components/ui/skeleton";
import { giveFeedbackOnchain, reputationConfigured } from "@/lib/chain";
import { rpcClient } from "@/lib/orpc";
import { useLiveQuery } from "@/lib/use-live-query";
import { cn } from "@/lib/utils";
import { useWallet } from "@/lib/wallet";

const STEPS: { label: string }[] = [
  { label: "Hired" },
  { label: "Dispatched" },
  { label: "Scanning" },
  { label: "Reconstructing" },
  { label: "Delivered" },
];

function currentStep(job: Job): number {
  switch (job.status) {
    case "booked":
      return 0;
    case "dispatched":
      return 1;
    case "scanning":
      return 2;
    case "reconstructing":
      return 3;
    case "done":
      return 4;
    case "failed":
    case "cancelled":
      return job.run ? 3 : job.dispatchedAt ? 2 : 1;
    default:
      return 0;
  }
}

export function JobView({ jobId }: { jobId: string }) {
  const { data: job, loading, refetch } = useLiveQuery<Job | null>(
    () => rpcClient.jobs.get({ id: jobId }),
    2500,
    [jobId],
  );
  const [agent, setAgent] = useState<Agent | null>(null);

  useEffect(() => {
    if (job && !agent) {
      rpcClient.agents
        .get({ slug: job.agentSlug })
        .then(setAgent)
        .catch(() => {});
    }
  }, [job, agent]);

  if (loading && !job) {
    return (
      <main className="mx-auto flex max-w-5xl flex-col gap-6 px-4 py-12">
        <Skeleton className="h-8 w-64" />
        <Skeleton className="h-14 w-full" />
        <Skeleton className="h-[50vh] w-full" />
      </main>
    );
  }

  if (!job) {
    return (
      <main className="mx-auto max-w-2xl px-4 py-12">
        <p className="text-muted-foreground">
          Job not found.{" "}
          <Link className="text-signal hover:underline" href="/">
            Back to marketplace
          </Link>
        </p>
      </main>
    );
  }

  const inFlight =
    job.status === "dispatched" ||
    job.status === "scanning" ||
    job.status === "reconstructing";
  const failed = job.status === "failed" || job.status === "cancelled";
  const done = job.status === "done";

  return (
    <main className="mx-auto flex max-w-5xl flex-col gap-8 px-4 py-12">
      <div className="flex flex-col gap-3">
        <Link
          className="inline-flex w-fit items-center gap-1.5 text-muted-foreground text-sm transition-colors hover:text-foreground"
          href={`/agents/${job.agentSlug}`}
        >
          <ArrowLeft size={14} /> {agent?.name ?? job.agentSlug}
        </Link>
        <div className="flex flex-wrap items-center gap-3">
          <h1 className="font-bold font-display text-3xl tracking-tight">
            {agent?.emoji ?? "🤖"} {agent?.name ?? "Agent"}
          </h1>
          <JobStatusPill status={job.status} />
          <span className="font-mono text-muted-foreground text-sm">
            {job.id}
          </span>
        </div>
      </div>

      <Stepper job={job} />

      <Telemetry agent={agent} job={job} />

      {inFlight ? (
        <ConsolePanel
          live
          title={`${agent?.name ?? "Agent"} — live feed`}
        >
          <MapView />
        </ConsolePanel>
      ) : null}

      {job.run ? (
        <section className="flex flex-col gap-3">
          <h2 className="font-display font-semibold text-lg">
            Scanned rooms · segmented
          </h2>
          <JobScans run={job.run} />
        </section>
      ) : inFlight ? (
        <ConsolePanel title="Capturing rooms">
          <div className="flex flex-col gap-3 p-4">
            <div className="flex items-center gap-2 text-muted-foreground text-sm">
              <Loader2 className="animate-spin text-working" size={15} />
              Waiting for the first scan — the run attaches automatically once
              {" "}
              {agent?.name ?? "the agent"} starts capturing.
            </div>
            <div className="flex gap-2">
              {[0, 1, 2, 3].map((i) => (
                <Skeleton className="shimmer h-24 w-32 rounded-md" key={i} />
              ))}
            </div>
          </div>
        </ConsolePanel>
      ) : null}

      {failed ? <FailedPanel job={job} /> : null}

      {done || job.splatId ? (
        <section className="flex flex-col gap-3">
          <div className="flex items-center gap-2">
            <Sparkles className="text-signal" size={18} />
            <h2 className="font-display font-semibold text-lg">
              Your 3D virtual tour is ready
            </h2>
          </div>
          <SplatViewer />
        </section>
      ) : null}

      {done && agent ? (
        <RateAgent agent={agent} job={job} onRated={refetch} />
      ) : null}

      <DemoControls job={job} onChange={refetch} />
    </main>
  );
}

function JobStatusPill({ status }: { status: JobStatus }) {
  const inFlight =
    status === "dispatched" ||
    status === "scanning" ||
    status === "reconstructing";
  const failed = status === "failed" || status === "cancelled";
  const cls = failed
    ? "bg-destructive/15 text-destructive"
    : status === "done"
      ? "bg-signal/15 text-signal"
      : inFlight
        ? "bg-working/15 text-working"
        : "bg-muted text-muted-foreground";
  return (
    <span
      className={cn(
        "inline-flex items-center gap-1.5 rounded-full px-2.5 py-0.5 font-medium text-xs uppercase tracking-wide",
        cls,
      )}
    >
      {inFlight ? (
        <span className="size-1.5 rounded-full bg-current pulse-dot" />
      ) : null}
      {inFlight ? "Live" : status}
    </span>
  );
}

function Stepper({ job }: { job: Job }) {
  const idx = currentStep(job);
  const failed = job.status === "failed" || job.status === "cancelled";
  const done = job.status === "done";

  return (
    <div className="flex items-start">
      {STEPS.map((s, i) => {
        const isDone = done || i < idx;
        const isActive = !done && !failed && i === idx;
        const isFailed = failed && i === idx;
        const lineDone = done || i < idx;
        return (
          // biome-ignore lint/suspicious/noArrayIndexKey: fixed-length static list
          <div className="flex flex-1 flex-col items-center gap-2" key={i}>
            <div className="flex w-full items-center">
              <div className={i === 0 ? "flex-1" : "flex-1"}>
                {i > 0 ? (
                  <div
                    className={cn(
                      "h-0.5 w-full",
                      done || i <= idx ? "bg-signal" : "bg-border",
                    )}
                  />
                ) : null}
              </div>
              <span
                className={cn(
                  "flex size-8 shrink-0 items-center justify-center rounded-full font-medium text-xs transition-colors",
                  isDone && "bg-signal text-signal-foreground",
                  isActive &&
                    "bg-working/20 text-working ring-2 ring-working/40 pulse-dot",
                  isFailed && "bg-destructive text-white",
                  !isDone &&
                    !isActive &&
                    !isFailed &&
                    "bg-muted text-muted-foreground",
                )}
              >
                {isDone ? (
                  <Check size={15} />
                ) : isFailed ? (
                  <AlertTriangle size={14} />
                ) : isActive ? (
                  <Loader2 className="animate-spin" size={14} />
                ) : (
                  i + 1
                )}
              </span>
              <div className="flex-1">
                {i < STEPS.length - 1 ? (
                  <div
                    className={cn(
                      "h-0.5 w-full",
                      lineDone ? "bg-signal" : "bg-border",
                    )}
                  />
                ) : null}
              </div>
            </div>
            <span
              className={cn(
                "text-[10px] uppercase tracking-wide",
                isActive
                  ? "text-working"
                  : isDone
                    ? "text-foreground"
                    : "text-muted-foreground",
              )}
            >
              {s.label}
            </span>
          </div>
        );
      })}
    </div>
  );
}

function Telemetry({ job, agent }: { job: Job; agent: Agent | null }) {
  const cells: { label: string; value: React.ReactNode }[] = [
    {
      label: "Elapsed",
      value: <Elapsed since={job.dispatchedAt} />,
    },
    {
      label: "Run",
      value: job.run ? (
        <span className="text-foreground">{job.run.slice(0, 12)}</span>
      ) : (
        "—"
      ),
    },
    {
      label: "Command",
      value: job.command ? (
        <span className="truncate text-foreground" title={job.command}>
          {job.command}
        </span>
      ) : (
        "—"
      ),
    },
    {
      label: "Payment",
      value: job.paid ? (
        <span className="text-signal">
          ${job.priceUsd} · {job.paymentMode}
        </span>
      ) : (
        <span className="text-muted-foreground">unpaid</span>
      ),
    },
  ];
  return (
    <div className="grid grid-cols-2 gap-px overflow-hidden rounded-xl border bg-border sm:grid-cols-4">
      {cells.map((c) => (
        <div className="flex flex-col gap-1 bg-card p-3" key={c.label}>
          <span className="font-mono text-[10px] text-muted-foreground uppercase tracking-wider">
            {c.label}
          </span>
          <span className="overflow-hidden font-mono text-sm">{c.value}</span>
        </div>
      ))}
    </div>
  );
}

function Elapsed({ since }: { since: string | null }) {
  const [now, setNow] = useState(() => Date.now());
  useEffect(() => {
    if (!since) return;
    const t = setInterval(() => setNow(Date.now()), 1000);
    return () => clearInterval(t);
  }, [since]);
  if (!since) return <span className="text-muted-foreground">—</span>;
  const s = Math.max(0, Math.floor((now - new Date(since).getTime()) / 1000));
  const mm = Math.floor(s / 60);
  const ss = s % 60;
  return (
    <span className="text-foreground tabular-nums">
      {mm}:{ss.toString().padStart(2, "0")}
    </span>
  );
}

function FailedPanel({ job }: { job: Job }) {
  return (
    <section className="flex items-start gap-3 rounded-xl border border-destructive/30 bg-destructive/5 p-5">
      <AlertTriangle className="mt-0.5 shrink-0 text-destructive" size={18} />
      <div className="flex flex-col gap-1">
        <h2 className="font-display font-semibold text-destructive">
          This job didn&apos;t complete
        </h2>
        <p className="text-muted-foreground text-sm leading-relaxed">
          The job is marked <span className="font-mono">{job.status}</span>. If
          the robot was running, use the demo controls below to attach a scan run
          or mark it delivered.
        </p>
      </div>
    </section>
  );
}

// Star rating → ERC-8004 giveFeedback (on-chain when registered + configured),
// always recorded off-chain on the job so reputation reflects it.
function RateAgent({
  agent,
  job,
  onRated,
}: {
  agent: Agent;
  job: Job;
  onRated: () => Promise<void>;
}) {
  const { address, connect, ensureSepolia } = useWallet();
  const [rating, setRating] = useState(5);
  const [hover, setHover] = useState(0);
  const [busy, setBusy] = useState(false);
  const [msg, setMsg] = useState<string | null>(null);
  const [error, setError] = useState<string | null>(null);

  if (job.rating) {
    return (
      <section className="flex flex-col gap-2 rounded-xl border border-signal/30 bg-signal/5 p-5">
        <h2 className="font-display font-semibold text-lg">Rated · thank you</h2>
        <p className="flex flex-wrap items-center gap-2 text-sm">
          You rated {agent.name}
          <span className="inline-flex">
            {[0, 1, 2, 3, 4].map((i) => (
              <Star
                className={
                  i < (job.rating ?? 0)
                    ? "text-amber-400"
                    : "text-muted-foreground/30"
                }
                fill={i < (job.rating ?? 0) ? "currentColor" : "none"}
                key={i}
                size={16}
                strokeWidth={i < (job.rating ?? 0) ? 0 : 1.5}
              />
            ))}
          </span>
          {job.feedbackTx ? (
            <a
              className="text-signal hover:underline"
              href={`https://sepolia.etherscan.io/tx/${job.feedbackTx}`}
              rel="noopener noreferrer"
              target="_blank"
            >
              view receipt ↗
            </a>
          ) : null}
        </p>
      </section>
    );
  }

  const canOnchain = Boolean(agent.agentId) && reputationConfigured;

  const submit = async () => {
    setError(null);
    setMsg(null);
    setBusy(true);
    try {
      let feedbackTx: string | null = null;
      if (canOnchain) {
        if (!address) {
          await connect();
          setBusy(false);
          return;
        }
        await ensureSepolia();
        setMsg("Confirm the feedback transaction in your wallet…");
        feedbackTx = await giveFeedbackOnchain(
          address,
          // biome-ignore lint/style/noNonNullAssertion: guarded by canOnchain
          agent.agentId!,
          rating,
        );
      }
      await rpcClient.jobs.setFeedback({ id: job.id, rating, feedbackTx });
      await onRated();
    } catch (e) {
      setError(e instanceof Error ? e.message : "failed to submit rating");
    } finally {
      setBusy(false);
    }
  };

  const shown = hover || rating;

  return (
    <section className="flex flex-col gap-3 rounded-xl border bg-card p-5">
      <h2 className="font-display font-semibold text-lg">Rate {agent.name}</h2>
      <p className="text-muted-foreground text-sm leading-relaxed">
        {canOnchain
          ? "Your rating becomes part of this agent's public, tamper-proof track record."
          : "Your rating updates the agent's reputation."}
      </p>
      <div className="flex items-center gap-1">
        {[1, 2, 3, 4, 5].map((n) => (
          <button
            aria-label={`${n} stars`}
            className="transition-transform hover:scale-110"
            key={n}
            onClick={() => setRating(n)}
            onMouseEnter={() => setHover(n)}
            onMouseLeave={() => setHover(0)}
            type="button"
          >
            <Star
              className={n <= shown ? "text-amber-400" : "text-muted-foreground/40"}
              fill={n <= shown ? "currentColor" : "none"}
              size={28}
              strokeWidth={n <= shown ? 0 : 1.5}
            />
          </button>
        ))}
      </div>
      <Button className="w-fit" disabled={busy} onClick={submit}>
        {busy ? <Loader2 className="animate-spin" size={16} /> : null}
        {busy ? "Submitting…" : `Submit ${rating}★ rating`}
      </Button>
      {msg ? <span className="text-muted-foreground text-xs">{msg}</span> : null}
      {error ? <span className="text-destructive text-xs">{error}</span> : null}
    </section>
  );
}

// Manual nudges for a live demo when there's no real robot driving the pipeline
// (claim-latest needs frames to exist; these let you wire deliverables by hand).
function DemoControls({
  job,
  onChange,
}: {
  job: Job;
  onChange: () => Promise<void>;
}) {
  const [open, setOpen] = useState(false);
  const [busy, setBusy] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const claimLatestRun = async () => {
    setBusy(true);
    setError(null);
    try {
      const headers = await rpcClient.frames.scansHeaders();
      const newest = headers[0]?.run;
      if (!newest) {
        setError("No scan runs exist yet.");
        return;
      }
      await rpcClient.jobs.attachRun({ id: job.id, run: newest });
      await onChange();
    } catch (e) {
      setError(e instanceof Error ? e.message : "failed");
    } finally {
      setBusy(false);
    }
  };

  const markDone = async () => {
    setBusy(true);
    setError(null);
    try {
      await rpcClient.jobs.complete({ id: job.id, splatId: null });
      await onChange();
    } catch (e) {
      setError(e instanceof Error ? e.message : "failed");
    } finally {
      setBusy(false);
    }
  };

  return (
    <section className="rounded-xl border border-dashed bg-muted/20 p-4 text-sm">
      <button
        className="font-mono text-muted-foreground text-xs uppercase tracking-wider"
        onClick={() => setOpen((o) => !o)}
        type="button"
      >
        {open ? "▾" : "▸"} Demo controls
      </button>
      {open ? (
        <div className="mt-3 flex flex-wrap items-center gap-2">
          <Button
            disabled={busy}
            onClick={claimLatestRun}
            size="sm"
            variant="outline"
          >
            Attach latest scan run
          </Button>
          <Button disabled={busy} onClick={markDone} size="sm" variant="outline">
            Mark delivered
          </Button>
          {job.run ? (
            <span className="font-mono text-muted-foreground text-xs">
              run: {job.run}
            </span>
          ) : null}
          {error ? (
            <span className="text-destructive text-xs">{error}</span>
          ) : null}
        </div>
      ) : null}
    </section>
  );
}
