"use client";

import type { Agent, Job, JobStatus } from "@robomoo/shared";
import Link from "next/link";
import { useCallback, useEffect, useState } from "react";
import { JobScans } from "@/components/job-scans";
import { MapView } from "@/components/map-view";
import { SplatViewer } from "@/components/splat-viewer";
import { Button } from "@/components/ui/button";
import { giveFeedbackOnchain, reputationConfigured } from "@/lib/chain";
import { rpcClient } from "@/lib/orpc";
import { useWallet } from "@/lib/wallet";

const STEPS: { key: JobStatus; label: string }[] = [
  { key: "booked", label: "Booked" },
  { key: "scanning", label: "Scanning" },
  { key: "reconstructing", label: "Reconstructing" },
  { key: "done", label: "Done" },
];

function stepIndex(status: JobStatus): number {
  switch (status) {
    case "booked":
      return 0;
    case "dispatched":
    case "scanning":
      return 1;
    case "reconstructing":
      return 2;
    case "done":
      return 3;
    default:
      return 0;
  }
}

export function JobView({ jobId }: { jobId: string }) {
  const [job, setJob] = useState<Job | null>(null);
  const [agent, setAgent] = useState<Agent | null>(null);
  const [notFound, setNotFound] = useState(false);

  const reload = useCallback(async () => {
    const j = await rpcClient.jobs.get({ id: jobId });
    if (!j) {
      setNotFound(true);
      return;
    }
    setJob(j);
    if (!agent) {
      const a = await rpcClient.agents.get({ slug: j.agentSlug });
      setAgent(a);
    }
  }, [jobId, agent]);

  useEffect(() => {
    let active = true;
    const tick = async () => {
      try {
        if (active) await reload();
      } catch {
        /* keep last good state */
      }
    };
    tick();
    const t = setInterval(tick, 3500);
    return () => {
      active = false;
      clearInterval(t);
    };
  }, [reload]);

  if (notFound) {
    return (
      <main className="mx-auto max-w-2xl px-4 py-12">
        <p className="text-muted-foreground">
          Job not found.{" "}
          <Link className="underline" href="/">
            Back to marketplace
          </Link>
        </p>
      </main>
    );
  }

  if (!job) {
    return (
      <main className="mx-auto max-w-2xl px-4 py-12 text-muted-foreground">
        Loading job…
      </main>
    );
  }

  const idx = stepIndex(job.status);
  const inFlight =
    job.status === "dispatched" ||
    job.status === "scanning" ||
    job.status === "reconstructing";

  return (
    <main className="mx-auto flex max-w-5xl flex-col gap-8 px-4 py-12">
      <div className="flex flex-col gap-1">
        <Link
          className="text-muted-foreground text-sm hover:text-foreground"
          href={`/agents/${job.agentSlug}`}
        >
          ← {agent?.name ?? job.agentSlug}
        </Link>
        <h1 className="font-bold text-3xl tracking-tight">
          {agent?.emoji ?? "🤖"} Job{" "}
          <span className="font-mono text-2xl text-muted-foreground">
            {job.id}
          </span>
        </h1>
        <p className="text-muted-foreground text-sm">
          {job.service} · ${job.priceUsd} ·{" "}
          {job.paid ? `paid (${job.paymentMode})` : "unpaid"}
        </p>
      </div>

      <Stepper currentIndex={idx} status={job.status} />

      {inFlight ? (
        <section className="flex flex-col gap-3">
          <h2 className="font-semibold text-lg">
            Live — {agent?.name ?? "the agent"} on the move
          </h2>
          <MapView />
        </section>
      ) : null}

      {job.run ? (
        <section className="flex flex-col gap-3">
          <h2 className="font-semibold text-lg">Scanned rooms (segmented)</h2>
          <JobScans run={job.run} />
        </section>
      ) : inFlight ? (
        <p className="text-muted-foreground text-sm">
          Waiting for the first scan to come in… (the scan run attaches
          automatically once RoboDoc starts capturing)
        </p>
      ) : null}

      {job.status === "done" || job.splatId ? (
        <section className="flex flex-col gap-3">
          <h2 className="font-semibold text-lg">Your 3D virtual tour</h2>
          <SplatViewer />
        </section>
      ) : null}

      {job.status === "done" && agent ? (
        <RateAgent agent={agent} job={job} onRated={reload} />
      ) : null}

      <DemoControls job={job} onChange={reload} />
    </main>
  );
}

function Stepper({
  currentIndex,
  status,
}: {
  currentIndex: number;
  status: JobStatus;
}) {
  const failed = status === "failed" || status === "cancelled";
  return (
    <div className="flex items-center gap-2">
      {STEPS.map((s, i) => {
        const done = i < currentIndex;
        const active = i === currentIndex;
        return (
          <div className="flex flex-1 items-center gap-2" key={s.key}>
            <div className="flex flex-col items-center gap-1">
              <span
                className={`flex size-7 items-center justify-center rounded-full text-xs ${
                  done
                    ? "bg-emerald-500 text-white"
                    : active
                      ? failed
                        ? "bg-destructive text-white"
                        : "bg-primary text-primary-foreground"
                      : "bg-muted text-muted-foreground"
                }`}
              >
                {done ? "✓" : i + 1}
              </span>
              <span className="text-[10px] text-muted-foreground">
                {s.label}
              </span>
            </div>
            {i < STEPS.length - 1 ? (
              <div
                className={`h-0.5 flex-1 ${done ? "bg-emerald-500" : "bg-border"}`}
              />
            ) : null}
          </div>
        );
      })}
    </div>
  );
}

// Star rating → ERC-8004 giveFeedback (on-chain when registered + configured),
// always recorded off-chain on the job so the agent's reputation reflects it.
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
  const [busy, setBusy] = useState(false);
  const [msg, setMsg] = useState<string | null>(null);
  const [error, setError] = useState<string | null>(null);

  if (job.rating) {
    return (
      <section className="flex flex-col gap-1 rounded-xl border bg-card p-5">
        <h2 className="font-semibold text-lg">Rated</h2>
        <p className="text-sm">
          You rated {agent.name}{" "}
          <span className="text-amber-500">{"★".repeat(job.rating)}</span>
          {job.feedbackTx ? (
            <>
              {" · "}
              <a
                className="underline"
                href={`https://sepolia.etherscan.io/tx/${job.feedbackTx}`}
                rel="noopener noreferrer"
                target="_blank"
              >
                on-chain feedback ↗
              </a>
            </>
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

  return (
    <section className="flex flex-col gap-3 rounded-xl border bg-card p-5">
      <h2 className="font-semibold text-lg">Rate {agent.name}</h2>
      <p className="text-muted-foreground text-sm">
        {canOnchain
          ? "Your rating is written to the ERC-8004 Reputation Registry — it ticks up the agent's on-chain reputation."
          : "Your rating updates the agent's reputation. (Connect the reputation registry to write it on-chain.)"}
      </p>
      <div className="flex items-center gap-1 text-2xl">
        {[1, 2, 3, 4, 5].map((n) => (
          <button
            aria-label={`${n} stars`}
            className={n <= rating ? "text-amber-500" : "text-muted-foreground"}
            key={n}
            onClick={() => setRating(n)}
            type="button"
          >
            ★
          </button>
        ))}
      </div>
      <Button className="w-fit" disabled={busy} onClick={submit}>
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
    <section className="rounded-xl border border-dashed bg-muted/30 p-4 text-sm">
      <button
        className="text-muted-foreground text-xs"
        onClick={() => setOpen((o) => !o)}
        type="button"
      >
        {open ? "▾" : "▸"} Demo controls
      </button>
      {open ? (
        <div className="mt-3 flex flex-wrap items-center gap-2">
          <Button disabled={busy} onClick={claimLatestRun} size="sm" variant="outline">
            Attach latest scan run
          </Button>
          <Button disabled={busy} onClick={markDone} size="sm" variant="outline">
            Mark done
          </Button>
          {job.run ? (
            <span className="text-muted-foreground text-xs">
              run: <span className="font-mono">{job.run}</span>
            </span>
          ) : null}
          {error ? <span className="text-destructive text-xs">{error}</span> : null}
        </div>
      ) : null}
    </section>
  );
}
