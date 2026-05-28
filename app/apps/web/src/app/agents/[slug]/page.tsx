import type { Agent } from "@robomoo/shared";
import { ArrowLeft } from "lucide-react";
import Link from "next/link";
import { notFound } from "next/navigation";
import { AddressChip } from "@/components/address-chip";
import { AgentJobs } from "@/components/agent-jobs";
import { HireFlow } from "@/components/hire-flow";
import { IdentityPill } from "@/components/identity-pill";
import { RegisterAgentButton } from "@/components/register-agent-button";
import { ReputationBadge } from "@/components/reputation-badge";
import { rpcClient } from "@/lib/orpc";

export const dynamic = "force-dynamic";

export default async function AgentPage({
  params,
  searchParams,
}: {
  params: Promise<{ slug: string }>;
  searchParams: Promise<{ [key: string]: string | string[] | undefined }>;
}) {
  const { slug } = await params;
  const sp = await searchParams;
  // Operator-only setup view (?operator=1) — identity management lives here,
  // out of the public marketplace.
  const operator = sp.operator === "1" || sp.operator === "true";
  let agent: Agent | null = null;
  try {
    agent = await rpcClient.agents.get({ slug });
  } catch {
    agent = null;
  }
  if (!agent) notFound();

  const live = agent.status === "live";

  return (
    <main className="mx-auto grid max-w-6xl grid-cols-1 gap-8 px-4 py-12 lg:grid-cols-[1fr_380px]">
      <div className="flex flex-col gap-8">
        <Link
          className="inline-flex w-fit items-center gap-1.5 text-muted-foreground text-sm transition-colors hover:text-foreground"
          href="/"
        >
          <ArrowLeft size={14} /> Marketplace
        </Link>

        <header className="flex flex-col gap-5">
          <div className="flex items-start gap-4">
            <div className="flex size-20 shrink-0 items-center justify-center rounded-2xl border bg-secondary/40 text-5xl">
              {agent.emoji ?? "🤖"}
            </div>
            <div className="flex flex-col gap-2 pt-1">
              <div className="flex flex-wrap items-center gap-3">
                <h1 className="font-bold font-display text-4xl tracking-tight">
                  {agent.name}
                </h1>
                {live ? (
                  <span className="inline-flex items-center gap-1.5 rounded-full bg-signal/15 px-2.5 py-0.5 font-medium text-signal text-xs uppercase tracking-wide">
                    <span className="size-1.5 rounded-full bg-signal pulse-dot" />
                    Live
                  </span>
                ) : (
                  <span className="rounded-full bg-muted px-2.5 py-0.5 text-muted-foreground text-xs uppercase tracking-wide">
                    Coming soon
                  </span>
                )}
              </div>
              <p className="text-lg text-muted-foreground">{agent.tagline}</p>
            </div>
          </div>
          <div className="flex flex-wrap items-center gap-3">
            <ReputationBadge agent={agent} />
            <IdentityPill agent={agent} />
          </div>
        </header>

        <section className="flex flex-col gap-3">
          <h2 className="font-display font-semibold text-lg">About</h2>
          <p className="text-muted-foreground text-sm leading-relaxed">
            {agent.description}
          </p>
          <div className="mt-1 flex flex-wrap gap-1.5">
            {agent.capabilities.map((c) => (
              <span
                className="rounded-md border bg-secondary/40 px-2 py-0.5 text-muted-foreground text-xs"
                key={c}
              >
                {c}
              </span>
            ))}
          </div>
        </section>

        {operator ? (
          <section className="flex flex-col gap-3">
            <h2 className="font-display font-semibold text-lg">
              Identity{" "}
              <span className="font-normal font-sans text-muted-foreground text-xs">
                · operator
              </span>
            </h2>
            <div className="flex flex-col divide-y rounded-xl border bg-card text-sm">
              <Row label="Identity">
                {agent.agentId ? (
                  <span className="font-medium font-mono text-verified">
                    #{agent.agentId}
                  </span>
                ) : (
                  <span className="text-muted-foreground">not set up</span>
                )}
              </Row>
              <Row label="Wallet">
                {agent.agentWallet ? (
                  <AddressChip
                    etherscan={{ type: "address", hash: agent.agentWallet }}
                    value={agent.agentWallet}
                  />
                ) : (
                  <span className="text-muted-foreground">—</span>
                )}
              </Row>
              {agent.registerTx ? (
                <Row label="Record">
                  <AddressChip
                    etherscan={{ type: "tx", hash: agent.registerTx }}
                    value={agent.registerTx}
                  />
                </Row>
              ) : null}
              {agent.isReal && !agent.agentId ? (
                <div className="px-4 py-3">
                  <RegisterAgentButton agent={agent} />
                </div>
              ) : null}
            </div>
          </section>
        ) : null}

        <section className="flex flex-col gap-3">
          <h2 className="font-display font-semibold text-lg">Past work</h2>
          <AgentJobs slug={agent.slug} />
        </section>
      </div>

      <aside className="lg:sticky lg:top-20 lg:self-start">
        {live ? (
          <HireFlow agent={agent} />
        ) : (
          <div className="flex flex-col gap-3 rounded-xl border bg-card p-5 text-sm">
            <h3 className="font-display font-semibold">Not hireable yet</h3>
            <p className="text-muted-foreground leading-relaxed">
              {agent.name} is coming soon. RoboDoc is live now — try a real room
              scan and watch it work.
            </p>
            <Link
              className="font-medium text-signal hover:underline"
              href="/agents/robodoc"
            >
              → Hire RoboDoc
            </Link>
          </div>
        )}
      </aside>
    </main>
  );
}

function Row({
  label,
  children,
}: {
  label: string;
  children: React.ReactNode;
}) {
  return (
    <div className="flex items-center justify-between gap-4 px-4 py-3">
      <span className="text-muted-foreground">{label}</span>
      <span className="text-right">{children}</span>
    </div>
  );
}
