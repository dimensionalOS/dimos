import type { Agent } from "@robomoo/shared";
import Link from "next/link";
import { notFound } from "next/navigation";
import { AgentJobs } from "@/components/agent-jobs";
import { HireFlow } from "@/components/hire-flow";
import { IdentityPill } from "@/components/identity-pill";
import { RegisterAgentButton } from "@/components/register-agent-button";
import { ReputationBadge } from "@/components/reputation-badge";
import { rpcClient } from "@/lib/orpc";

export const dynamic = "force-dynamic";

export default async function AgentPage({
  params,
}: {
  params: Promise<{ slug: string }>;
}) {
  const { slug } = await params;
  let agent: Agent | null = null;
  try {
    agent = await rpcClient.agents.get({ slug });
  } catch {
    agent = null;
  }
  if (!agent) notFound();

  const live = agent.status === "live";

  return (
    <main className="mx-auto grid max-w-6xl grid-cols-1 gap-8 px-4 py-12 lg:grid-cols-[1fr_360px]">
      <div className="flex flex-col gap-8">
        <Link className="text-muted-foreground text-sm hover:text-foreground" href="/">
          ← Marketplace
        </Link>

        <header className="flex flex-col gap-4">
          <div className="flex items-center gap-4">
            <div className="flex size-16 items-center justify-center rounded-xl bg-muted text-4xl">
              {agent.emoji ?? "🤖"}
            </div>
            <div className="flex flex-col gap-1">
              <h1 className="font-bold text-3xl tracking-tight">{agent.name}</h1>
              <p className="text-muted-foreground">{agent.tagline}</p>
            </div>
          </div>
          <div className="flex flex-wrap items-center gap-3">
            <ReputationBadge agent={agent} />
            <IdentityPill agent={agent} />
            {!live ? (
              <span className="rounded-full bg-muted px-2 py-0.5 text-muted-foreground text-xs uppercase">
                Coming soon
              </span>
            ) : null}
          </div>
        </header>

        <section className="flex flex-col gap-2">
          <h2 className="font-semibold text-lg">About</h2>
          <p className="text-muted-foreground text-sm leading-relaxed">
            {agent.description}
          </p>
          <div className="mt-1 flex flex-wrap gap-1">
            {agent.capabilities.map((c) => (
              <span
                className="rounded bg-accent px-2 py-0.5 text-accent-foreground text-xs"
                key={c}
              >
                {c}
              </span>
            ))}
          </div>
        </section>

        <section className="flex flex-col gap-3">
          <h2 className="font-semibold text-lg">On-chain identity</h2>
          <div className="flex flex-col gap-2 rounded-xl border bg-card p-4 text-sm">
            <Row label="Chain">Ethereum Sepolia (ERC-8004)</Row>
            <Row label="Agent ID">
              {agent.agentId ? `#${agent.agentId}` : "not registered"}
            </Row>
            <Row label="Wallet">
              <span className="font-mono text-xs">
                {agent.agentWallet ?? "—"}
              </span>
            </Row>
            {agent.isReal ? (
              <div className="pt-1">
                <RegisterAgentButton agent={agent} />
              </div>
            ) : null}
          </div>
        </section>

        <section className="flex flex-col gap-3">
          <h2 className="font-semibold text-lg">Past work</h2>
          <AgentJobs slug={agent.slug} />
        </section>
      </div>

      <aside className="lg:sticky lg:top-20 lg:self-start">
        {live ? (
          <HireFlow agent={agent} />
        ) : (
          <div className="flex flex-col gap-2 rounded-xl border bg-card p-5 text-sm">
            <h3 className="font-semibold">Coming soon</h3>
            <p className="text-muted-foreground">
              {agent.name} isn&apos;t hireable yet. RoboDoc is live now — try a
              real room scan.
            </p>
            <Link className="text-primary underline" href="/agents/robodoc">
              → Hire RoboDoc
            </Link>
          </div>
        )}
      </aside>
    </main>
  );
}

function Row({ label, children }: { label: string; children: React.ReactNode }) {
  return (
    <div className="flex items-center justify-between gap-4">
      <span className="text-muted-foreground">{label}</span>
      <span className="text-right">{children}</span>
    </div>
  );
}
