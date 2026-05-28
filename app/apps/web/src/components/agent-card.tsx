import type { Agent } from "@robomoo/shared";
import Link from "next/link";
import { IdentityPill } from "@/components/identity-pill";
import { ReputationBadge } from "@/components/reputation-badge";

export function AgentCard({ agent }: { agent: Agent }) {
  const live = agent.status === "live";
  return (
    <Link
      className="group relative flex flex-col gap-3 rounded-xl border bg-card p-5 text-card-foreground shadow-sm transition-all hover:border-foreground/30 hover:shadow-md"
      href={`/agents/${agent.slug}`}
    >
      {!live ? (
        <span className="absolute top-3 right-3 rounded-full bg-muted px-2 py-0.5 text-[10px] text-muted-foreground uppercase tracking-wide">
          Coming soon
        </span>
      ) : null}

      <div className="flex items-center gap-3">
        <div className="flex size-12 items-center justify-center rounded-lg bg-muted text-2xl">
          {agent.emoji ?? "🤖"}
        </div>
        <div className="flex flex-col">
          <span className="font-semibold text-lg leading-tight">
            {agent.name}
          </span>
          <ReputationBadge agent={agent} />
        </div>
      </div>

      <p className="text-muted-foreground text-sm">{agent.tagline}</p>

      <div className="flex flex-wrap gap-1">
        {agent.capabilities.slice(0, 4).map((c) => (
          <span
            className="rounded bg-accent px-1.5 py-0.5 text-[10px] text-accent-foreground"
            key={c}
          >
            {c}
          </span>
        ))}
      </div>

      <div className="mt-auto flex items-center justify-between pt-2">
        <IdentityPill agent={agent} />
        <span className="font-medium text-sm">
          {agent.basePriceUsd > 0 ? `from $${agent.basePriceUsd}` : "—"}
        </span>
      </div>
    </Link>
  );
}
