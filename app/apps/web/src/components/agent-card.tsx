import type { Agent } from "@robomoo/shared";
import Link from "next/link";
import { IdentityPill } from "@/components/identity-pill";
import { ReputationBadge } from "@/components/reputation-badge";
import { cn } from "@/lib/utils";

export function AgentCard({ agent }: { agent: Agent }) {
  const live = agent.status === "live";
  const extraCaps = Math.max(0, agent.capabilities.length - 3);

  return (
    <Link
      className={cn(
        "group relative flex h-full flex-col gap-4 overflow-hidden rounded-xl border bg-card p-6 transition-all duration-200",
        live
          ? "hover:-translate-y-0.5 hover:border-signal/50 hover:shadow-[0_8px_30px_-14px_var(--signal)]"
          : "opacity-65 hover:opacity-100 hover:border-foreground/20",
      )}
      href={`/agents/${agent.slug}`}
    >
      <span className="absolute top-4 right-4">
        {live ? (
          <span className="inline-flex items-center gap-1.5 rounded-full bg-signal/15 px-2 py-0.5 font-medium text-[10px] text-signal uppercase tracking-wide">
            <span className="size-1.5 rounded-full bg-signal pulse-dot" /> Live
          </span>
        ) : (
          <span className="rounded-full bg-muted px-2 py-0.5 text-[10px] text-muted-foreground uppercase tracking-wide">
            Soon
          </span>
        )}
      </span>

      <div className="flex items-center gap-3">
        <div className="flex size-14 shrink-0 items-center justify-center rounded-xl border bg-secondary/40 text-3xl">
          {agent.emoji ?? "🤖"}
        </div>
        <div className="flex min-w-0 flex-col gap-1">
          <span className="font-display font-semibold text-lg leading-tight">
            {agent.name}
          </span>
          <ReputationBadge agent={agent} />
        </div>
      </div>

      <p className="text-muted-foreground text-sm leading-relaxed">
        {agent.tagline}
      </p>

      <div className="flex flex-wrap gap-1.5">
        {agent.capabilities.slice(0, 3).map((c) => (
          <span
            className="rounded-md border bg-secondary/40 px-2 py-0.5 text-[11px] text-muted-foreground"
            key={c}
          >
            {c}
          </span>
        ))}
        {extraCaps > 0 ? (
          <span className="rounded-md px-2 py-0.5 text-[11px] text-muted-foreground/70">
            +{extraCaps}
          </span>
        ) : null}
      </div>

      <div className="mt-auto flex items-center justify-between border-t pt-3.5">
        <IdentityPill agent={agent} />
        <span className="font-medium font-mono text-sm">
          {agent.basePriceUsd > 0 ? (
            <>
              <span className="text-muted-foreground text-xs">from </span>$
              {agent.basePriceUsd}
            </>
          ) : (
            "—"
          )}
        </span>
      </div>
    </Link>
  );
}
