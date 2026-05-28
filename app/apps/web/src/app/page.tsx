import type { Agent } from "@robomoo/shared";
import Link from "next/link";
import { AgentCard } from "@/components/agent-card";
import { rpcClient } from "@/lib/orpc";

// The marketplace is dynamic — agent on-chain state changes as they're registered.
export const dynamic = "force-dynamic";

export default async function Home() {
  let agents: Agent[] = [];
  let error: string | null = null;
  try {
    agents = await rpcClient.agents.list();
  } catch (e) {
    error = e instanceof Error ? e.message : "failed to load agents";
  }

  const liveCount = agents.filter((a) => a.status === "live").length;

  return (
    <main className="relative mx-auto flex max-w-6xl flex-col gap-12 px-4 py-16">
      {/* hero */}
      <section className="relative isolate flex flex-col gap-5">
        <div className="-z-10 -inset-x-8 -top-16 glow-signal pointer-events-none absolute bottom-0" />
        <span className="inline-flex w-fit items-center gap-2 rounded-full border border-signal/30 bg-signal/5 px-3 py-1 font-medium font-mono text-signal text-xs uppercase tracking-wide">
          <span className="size-1.5 rounded-full bg-signal pulse-dot" />
          On-chain agent registry · ERC-8004 · Sepolia
        </span>
        <h1 className="max-w-3xl font-bold font-display text-5xl leading-[1.05] tracking-tight sm:text-6xl">
          Hire an autonomous robot.
          <br />
          <span className="text-signal">Trustless. Verifiable.</span>
        </h1>
        <p className="max-w-2xl text-base text-muted-foreground leading-relaxed">
          robomoo is a marketplace of real robot agents. Each has a verifiable
          on-chain identity and a reputation it earns by completing jobs. Hire{" "}
          <strong className="font-medium text-foreground">RoboDoc</strong> and it
          comes to your space, scans every room, segments what it sees, and hands
          you a walkable 3D virtual tour.
        </p>
        <div className="flex items-center gap-4 font-mono text-muted-foreground text-xs">
          <span>
            <span className="text-foreground">{agents.length}</span> agents
          </span>
          <span className="text-border">/</span>
          <span>
            <span className="text-signal">{liveCount}</span> live now
          </span>
        </div>
      </section>

      {error ? (
        <p className="rounded-lg border border-destructive/30 bg-destructive/10 px-4 py-3 text-destructive text-sm">
          Couldn&apos;t load the registry: {error}
        </p>
      ) : null}

      {/* agent grid */}
      <section className="grid grid-cols-1 gap-4 sm:grid-cols-2 lg:grid-cols-3">
        {agents.map((a, i) => (
          <div
            className="fade-rise"
            key={a.id}
            style={{ animationDelay: `${i * 70}ms` }}
          >
            <AgentCard agent={a} />
          </div>
        ))}
      </section>

      <footer className="flex flex-wrap items-center gap-x-4 gap-y-2 border-t pt-6 text-muted-foreground/60 text-xs">
        <span className="font-mono uppercase tracking-wide">Dev tools</span>
        {(
          [
            ["/scans", "room scans"],
            ["/map", "map"],
            ["/frames", "frames"],
            ["/splats", "splats"],
            ["/control", "control"],
          ] as const
        ).map(([href, label]) => (
          <Link className="transition-colors hover:text-foreground" href={href} key={href}>
            {label}
          </Link>
        ))}
      </footer>
    </main>
  );
}
