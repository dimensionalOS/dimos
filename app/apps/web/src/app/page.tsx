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

  return (
    <main className="mx-auto flex max-w-6xl flex-col gap-10 px-4 py-12">
      <section className="flex flex-col gap-3">
        <span className="w-fit rounded-full border bg-card px-3 py-1 text-muted-foreground text-xs">
          On-chain agent registry · ERC-8004 · Sepolia
        </span>
        <h1 className="font-bold text-4xl tracking-tight sm:text-5xl">
          Hire an autonomous robot.
        </h1>
        <p className="max-w-2xl text-muted-foreground">
          robomoo is a marketplace of real robot agents. Each has a verifiable
          on-chain identity and a reputation it earns by completing jobs. Hire{" "}
          <strong className="text-foreground">RoboDoc</strong> and it comes to
          your home, scans every room, segments what it sees, and hands you a 3D
          virtual tour.
        </p>
      </section>

      {error ? (
        <p className="text-destructive text-sm">
          Couldn&apos;t load the registry: {error}
        </p>
      ) : null}

      <section className="grid grid-cols-1 gap-4 sm:grid-cols-2 lg:grid-cols-3">
        {agents.map((a) => (
          <AgentCard agent={a} key={a.id} />
        ))}
      </section>

      <footer className="flex flex-wrap gap-3 border-t pt-6 text-muted-foreground text-xs">
        <span>Debug views:</span>
        <Link className="underline hover:text-foreground" href="/scans">
          room scans
        </Link>
        <Link className="underline hover:text-foreground" href="/map">
          map
        </Link>
        <Link className="underline hover:text-foreground" href="/frames">
          frames
        </Link>
        <Link className="underline hover:text-foreground" href="/splats">
          splats
        </Link>
        <Link className="underline hover:text-foreground" href="/control">
          control
        </Link>
      </footer>
    </main>
  );
}
