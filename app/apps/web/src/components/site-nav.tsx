"use client";

import Link from "next/link";
import { Button } from "@/components/ui/button";
import { useWallet } from "@/lib/wallet";

function shortAddr(a: string): string {
  return `${a.slice(0, 6)}…${a.slice(-4)}`;
}

export function ConnectWallet() {
  const { address, connect, connecting, hasWallet, onSepolia, ensureSepolia } =
    useWallet();

  if (address) {
    return (
      <div className="flex items-center gap-2">
        {!onSepolia ? (
          <Button onClick={() => ensureSepolia()} size="sm" variant="outline">
            Switch to Sepolia
          </Button>
        ) : null}
        <span
          className="rounded-full border bg-card px-3 py-1 font-mono text-xs"
          title={address}
        >
          {onSepolia ? "🟢" : "🟡"} {shortAddr(address)}
        </span>
      </div>
    );
  }

  return (
    <Button
      disabled={connecting}
      onClick={() => connect()}
      size="sm"
      variant="outline"
    >
      {connecting ? "Connecting…" : hasWallet ? "Connect wallet" : "Get a wallet"}
    </Button>
  );
}

export function SiteNav() {
  return (
    <header className="sticky top-0 z-40 border-b bg-background/80 backdrop-blur">
      <div className="mx-auto flex max-w-6xl items-center justify-between gap-4 px-4 py-3">
        <div className="flex items-center gap-6">
          <Link className="font-bold text-lg tracking-tight" href="/">
            robomoo
          </Link>
          <nav className="flex items-center gap-4 text-muted-foreground text-sm">
            <Link className="hover:text-foreground" href="/">
              Marketplace
            </Link>
            <Link className="hover:text-foreground" href="/jobs">
              My jobs
            </Link>
          </nav>
        </div>
        <ConnectWallet />
      </div>
    </header>
  );
}
