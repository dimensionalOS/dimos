"use client";

import { Check, Copy } from "lucide-react";
import Link from "next/link";
import { usePathname } from "next/navigation";
import { useState } from "react";
import { Button } from "@/components/ui/button";
import { cn } from "@/lib/utils";
import { useWallet } from "@/lib/wallet";

function shortAddr(a: string): string {
  return `${a.slice(0, 6)}…${a.slice(-4)}`;
}

function WalletPill({ address, onSepolia }: { address: string; onSepolia: boolean }) {
  const [copied, setCopied] = useState(false);
  const copy = async () => {
    try {
      await navigator.clipboard.writeText(address);
      setCopied(true);
      setTimeout(() => setCopied(false), 1200);
    } catch {
      /* ignore */
    }
  };
  return (
    <button
      className="inline-flex items-center gap-2 rounded-full border bg-card px-3 py-1 font-mono text-xs transition-colors hover:border-foreground/30"
      onClick={copy}
      title={onSepolia ? `${address} · Sepolia` : `${address} · wrong network`}
      type="button"
    >
      <span
        className={cn(
          "size-1.5 rounded-full",
          onSepolia ? "bg-signal pulse-dot" : "bg-working",
        )}
      />
      {shortAddr(address)}
      {copied ? (
        <Check className="text-signal" size={12} />
      ) : (
        <Copy className="text-muted-foreground" size={12} />
      )}
    </button>
  );
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
        <WalletPill address={address} onSepolia={onSepolia} />
      </div>
    );
  }

  return (
    <Button disabled={connecting} onClick={() => connect()} size="sm">
      {connecting ? "Connecting…" : hasWallet ? "Connect wallet" : "Get a wallet"}
    </Button>
  );
}

function NavLink({ href, active, children }: { href: string; active: boolean; children: React.ReactNode }) {
  return (
    <Link
      className={cn(
        "relative py-1 transition-colors hover:text-foreground",
        active ? "text-foreground" : "text-muted-foreground",
      )}
      href={href}
    >
      {children}
      {active ? (
        <span className="-bottom-px absolute inset-x-0 h-0.5 rounded-full bg-signal" />
      ) : null}
    </Link>
  );
}

export function SiteNav() {
  const pathname = usePathname();
  const onMarket = pathname === "/" || pathname.startsWith("/agents");
  const onJobs = pathname.startsWith("/jobs");
  const onConsole = pathname.startsWith("/console");

  return (
    <header className="sticky top-0 z-40 border-b bg-background/70 backdrop-blur-md">
      <div className="mx-auto flex max-w-6xl items-center justify-between gap-4 px-4 py-3">
        <div className="flex items-center gap-7">
          <Link className="flex items-center gap-2" href="/">
            <span className="size-2.5 rounded-[3px] bg-signal shadow-[0_0_12px_var(--signal)]" />
            <span className="font-bold font-display text-lg tracking-tight">
              robomoo
            </span>
          </Link>
          <nav className="flex items-center gap-5 text-sm">
            <NavLink active={onMarket} href="/">
              Marketplace
            </NavLink>
            <NavLink active={onJobs} href="/jobs">
              My jobs
            </NavLink>
            <NavLink active={onConsole} href="/console">
              Console
            </NavLink>
          </nav>
        </div>
        <ConnectWallet />
      </div>
    </header>
  );
}
