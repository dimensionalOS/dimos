"use client";

import { Check, Copy, ExternalLink } from "lucide-react";
import { useState } from "react";
import { cn } from "@/lib/utils";

const ETHERSCAN = "https://sepolia.etherscan.io";

function short(v: string): string {
  return v.length > 14 ? `${v.slice(0, 8)}…${v.slice(-6)}` : v;
}

// Mono chip for an address / tx hash / agent id: copy-to-clipboard + an
// optional Etherscan link. Replaces the bare `font-mono` spans that used to
// overflow on mobile and offered no way to copy.
export function AddressChip({
  value,
  etherscan,
  truncate = true,
  className,
}: {
  value: string;
  etherscan?: { type: "tx" | "address"; hash: string };
  truncate?: boolean;
  className?: string;
}) {
  const [copied, setCopied] = useState(false);

  const copy = async () => {
    try {
      await navigator.clipboard.writeText(value);
      setCopied(true);
      setTimeout(() => setCopied(false), 1200);
    } catch {
      /* clipboard blocked — ignore */
    }
  };

  return (
    <span
      className={cn(
        "inline-flex items-center gap-1.5 rounded-md border bg-secondary/40 px-2 py-0.5 font-mono text-xs",
        className,
      )}
    >
      <span className="truncate" title={value}>
        {truncate ? short(value) : value}
      </span>
      <button
        aria-label="Copy"
        className="shrink-0 text-muted-foreground transition-colors hover:text-foreground"
        onClick={copy}
        type="button"
      >
        {copied ? (
          <Check className="text-signal" size={12} />
        ) : (
          <Copy size={12} />
        )}
      </button>
      {etherscan ? (
        <a
          aria-label="View on Etherscan"
          className="shrink-0 text-muted-foreground transition-colors hover:text-foreground"
          href={`${ETHERSCAN}/${etherscan.type}/${etherscan.hash}`}
          rel="noopener noreferrer"
          target="_blank"
        >
          <ExternalLink size={12} />
        </a>
      ) : null}
    </span>
  );
}
