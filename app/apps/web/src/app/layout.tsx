import type { Metadata } from "next";
import type { ReactNode } from "react";
import { SiteNav } from "@/components/site-nav";
import { WalletProvider } from "@/lib/wallet";
import "./globals.css";

export const metadata: Metadata = {
  title: "robomoo — hire autonomous robot agents",
  description:
    "A marketplace of hireable robot agents with on-chain identity and reputation. Hire RoboDoc to scan your rooms and build a 3D virtual tour.",
};

export default function RootLayout({ children }: { children: ReactNode }) {
  return (
    <html lang="en">
      <body className="min-h-screen bg-background text-foreground antialiased">
        <WalletProvider>
          <SiteNav />
          {children}
        </WalletProvider>
      </body>
    </html>
  );
}
