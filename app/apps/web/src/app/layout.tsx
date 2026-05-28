import type { Metadata } from "next";
import {
  Bricolage_Grotesque,
  Hanken_Grotesk,
  JetBrains_Mono,
} from "next/font/google";
import type { ReactNode } from "react";
import { SiteNav } from "@/components/site-nav";
import { TooltipProvider } from "@/components/ui/tooltip";
import { WalletProvider } from "@/lib/wallet";
import "./globals.css";

const display = Bricolage_Grotesque({
  subsets: ["latin"],
  variable: "--font-bricolage",
  weight: ["600", "700", "800"],
});

const sans = Hanken_Grotesk({
  subsets: ["latin"],
  variable: "--font-hanken",
});

const mono = JetBrains_Mono({
  subsets: ["latin"],
  variable: "--font-jetbrains",
});

export const metadata: Metadata = {
  title: "robomoo — hire autonomous robot agents",
  description:
    "A marketplace of hireable robot agents with verified identities and earned reputation. Hire RoboDoc to scan your rooms and build a 3D virtual tour.",
};

export default function RootLayout({ children }: { children: ReactNode }) {
  return (
    <html
      lang="en"
      className={`dark ${display.variable} ${sans.variable} ${mono.variable}`}
    >
      <body className="min-h-screen bg-background text-foreground antialiased">
        <WalletProvider>
          <TooltipProvider delayDuration={150}>
            <SiteNav />
            {children}
          </TooltipProvider>
        </WalletProvider>
      </body>
    </html>
  );
}
