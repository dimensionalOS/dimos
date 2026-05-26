import Link from "next/link";
import { MapView } from "@/components/map-view";

export const dynamic = "force-dynamic";

export default function MapPage() {
  return (
    <main className="mx-auto flex max-w-4xl flex-col gap-6 px-4 py-12">
      <header className="flex flex-col gap-1">
        <h1 className="font-bold text-3xl tracking-tight">Robot map</h1>
        <p className="text-muted-foreground text-sm">
          The robot's 2D occupancy map with a pin where each photo was captured.
          Live — polls every few seconds.
        </p>
        <div className="flex gap-3 text-sm underline">
          <Link href="/">← home</Link>
          <Link href="/frames">all frames</Link>
        </div>
      </header>

      <MapView />
    </main>
  );
}
