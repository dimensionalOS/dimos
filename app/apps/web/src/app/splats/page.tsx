import Link from "next/link";
import { SplatViewer } from "@/components/splat-viewer";

export const dynamic = "force-dynamic";

export default function SplatsPage() {
  return (
    <main className="mx-auto flex max-w-5xl flex-col gap-6 px-4 py-12">
      <header className="flex flex-col gap-1">
        <h1 className="font-bold text-3xl tracking-tight">3D splats</h1>
        <p className="text-muted-foreground text-sm">
          Walk through 3D Gaussian splats reconstructed from the robot&apos;s
          photos. Pick a demo scene below — robot captures POSTed to{" "}
          <code>/api/robot/splat</code> appear here automatically.
        </p>
        <div className="flex gap-3 text-sm underline">
          <Link href="/">← home</Link>
          <Link href="/frames">all frames</Link>
        </div>
      </header>

      <SplatViewer />
    </main>
  );
}
