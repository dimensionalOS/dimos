import Link from "next/link";
import { FramesGallery } from "@/components/frames-gallery";

export const dynamic = "force-dynamic";

export default function FramesPage() {
  return (
    <main className="mx-auto flex max-w-4xl flex-col gap-6 px-4 py-12">
      <header className="flex flex-col gap-1">
        <h1 className="font-bold text-3xl tracking-tight">Robot frames</h1>
        <p className="text-muted-foreground text-sm">
          Images the robot captured and POSTed to{" "}
          <code>/api/robot/frame</code>. Live — polls every few seconds.
        </p>
        <Link className="text-sm underline" href="/">
          ← back
        </Link>
      </header>

      <FramesGallery />
    </main>
  );
}
