import Link from "next/link";
import { RobotControls } from "@/components/robot-controls";

export const dynamic = "force-dynamic";

export default function ControlPage() {
  return (
    <main className="mx-auto flex max-w-4xl flex-col gap-6 px-4 py-12">
      <header className="flex flex-col gap-1">
        <h1 className="font-bold text-3xl tracking-tight">Robot control</h1>
        <p className="text-muted-foreground text-sm">
          Send a command to the robot's agent — it runs it just like{" "}
          <code>dimos agent-send</code>.
        </p>
        <div className="flex gap-3 text-sm underline">
          <Link href="/">← home</Link>
          <Link href="/map">map</Link>
        </div>
      </header>

      <RobotControls />
    </main>
  );
}
