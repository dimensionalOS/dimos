import Link from "next/link";
import { ScansBrowser } from "@/components/scans-browser";

export const dynamic = "force-dynamic";

export default function ScansPage() {
	return (
		<main className="mx-auto flex max-w-5xl flex-col gap-6 px-4 py-12">
			<header className="flex flex-col gap-1">
				<h1 className="font-bold text-3xl tracking-tight">Room scans</h1>
				<p className="text-muted-foreground text-sm">
					The robot&apos;s <code>room_scan</code> sweeps, grouped by run and
					position (one 360° panorama per stop). Also available as JSON at{" "}
					<code>/api/scans</code> (or <code>/api/scans/:run</code>).
				</p>
				<div className="flex gap-3 text-sm underline">
					<Link href="/">← back</Link>
					<Link href="/frames">→ robot frames</Link>
				</div>
			</header>

			<ScansBrowser />
		</main>
	);
}
