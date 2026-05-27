"use client";

import type { ScanImage, ScanRun } from "@robomoo/shared";
import { useEffect, useState } from "react";
import { rpcClient } from "@/lib/orpc";

// Where the gs-pot reconstruction box is exposed (e.g. ngrok). Set in
// .env.local: NEXT_PUBLIC_GS_POT_URL=https://twilight-getting-possum.ngrok-free.dev
// When unset the "Build splat" button is hidden so the page still renders.
const GS_POT_URL = (process.env.NEXT_PUBLIC_GS_POT_URL ?? "").replace(/\/$/, "");

type SplatState =
	| { status: "idle" }
	| { status: "submitting" }
	| { status: "queued"; scanId: string; queueDepth: number }
	| { status: "error"; message: string };

// "Build splat" button per run header. Fires the gs-pot webhook with an empty
// body; gs-pot reads its robohack_base + ingest_token from env on the Mac so
// no secrets land in browser-bundled code. State is local to the component
// (refresh loses it — the resulting splat will still appear in the splats
// gallery once gs-pot pushes it back).
function BuildSplatButton({ runId }: { runId: string }) {
	const [state, setState] = useState<SplatState>({ status: "idle" });

	if (!GS_POT_URL) return null;

	const trigger = async () => {
		setState({ status: "submitting" });
		try {
			const r = await fetch(`${GS_POT_URL}/api/runs/${encodeURIComponent(runId)}/process`, {
				method: "POST",
				headers: { "Content-Type": "application/json" },
				body: "{}",
			});
			if (!r.ok) {
				const text = await r.text().catch(() => "");
				throw new Error(`${r.status} ${r.statusText}${text ? `: ${text}` : ""}`);
			}
			const body = (await r.json()) as { scan_id: string; queue_depth: number };
			setState({ status: "queued", scanId: body.scan_id, queueDepth: body.queue_depth });
		} catch (e) {
			setState({ status: "error", message: e instanceof Error ? e.message : String(e) });
		}
	};

	if (state.status === "queued") {
		return (
			<span className="text-muted-foreground text-xs">
				✓ queued as {state.scanId}
				{state.queueDepth > 1 ? ` (#${state.queueDepth} in line)` : ""} — splat will appear
				once training finishes
			</span>
		);
	}
	if (state.status === "error") {
		return (
			<div className="flex flex-col items-end gap-1">
				<button
					className="rounded border border-destructive px-2 py-1 text-destructive text-xs"
					onClick={trigger}
					type="button"
				>
					Retry build splat
				</button>
				<span className="max-w-xs text-destructive text-xs">{state.message}</span>
			</div>
		);
	}
	return (
		<button
			className="shrink-0 rounded border bg-primary px-3 py-1 text-primary-foreground text-xs disabled:opacity-50"
			disabled={state.status === "submitting"}
			onClick={trigger}
			type="button"
		>
			{state.status === "submitting" ? "Submitting…" : "Build splat"}
		</button>
	);
}

// Browse robot room scans grouped run → position → angle-sorted images. Each run
// is a scan; each position is one 360° panorama (~36 photos). Polls every 5s so
// a scan in progress fills in live. Click a thumbnail for a lightbox.
export function ScansBrowser() {
	const [runs, setRuns] = useState<ScanRun[]>([]);
	const [error, setError] = useState<string | null>(null);
	const [selected, setSelected] = useState<ScanImage | null>(null);

	useEffect(() => {
		let active = true;
		const load = async () => {
			try {
				const next = await rpcClient.frames.scans();
				if (active) {
					setRuns(next);
					setError(null);
				}
			} catch (e) {
				if (active) setError(e instanceof Error ? e.message : "failed to load");
			}
		};
		load();
		const t = setInterval(load, 5000);
		return () => {
			active = false;
			clearInterval(t);
		};
	}, []);

	if (error) {
		return <p className="text-destructive text-sm">Error: {error}</p>;
	}
	if (runs.length === 0) {
		return (
			<p className="text-muted-foreground text-sm">
				No scans yet. Ask the robot to <code>room_scan</code>, or fetch{" "}
				<code>/api/scans</code>.
			</p>
		);
	}

	return (
		<>
			<div className="flex flex-col gap-8">
				{runs.map((run) => (
					<section className="flex flex-col gap-3" key={run.run}>
						<header className="flex items-start justify-between gap-3">
							<div className="flex flex-col gap-0.5">
								<h2 className="font-semibold text-lg">{run.run}</h2>
								<p className="text-muted-foreground text-xs">
									{run.positionCount} positions · {run.imageCount} images ·{" "}
									{new Date(run.capturedAt).toLocaleString()}
								</p>
							</div>
							<BuildSplatButton runId={run.run} />
						</header>

						{run.positions.map((pos) => (
							<div
								className="flex flex-col gap-1"
								key={`${run.run}-${pos.position}`}
							>
								<span className="text-muted-foreground text-xs">
									Position {pos.position ?? "—"}
									{pos.poseX !== null && pos.poseY !== null
										? ` · (${pos.poseX.toFixed(2)}, ${pos.poseY.toFixed(2)})`
										: ""}{" "}
									· {pos.images.length} images
								</span>
								<div className="flex gap-2 overflow-x-auto pb-2">
									{pos.images.map((img) => (
										<button
											className="relative shrink-0 cursor-zoom-in"
											key={img.id}
											onClick={() => setSelected(img)}
											type="button"
										>
											{/* biome-ignore lint/performance/noImgElement: presigned URLs rotate; plain img is simplest */}
											<img
												alt={`angle ${img.angle ?? "?"}`}
												className="h-24 w-32 rounded border object-cover transition-opacity hover:opacity-90"
												src={img.url}
											/>
											<span className="absolute right-0 bottom-0 bg-black/60 px-1 text-[10px] text-white">
												{img.angle ?? "?"}°
											</span>
										</button>
									))}
								</div>
							</div>
						))}
					</section>
				))}
			</div>

			{selected ? (
				// biome-ignore lint/a11y/useKeyWithClickEvents: simple click-to-dismiss lightbox
				// biome-ignore lint/a11y/noStaticElementInteractions: simple click-to-dismiss lightbox
				<div
					className="fixed inset-0 z-50 flex flex-col items-center justify-center gap-3 bg-black/80 p-4"
					onClick={() => setSelected(null)}
				>
					{/* biome-ignore lint/performance/noImgElement: presigned URL, plain img */}
					<img
						alt={`angle ${selected.angle ?? "?"}`}
						className="max-h-[85vh] max-w-full object-contain"
						src={selected.url}
					/>
					<div className="flex items-center gap-3 text-white/80 text-xs">
						<span>angle {selected.angle ?? "?"}°</span>
						<a
							className="underline"
							href={selected.url}
							onClick={(e) => e.stopPropagation()}
							rel="noopener noreferrer"
							target="_blank"
						>
							open original ↗
						</a>
					</div>
					<span className="text-white/50 text-xs">click anywhere to close</span>
				</div>
			) : null}
		</>
	);
}
