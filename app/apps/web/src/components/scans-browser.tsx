"use client";

import type { FrameAnalysis, ScanImage, ScanRun } from "@robomoo/shared";
import { useCallback, useEffect, useRef, useState } from "react";
import { rpcClient } from "@/lib/orpc";

// Where the gs-pot reconstruction box is exposed (e.g. ngrok). Set in
// .env.local: NEXT_PUBLIC_GS_POT_URL=https://twilight-getting-possum.ngrok-free.dev
// When unset the "Build splat" button is hidden so the page still renders.
const GS_POT_URL = (process.env.NEXT_PUBLIC_GS_POT_URL ?? "").replace(/\/$/, "");

// Where mlxvlm is exposed (e.g. ngrok). Same pattern as GS_POT_URL — when
// unset, the in-browser "Analyze" buttons are hidden (auto-trigger on ingest
// still runs via the server's MLXVLM_URL env, independent of this).
const MLXVLM_URL = (process.env.NEXT_PUBLIC_MLXVLM_URL ?? "").replace(/\/$/, "");

type SplatState =
	| { status: "idle" }
	| { status: "submitting" }
	| {
			status: "queued";
			scanId: string;
			queueDepth: number;
			jobStatus: string; // queued | capturing | poses | training | pushing
			progress: number; // 0..1
			detail: string | null; // e.g. "downloading 5/20", "uploading 6.1 MB"
	  }
	| { status: "ready"; scanId: string; viewerUrl: string }
	| { status: "error"; message: string };

// "Build splat" button per run header.
// 1. POSTs body-less to gs-pot's /api/runs/<run_id>/process. gs-pot reads
//    robohack_base + ingest_token from its own env on the Mac, so neither
//    secret lands in browser-bundled code.
// 2. Polls gs-pot's /scans/<scan_id> every 3s for live progress.
// 3. When status flips to "ready", swaps to a "View splat ↗" link pointing
//    at gs-pot's local Spark viewer.
//
// Component state is in-memory only — a refresh loses it, but the trained
// splat still lands in the splats gallery (gs-pot POSTs it to
// /api/robot/splat at the end of training).
function BuildSplatButton({ runId }: { runId: string }) {
	const [state, setState] = useState<SplatState>({ status: "idle" });

	const pollingScanId = state.status === "queued" ? state.scanId : null;

	useEffect(() => {
		if (!pollingScanId) return;
		let cancelled = false;
		let timer: ReturnType<typeof setTimeout> | undefined;
		const tick = async () => {
			try {
				const r = await fetch(
					`${GS_POT_URL}/scans/${encodeURIComponent(pollingScanId)}`,
					{ headers: { "ngrok-skip-browser-warning": "true" } },
				);
				if (cancelled) return;
				if (!r.ok) {
					timer = setTimeout(tick, 3000);
					return;
				}
				const body = (await r.json()) as {
					status: string;
					progress: number;
					detail: string | null;
					error: string | null;
				};
				if (cancelled) return;
				if (body.status === "ready") {
					setState({
						status: "ready",
						scanId: pollingScanId,
						viewerUrl: `${GS_POT_URL}/web/?scene=/scenes/${pollingScanId}.ply`,
					});
					return;
				}
				if (body.status === "error") {
					setState({
						status: "error",
						message: body.error ?? "scan failed (see gs-pot logs)",
					});
					return;
				}
				setState((prev) =>
					prev.status === "queued"
						? {
								...prev,
								jobStatus: body.status,
								progress: body.progress ?? 0,
								detail: body.detail ?? null,
							}
						: prev,
				);
				timer = setTimeout(tick, 3000);
			} catch {
				if (!cancelled) timer = setTimeout(tick, 3000);
			}
		};
		tick();
		return () => {
			cancelled = true;
			if (timer) clearTimeout(timer);
		};
	}, [pollingScanId]);

	if (!GS_POT_URL) return null;

	const trigger = async () => {
		setState({ status: "submitting" });
		try {
			const r = await fetch(`${GS_POT_URL}/api/runs/${encodeURIComponent(runId)}/process`, {
				method: "POST",
				headers: {
					"Content-Type": "application/json",
					// Bypass ngrok-free's HTML interstitial on first hit so we get
					// the actual JSON response (which has gs-pot's CORS headers).
					"ngrok-skip-browser-warning": "true",
				},
				body: "{}",
			});
			if (!r.ok) {
				const text = await r.text().catch(() => "");
				throw new Error(`${r.status} ${r.statusText}${text ? `: ${text}` : ""}`);
			}
			const body = (await r.json()) as { scan_id: string; queue_depth: number };
			setState({
				status: "queued",
				scanId: body.scan_id,
				queueDepth: body.queue_depth,
				jobStatus: "queued",
				progress: 0,
				detail: null,
			});
		} catch (e) {
			setState({ status: "error", message: e instanceof Error ? e.message : String(e) });
		}
	};

	if (state.status === "ready") {
		return (
			<a
				className="shrink-0 rounded border bg-emerald-600 px-3 py-1 text-white text-xs hover:bg-emerald-700"
				href={state.viewerUrl}
				rel="noopener noreferrer"
				target="_blank"
			>
				View splat ↗
			</a>
		);
	}
	if (state.status === "queued") {
		const pct = Math.max(0, Math.min(100, Math.round(state.progress * 100)));
		const queueHint =
			state.queueDepth > 1 ? ` (#${state.queueDepth} in line)` : "";
		return (
			<span className="text-muted-foreground text-xs">
				{state.jobStatus}…{queueHint} {pct > 0 ? `${pct}%` : ""}
				{state.detail ? (
					<span className="ml-1 italic">· {state.detail}</span>
				) : null}
				<span className="ml-1 opacity-60">{state.scanId.slice(0, 16)}</span>
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

// POSTs every frame in a run to mlxvlm's /api/analyze-async (callback path —
// mlxvlm uploads the result back to the server out of band). We track per-frame
// job state in memory so the UI shows "12/36 analyzed". Doesn't block other
// runs and is safe to click again (overwrites old in-memory state, server keeps
// history).
type FrameJob =
	| { status: "queued"; jobId: string }
	| { status: "running"; jobId: string; progress: number; detail: string | null }
	| { status: "done"; jobId: string }
	| { status: "error"; message: string };

// POST a single frame to mlxvlm's /api/analyze-async. mlxvlm fetches the image
// from `imageUrl` (the presigned URL we already have for the gallery), runs
// Gemma + Falcon, and POSTs results back to robohack — callback_base /
// callback_token come from mlxvlm's own env (ROBOHACK_BASE/INGEST_TOKEN),
// never the browser bundle.
async function kickAnalyzeWithImage(
	frameId: string,
	imageUrl: string,
): Promise<string> {
	const r = await fetch(`${MLXVLM_URL}/api/analyze-async`, {
		method: "POST",
		headers: {
			"Content-Type": "application/json",
			"ngrok-skip-browser-warning": "true",
		},
		body: JSON.stringify({ frame_id: frameId, image_url: imageUrl }),
	});
	if (!r.ok) {
		const text = await r.text().catch(() => "");
		throw new Error(`${r.status} ${r.statusText}${text ? `: ${text}` : ""}`);
	}
	const body = (await r.json()) as { job_id: string };
	return body.job_id;
}

type JobStatusBody = {
	progress: number;
	detail: string | null;
	status: string;
	error: string | null;
};

// Polls mlxvlm's job status until it's no longer in flight. `onUpdate` is held
// in a ref so callers can use inline closures without restarting the poll on
// every re-render (the effect would otherwise re-fire on each state change the
// callback triggers, leaking pollers).
function useJobPoll(
	jobId: string | null,
	onUpdate: (s: JobStatusBody) => void,
) {
	const onUpdateRef = useRef(onUpdate);
	useEffect(() => {
		onUpdateRef.current = onUpdate;
	}, [onUpdate]);

	useEffect(() => {
		if (!jobId) return;
		let cancelled = false;
		let timer: ReturnType<typeof setTimeout> | undefined;
		const tick = async () => {
			try {
				const r = await fetch(
					`${MLXVLM_URL}/api/analyze-async/${encodeURIComponent(jobId)}`,
					{ headers: { "ngrok-skip-browser-warning": "true" } },
				);
				if (cancelled) return;
				if (!r.ok) {
					timer = setTimeout(tick, 3000);
					return;
				}
				const body = (await r.json()) as JobStatusBody;
				if (cancelled) return;
				onUpdateRef.current(body);
				if (body.status !== "done" && body.status !== "error") {
					timer = setTimeout(tick, 3000);
				}
			} catch {
				if (!cancelled) timer = setTimeout(tick, 3000);
			}
		};
		tick();
		return () => {
			cancelled = true;
			if (timer) clearTimeout(timer);
		};
	}, [jobId]);
}

// Per-run "Analyze room" — kicks an analyze job for every frame in the run.
// Job state is in-memory; counts update via the polling rpc (analysisCount on
// the scan image). Component is intentionally minimal: shows N/M analyzed.
function AnalyzeRoomButton({ images }: { images: ScanImage[] }) {
	const [submitting, setSubmitting] = useState(false);
	const [error, setError] = useState<string | null>(null);
	const [submitted, setSubmitted] = useState(0);

	if (!MLXVLM_URL) return null;

	const trigger = async () => {
		setSubmitting(true);
		setError(null);
		setSubmitted(0);
		let ok = 0;
		for (const img of images) {
			try {
				await kickAnalyzeWithImage(img.id, img.url);
				ok += 1;
				setSubmitted(ok);
			} catch (e) {
				setError(e instanceof Error ? e.message : String(e));
				break;
			}
		}
		setSubmitting(false);
	};

	const label = submitting
		? `Queuing ${submitted}/${images.length}…`
		: submitted > 0 && !error
			? `Re-analyze room (${submitted} queued)`
			: "Analyze room";

	return (
		<div className="flex flex-col items-end gap-1">
			<button
				className="shrink-0 rounded border bg-primary px-3 py-1 text-primary-foreground text-xs disabled:opacity-50"
				disabled={submitting || images.length === 0}
				onClick={trigger}
				type="button"
			>
				{label}
			</button>
			{error ? (
				<span className="max-w-xs text-destructive text-xs">{error}</span>
			) : null}
		</div>
	);
}

// Per-image lightbox panel: kicks one analyze job, polls progress, then fetches
// the latest stored analyses for the frame. Auto-loads existing analyses on
// mount so an already-analyzed frame shows results immediately.
function FrameAnalysisPanel({
	image,
	onDisplayUrl,
}: {
	image: ScanImage;
	onDisplayUrl: (url: string | null) => void;
}) {
	const [analyses, setAnalyses] = useState<FrameAnalysis[] | null>(null);
	const [loadError, setLoadError] = useState<string | null>(null);
	const [jobId, setJobId] = useState<string | null>(null);
	const [job, setJob] = useState<FrameJob | null>(null);
	const [overlay, setOverlay] = useState<"image" | "mask" | "crop">("image");
	const [activeObjectId, setActiveObjectId] = useState<string | null>(null);
	// Which analysis in the history list is being viewed. The list is sorted
	// newest-first by the server, so 0 = latest. Pinned by user clicks so a new
	// arrival doesn't yank them out of an older one they're reading.
	const [historyIdx, setHistoryIdx] = useState(0);
	const [pinnedToOld, setPinnedToOld] = useState(false);

	// Initial load + reload after a job finishes.
	const loadAnalyses = useCallback(async () => {
		try {
			const next = await rpcClient.frames.analyses({ frameIds: [image.id] });
			setAnalyses(next);
			setLoadError(null);
		} catch (e) {
			setLoadError(e instanceof Error ? e.message : "failed");
		}
	}, [image.id]);

	useEffect(() => {
		// Reset history selection whenever the lightbox jumps to a different frame.
		setHistoryIdx(0);
		setPinnedToOld(false);
		setActiveObjectId(null);
		setOverlay("image");
		void loadAnalyses();
	}, [loadAnalyses]);

	useJobPoll(jobId, (body) => {
		if (body.status === "done") {
			setJob({ status: "done", jobId: jobId ?? "" });
			// New analysis just landed — unless the user is pinned to an older
			// one, jump the view back to the freshest (now at index 0).
			if (!pinnedToOld) setHistoryIdx(0);
			void loadAnalyses();
		} else if (body.status === "error") {
			setJob({ status: "error", message: body.error ?? "analysis failed" });
		} else {
			setJob({
				status: "running",
				jobId: jobId ?? "",
				progress: body.progress ?? 0,
				detail: body.detail,
			});
		}
	});

	const trigger = async () => {
		try {
			setJob({ status: "queued", jobId: "" });
			const id = await kickAnalyzeWithImage(image.id, image.url);
			setJobId(id);
			setJob({ status: "queued", jobId: id });
		} catch (e) {
			setJob({ status: "error", message: e instanceof Error ? e.message : String(e) });
		}
	};

	const current = analyses?.[historyIdx] ?? analyses?.[0] ?? null;
	const latest = analyses?.[0] ?? null;
	const isOlder = analyses ? historyIdx > 0 && historyIdx < analyses.length : false;
	const activeObject =
		current?.objects.find((o) => o.id === activeObjectId) ?? null;

	// Clear the selected object whenever the active analysis changes; object ids
	// don't carry across analyses (each run gets fresh ids).
	const currentId = current?.id ?? null;
	useEffect(() => {
		setActiveObjectId(null);
		setOverlay("image");
	}, [currentId]);

	const overrideUrl =
		overlay === "mask" && activeObject?.maskUrl
			? activeObject.maskUrl
			: overlay === "crop" && activeObject?.cropUrl
				? activeObject.cropUrl
				: null;
	useEffect(() => {
		onDisplayUrl(overrideUrl);
	}, [overrideUrl, onDisplayUrl]);

	return (
		<div className="flex flex-col gap-3 text-white">
			<div className="flex flex-wrap items-center gap-2 text-xs">
				{MLXVLM_URL ? (
					<button
						className="rounded border border-white/40 bg-white/10 px-2 py-1 hover:bg-white/20 disabled:opacity-50"
						disabled={job?.status === "queued" || job?.status === "running"}
						onClick={(e) => {
							e.stopPropagation();
							void trigger();
						}}
						type="button"
					>
						{job?.status === "queued"
							? "Queued…"
							: job?.status === "running"
								? `Analyzing… ${Math.round((job.progress ?? 0) * 100)}%`
								: latest
									? "Re-analyze"
									: "Analyze"}
					</button>
				) : null}
				{job?.status === "running" && job.detail ? (
					<span className="italic opacity-70">· {job.detail}</span>
				) : null}
				{job?.status === "error" ? (
					<span className="text-rose-400">· {job.message}</span>
				) : null}
				{analyses && analyses.length > 1 ? (
					<span className="opacity-60">
						· {analyses.length} analyses on file
					</span>
				) : null}
			</div>

			{loadError ? (
				<span className="text-rose-400 text-xs">{loadError}</span>
			) : null}

			{/* History strip — one pill per analysis, newest on the left. Clicking
			    "pins" the user to that older entry; the panel won't yank them back to
			    a freshly-arrived analysis. The "↻ latest" button unpins. */}
			{analyses && analyses.length > 1 ? (
				<div
					className="flex flex-wrap items-center gap-1 text-xs"
					onClick={(e) => e.stopPropagation()}
					onKeyDown={(e) => e.stopPropagation()}
				>
					<span className="text-white/60">History:</span>
					{analyses.map((a, idx) => {
						const isActive = idx === historyIdx;
						const label =
							idx === 0
								? "latest"
								: new Date(a.createdAt).toLocaleTimeString([], {
										hour: "2-digit",
										minute: "2-digit",
									});
						return (
							<button
								className={`rounded border px-2 py-0.5 ${
									isActive
										? "border-sky-400 bg-sky-500/20 text-white"
										: "border-white/30 bg-white/5 text-white/80 hover:bg-white/15"
								}`}
								key={a.id}
								onClick={(e) => {
									e.stopPropagation();
									setHistoryIdx(idx);
									setPinnedToOld(idx > 0);
								}}
								type="button"
								title={new Date(a.createdAt).toLocaleString()}
							>
								{label}
								{a.objects.length > 0 ? (
									<span className="ml-1 opacity-60">·{a.objects.length}</span>
								) : null}
							</button>
						);
					})}
					{isOlder ? (
						<button
							className="ml-1 rounded border border-emerald-400/40 px-2 py-0.5 text-emerald-300 hover:bg-emerald-500/10"
							onClick={(e) => {
								e.stopPropagation();
								setHistoryIdx(0);
								setPinnedToOld(false);
							}}
							type="button"
						>
							↻ latest
						</button>
					) : null}
				</div>
			) : null}

			{current ? (
				<div
					className="flex flex-col gap-2 rounded bg-black/40 p-3 text-sm"
					onClick={(e) => e.stopPropagation()}
					onKeyDown={(e) => e.stopPropagation()}
				>
					{isOlder ? (
						<span className="self-start rounded bg-amber-500/20 px-2 py-0.5 text-[10px] text-amber-200 uppercase tracking-wide">
							viewing older analysis
						</span>
					) : null}

					{current.summary ? (
						<p className="text-white/90">{current.summary}</p>
					) : current.description ? (
						<p className="text-white/80">{current.description}</p>
					) : null}

					{current.objects.length > 0 ? (
						<div className="flex flex-col gap-1">
							<span className="text-white/60 text-xs uppercase tracking-wide">
								Objects ({current.objects.length})
							</span>
							<div className="flex flex-wrap gap-1">
								{current.objects.map((o) => (
									<button
										className={`rounded border px-2 py-0.5 text-xs ${
											o.id === activeObjectId
												? "border-emerald-400 bg-emerald-500/20"
												: "border-white/30 bg-white/5 hover:bg-white/15"
										}`}
										key={o.id}
										onClick={(e) => {
											e.stopPropagation();
											setActiveObjectId(o.id === activeObjectId ? null : o.id);
											setOverlay("image");
										}}
										type="button"
									>
										{o.label ?? o.query ?? "object"}
										{o.maskArea ? (
											<span className="ml-1 opacity-60">
												·{Math.round(o.maskArea / 1000)}k
											</span>
										) : null}
									</button>
								))}
							</div>
						</div>
					) : null}

					{activeObject ? (
						<div className="flex flex-wrap gap-1 text-xs">
							<span className="text-white/60">View:</span>
							<button
								className={`rounded px-2 py-0.5 ${overlay === "image" ? "bg-white/20" : "bg-white/5 hover:bg-white/10"}`}
								onClick={(e) => {
									e.stopPropagation();
									setOverlay("image");
								}}
								type="button"
							>
								frame
							</button>
							{activeObject.cropUrl ? (
								<button
									className={`rounded px-2 py-0.5 ${overlay === "crop" ? "bg-white/20" : "bg-white/5 hover:bg-white/10"}`}
									onClick={(e) => {
										e.stopPropagation();
										setOverlay("crop");
									}}
									type="button"
								>
									crop
								</button>
							) : null}
							{activeObject.maskUrl ? (
								<button
									className={`rounded px-2 py-0.5 ${overlay === "mask" ? "bg-white/20" : "bg-white/5 hover:bg-white/10"}`}
									onClick={(e) => {
										e.stopPropagation();
										setOverlay("mask");
									}}
									type="button"
								>
									mask
								</button>
							) : null}
						</div>
					) : null}

					{current.texts.length > 0 ? (
						<details className="text-white/70 text-xs">
							<summary className="cursor-pointer opacity-80">
								Raw texts ({current.texts.length})
							</summary>
							<ul className="mt-1 flex flex-col gap-1">
								{current.texts.map((t) => (
									<li
										className="rounded bg-white/5 px-2 py-1 font-mono text-[11px] leading-snug"
										// biome-ignore lint/suspicious/noArrayIndexKey: read-only list, stable order
										key={t}
									>
										{t}
									</li>
								))}
							</ul>
						</details>
					) : null}

					{current.model ? (
						<span className="text-white/40 text-[10px]">
							{current.model} · {new Date(current.createdAt).toLocaleString()}
						</span>
					) : null}
				</div>
			) : !job ? (
				<p className="text-white/60 text-xs">
					No analysis yet for this frame.
					{MLXVLM_URL ? " Click Analyze to run Gemma + Falcon." : ""}
				</p>
			) : null}
		</div>
	);
}

// Browse robot room scans grouped run → position → angle-sorted images. Each run
// is a scan; each position is one 360° panorama (~36 photos). Polls every 5s so
// a scan in progress fills in live. Click a thumbnail for a lightbox.
export function ScansBrowser() {
	const [runs, setRuns] = useState<ScanRun[]>([]);
	const [error, setError] = useState<string | null>(null);
	const [selected, setSelected] = useState<ScanImage | null>(null);
	// Override of the lightbox image source — driven by the analysis panel's
	// crop/mask toggle. Reset whenever the lightbox closes or a new image opens
	// (handled at the thumbnail-click and dismiss sites).
	const [lightboxOverride, setLightboxOverride] = useState<string | null>(null);

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
							<div className="flex items-center gap-2">
								<AnalyzeRoomButton
									images={run.positions.flatMap((p) => p.images)}
								/>
								<BuildSplatButton runId={run.run} />
							</div>
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
											onClick={() => {
												setLightboxOverride(null);
												setSelected(img);
											}}
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
											{img.analysisCount > 0 ? (
												<span
													className="absolute top-1 right-1 h-2 w-2 rounded-full bg-emerald-400 shadow ring-1 ring-emerald-900/40"
													title={`${img.analysisCount} analysis(es)`}
												/>
											) : null}
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
					className="fixed inset-0 z-50 flex flex-col items-center justify-start gap-3 overflow-y-auto bg-black/80 p-4"
					onClick={() => {
						setSelected(null);
						setLightboxOverride(null);
					}}
				>
					{/* biome-ignore lint/performance/noImgElement: presigned URL, plain img */}
					<img
						alt={`angle ${selected.angle ?? "?"}`}
						className="max-h-[70vh] max-w-full object-contain"
						src={lightboxOverride ?? selected.url}
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
						{lightboxOverride ? (
							<button
								className="rounded border border-white/30 px-2 py-0.5"
								onClick={(e) => {
									e.stopPropagation();
									setLightboxOverride(null);
								}}
								type="button"
							>
								reset view
							</button>
						) : null}
					</div>
					<div
						className="w-full max-w-3xl"
						onClick={(e) => e.stopPropagation()}
						onKeyDown={(e) => e.stopPropagation()}
					>
						<FrameAnalysisPanel
							image={selected}
							onDisplayUrl={setLightboxOverride}
						/>
					</div>
					<span className="text-white/50 text-xs">
						click the dark background to close
					</span>
				</div>
			) : null}
		</>
	);
}
