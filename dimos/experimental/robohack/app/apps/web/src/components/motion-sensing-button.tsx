"use client";

import { useEffect, useState } from "react";

// Where gs-pot lives (e.g. ngrok). Same env as the Build splat button.
const GS_POT_URL = (process.env.NEXT_PUBLIC_GS_POT_URL ?? "").replace(/\/$/, "");

type MotionStatus = {
	running: boolean;
	state: "quiet" | "motion" | null;
	state_since: string | null;
	last_sample: number | null;
	threshold: number | null;
};

type ButtonState =
	| { kind: "idle" } // no status fetched yet
	| { kind: "off" }
	| { kind: "on"; status: MotionStatus }
	| { kind: "starting" }
	| { kind: "stopping" }
	| { kind: "error"; message: string };

const fetchOpts = (init: RequestInit = {}): RequestInit => ({
	...init,
	headers: {
		"Content-Type": "application/json",
		"ngrok-skip-browser-warning": "true",
		...(init.headers ?? {}),
	},
});

/**
 * "Start sensing" button — POSTs to gs-pot's /api/motion/start. While the
 * monitor is running gs-pot polls the ESP32 nodes' motion_band_power, and on
 * a quiet→motion flip the robot barks. The button polls /api/motion/status
 * every 3s and shows live state (quiet | motion) + a Stop button when running.
 *
 * Hidden when NEXT_PUBLIC_GS_POT_URL isn't baked in.
 */
export function MotionSensingButton() {
	const [state, setState] = useState<ButtonState>({ kind: "idle" });

	// Refresh status on mount + every 3s so the button reflects gs-pot truth
	// (someone may have started it from curl / another browser).
	useEffect(() => {
		if (!GS_POT_URL) return;
		let cancelled = false;
		let timer: ReturnType<typeof setTimeout> | undefined;
		const tick = async () => {
			try {
				const r = await fetch(`${GS_POT_URL}/api/motion/status`, fetchOpts());
				if (cancelled) return;
				if (!r.ok) {
					timer = setTimeout(tick, 3000);
					return;
				}
				const body = (await r.json()) as MotionStatus;
				if (cancelled) return;
				setState((prev) => {
					if (prev.kind === "starting" || prev.kind === "stopping") return prev;
					return body.running ? { kind: "on", status: body } : { kind: "off" };
				});
			} catch {
				/* network blip; keep polling */
			}
			if (!cancelled) timer = setTimeout(tick, 3000);
		};
		tick();
		return () => {
			cancelled = true;
			if (timer) clearTimeout(timer);
		};
	}, []);

	if (!GS_POT_URL) return null;

	const start = async () => {
		setState({ kind: "starting" });
		try {
			const r = await fetch(
				`${GS_POT_URL}/api/motion/start`,
				fetchOpts({ method: "POST", body: "{}" }),
			);
			if (!r.ok) {
				const text = await r.text().catch(() => "");
				throw new Error(`${r.status} ${r.statusText}${text ? `: ${text}` : ""}`);
			}
			// Don't bother parsing — the 3s poll will pick up "running: true".
		} catch (e) {
			setState({ kind: "error", message: e instanceof Error ? e.message : String(e) });
		}
	};

	const stop = async () => {
		setState({ kind: "stopping" });
		try {
			const r = await fetch(
				`${GS_POT_URL}/api/motion/stop`,
				fetchOpts({ method: "POST" }),
			);
			if (!r.ok) {
				const text = await r.text().catch(() => "");
				throw new Error(`${r.status} ${r.statusText}${text ? `: ${text}` : ""}`);
			}
		} catch (e) {
			setState({ kind: "error", message: e instanceof Error ? e.message : String(e) });
		}
	};

	if (state.kind === "on") {
		const live = state.status.state ?? "quiet";
		const sample = state.status.last_sample;
		const threshold = state.status.threshold;
		return (
			<div className="flex items-center gap-2">
				<span
					className={`rounded-full px-2 py-0.5 text-xs ${
						live === "motion"
							? "bg-amber-500/20 text-amber-700"
							: "bg-emerald-500/20 text-emerald-700"
					}`}
				>
					{live === "motion" ? "● motion" : "● quiet"}
				</span>
				{sample !== null && threshold !== null ? (
					<span className="text-muted-foreground text-xs tabular-nums">
						{sample.toFixed(2)} / {threshold.toFixed(2)}
					</span>
				) : null}
				<button
					className="rounded border bg-destructive px-3 py-1 text-destructive-foreground text-xs"
					onClick={stop}
					type="button"
				>
					Stop sensing
				</button>
			</div>
		);
	}
	if (state.kind === "error") {
		return (
			<div className="flex flex-col items-end gap-1">
				<button
					className="rounded border border-destructive px-2 py-1 text-destructive text-xs"
					onClick={start}
					type="button"
				>
					Retry start sensing
				</button>
				<span className="max-w-xs text-destructive text-xs">{state.message}</span>
			</div>
		);
	}
	const busy = state.kind === "starting" || state.kind === "stopping";
	return (
		<button
			className="rounded border bg-primary px-3 py-1 text-primary-foreground text-xs disabled:opacity-50"
			disabled={busy || state.kind === "idle"}
			onClick={start}
			type="button"
		>
			{state.kind === "starting"
				? "Starting…"
				: state.kind === "stopping"
					? "Stopping…"
					: "Start sensing"}
		</button>
	);
}
