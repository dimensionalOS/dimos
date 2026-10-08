// Battery page: the pack's state from a battery.json.v1 latest channel, the
// charge plotted over this run and the time left at the recent drain. The
// history is the page's own (Tabs unmounts inactive pages), like Stats.

import { useEffect, useRef, useState } from "react";
import type { PanelSpec } from "@dimos/shared";
import type { ChannelStore } from "@dimos/sdk";
import { useStoreChannel } from "@dimos/sdk/react";
import { Badge, PanelFrame } from "../layout/PanelFrame.tsx";
import type { PanelProps } from "./registry.tsx";
import {
  type BatteryHistory,
  type BatterySample,
  EMPTY_HISTORY,
  estimateRemainingS,
  fmtClock,
  fmtDuration,
  foldBattery,
} from "./battery.ts";
import styles from "./BatteryPanel.module.css";

/** The Go2 pushes lowstate about once a second. */
export const BATTERY_STALE_MS = 5000;

function useBatteryHistory(store: ChannelStore, ch: string): BatteryHistory {
  const [history, setHistory] = useState(EMPTY_HISTORY);
  useEffect(() => {
    let seen = 0;
    let current = EMPTY_HISTORY;
    const pull = (): void => {
      const slot = store.get(ch);
      if (slot === null) {
        if (seen === 0) return;
        seen = 0;
        current = EMPTY_HISTORY;
      } else if (slot.version > seen) {
        seen = slot.version;
        current = foldBattery(current, slot.value);
      } else {
        return;
      }
      setHistory(current);
    };
    const unsubscribe = store.subscribe(ch, pull);
    pull();
    return unsubscribe;
  }, [store, ch]);
  return history;
}

const COLORS = { ok: "#3fb950", warn: "#d29922", danger: "#ff5c5c" };

function level(pct: number): keyof typeof COLORS {
  return pct > 30 ? "ok" : pct > 15 ? "warn" : "danger";
}

export function drawBatteryChart(
  ctx: CanvasRenderingContext2D,
  samples: readonly BatterySample[],
  remainingS: number | null,
  w: number,
  h: number,
  dpr: number,
): void {
  ctx.clearRect(0, 0, w, h);
  const padL = 34 * dpr, padR = 10 * dpr, padT = 10 * dpr, padB = 18 * dpr;
  const plotW = w - padL - padR, plotH = h - padT - padB;
  const last = samples[samples.length - 1];
  const span = Math.max(60, (last?.t ?? 0) + (remainingS ?? 0));
  const x = (t: number): number => padL + (plotW * t) / span;
  const y = (pct: number): number => padT + plotH * (1 - Math.min(Math.max(pct, 0), 100) / 100);
  ctx.font = `${10 * dpr}px ui-monospace, monospace`;
  ctx.fillStyle = "#5f6873";
  ctx.strokeStyle = "rgba(255,255,255,0.08)";
  ctx.lineWidth = dpr;
  ctx.textAlign = "right";
  ctx.textBaseline = "middle";
  for (const pct of [0, 25, 50, 75, 100]) {
    ctx.beginPath();
    ctx.moveTo(padL, y(pct));
    ctx.lineTo(w - padR, y(pct));
    ctx.stroke();
    ctx.fillText(`${pct}`, padL - 6 * dpr, y(pct));
  }
  ctx.textAlign = "center";
  ctx.textBaseline = "top";
  const minutes = Math.ceil(span / 60);
  const step = minutes <= 10 ? 1 : minutes <= 30 ? 5 : minutes <= 120 ? 15 : 60;
  for (let m = 0; m <= minutes; m += step) {
    ctx.fillText(`${m}m`, x(m * 60), h - padB + 5 * dpr);
  }
  if (last === undefined) return;
  const color = COLORS[level(last.pct)];
  ctx.strokeStyle = color;
  ctx.fillStyle = color;
  ctx.lineWidth = 2 * dpr;
  ctx.lineJoin = "round";
  ctx.beginPath();
  ctx.moveTo(x(samples[0].t), y(samples[0].pct));
  for (let i = 1; i < samples.length; i++) ctx.lineTo(x(samples[i].t), y(samples[i].pct));
  ctx.stroke();
  ctx.lineTo(x(last.t), y(0));
  ctx.lineTo(x(samples[0].t), y(0));
  ctx.closePath();
  ctx.globalAlpha = 0.15;
  ctx.fill();
  ctx.globalAlpha = 1;
  if (remainingS !== null) {
    ctx.setLineDash([4 * dpr, 4 * dpr]);
    ctx.lineWidth = 1.5 * dpr;
    ctx.beginPath();
    ctx.moveTo(x(last.t), y(last.pct));
    ctx.lineTo(x(last.t + remainingS), y(0));
    ctx.stroke();
    ctx.setLineDash([]);
  }
  ctx.beginPath();
  ctx.arc(x(last.t), y(last.pct), 3 * dpr, 0, 2 * Math.PI);
  ctx.fill();
}

function Chart({ samples, remainingS }: {
  samples: readonly BatterySample[];
  remainingS: number | null;
}) {
  const ref = useRef<HTMLCanvasElement | null>(null);
  useEffect(() => {
    const canvas = ref.current;
    const ctx = canvas === null ? null : canvas.getContext("2d");
    if (canvas === null || ctx === null) return;
    const dpr = globalThis.devicePixelRatio || 1;
    const w = Math.round(canvas.clientWidth * dpr) || canvas.width;
    const h = Math.round(canvas.clientHeight * dpr) || canvas.height;
    if (canvas.width !== w || canvas.height !== h) {
      canvas.width = w;
      canvas.height = h;
    }
    drawBatteryChart(ctx, samples, remainingS, w, h, dpr);
  }, [samples, remainingS]);
  return (
    <div className={styles.chart}>
      <canvas ref={ref} className={styles.canvas} width={600} height={200} aria-hidden="true" />
    </div>
  );
}

function Card({ label, value, tone, big, testId }: {
  label: string;
  value: string;
  tone?: keyof typeof COLORS;
  big?: boolean;
  testId?: string;
}) {
  const cls = [styles.value, big ? styles.big : "", tone ? styles[tone] : ""].join(" ");
  return (
    <div className={styles.card}>
      <span className={styles.label}>{label}</span>
      <span className={cls} data-testid={testId}>{value}</span>
    </div>
  );
}

function BatteryPage({ spec, store }: { spec: PanelSpec; store: ChannelStore }) {
  const ch = spec.channels[0];
  const history = useBatteryHistory(store, ch);
  const { stats } = useStoreChannel(store, ch);
  const stale = stats.ageMs !== null && stats.ageMs > BATTERY_STALE_MS;
  const badge = (
    <Badge
      store={store}
      ch={ch}
      staleMs={BATTERY_STALE_MS}
      unit="Hz"
      testId={`battery-${ch}-badge`}
    />
  );
  const frame = history.frame;
  const last = history.samples[history.samples.length - 1];
  const remainingS = estimateRemainingS(history.samples);
  const pct = last?.pct ?? null;
  return (
    <PanelFrame spec={spec} badge={badge}>
      <div className={styles.page} data-testid={`battery-${ch}`} data-stale={stale || undefined}>
        {history.unreadable && (
          <div className={styles.error} role="alert">unreadable battery frame</div>
        )}
        {frame === null ? <span className={styles.hint}>waiting for battery state...</span> : (
          <>
            <div className={styles.cards}>
              <Card
                label="charge"
                value={pct === null ? "-" : `${pct.toFixed(0)} %`}
                tone={pct === null ? undefined : level(pct)}
                big
                testId={`battery-${ch}-charge`}
              />
              <Card
                label="time left"
                value={remainingS === null ? "estimating..." : `~${fmtDuration(remainingS)}`}
                tone={remainingS !== null && remainingS < 600 ? "danger" : undefined}
                big
                testId={`battery-${ch}-left`}
              />
              <Card label="run time" value={fmtClock(last?.t ?? 0)} />
              <Card
                label="voltage"
                value={frame.voltage === null ? "-" : `${frame.voltage.toFixed(1)} V`}
              />
              <Card
                label="current"
                value={frame.current === null ? "-" : `${frame.current.toFixed(1)} A`}
              />
              <Card
                label="temperature"
                value={frame.temperature === null ? "-" : `${frame.temperature.toFixed(0)} °C`}
              />
            </div>
            <Chart samples={history.samples} remainingS={remainingS} />
          </>
        )}
      </div>
    </PanelFrame>
  );
}

export function BatteryPanel({ spec, store }: PanelProps) {
  return <BatteryPage spec={spec} store={store} />;
}
