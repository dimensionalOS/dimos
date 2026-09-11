import { useEffect, useLayoutEffect, useRef, useState, type ReactNode, type CSSProperties, type PointerEvent } from "react";
import type { PanelSpec } from "@dimos/shared";
import { useWorkspace } from "./Workspace.tsx";
export { Badge, type DrawHealth } from "./Badge.tsx";
type Rect = { x: number; y: number; width: number; height: number };
let top = 30;
export function PanelFrame({ spec, badge, children }: { spec: PanelSpec; badge?: ReactNode; children: ReactNode }) {
  const frame = useRef<HTMLElement>(null);
  const workspace = useWorkspace();
  const [dockHeight, setDockHeight] = useState<number>();
  const [rect, setRect] = useState<Rect | null>(null);
  const [max, setMax] = useState(false);
  const [hidden, setHidden] = useState(false);
  const [z, setZ] = useState(1);
  const [viewport, setViewport] = useState({ width: innerWidth, height: innerHeight });
  const gesture = useRef<{ x: number; y: number; rect: Rect; resize?: boolean } | null>(null);
  const title = spec.kind === "ball-camera" ? "YOLO" : spec.kind === "chat" ? "humancli" : spec.kind === "teleop" ? "Teleop" : spec.title || spec.id;
  const forget = useRef(workspace?.forget); forget.current = workspace?.forget;
  const restore = () => { setHidden(false); setZ(++top); };
  useEffect(() => {
    const reset = () => { setRect(null); setMax(false); setHidden(false); forget.current?.(spec.id); };
    const escape = (e: KeyboardEvent) => { if (e.key === "Escape") setMax(false); };
    const resize = () => { setViewport({ width: innerWidth, height: innerHeight }); setRect(old => old ? { ...old, x: Math.max(0, Math.min(old.x, innerWidth - Math.min(old.width, innerWidth))), y: Math.max(0, Math.min(old.y, innerHeight - 40)), width: Math.min(old.width, innerWidth), height: Math.min(old.height, innerHeight) } : null); };
    globalThis.addEventListener("microduck-arrange", reset);
    globalThis.addEventListener("resize", resize);
    document.addEventListener("keydown", escape);
    return () => { globalThis.removeEventListener("microduck-arrange", reset); globalThis.removeEventListener("resize", resize); document.removeEventListener("keydown", escape); forget.current?.(spec.id); };
  }, [spec.id]);
  useLayoutEffect(() => {
    if (rect || max || hidden || !frame.current) return;
    const measure = () => setDockHeight(frame.current!.getBoundingClientRect().height);
    measure();
    const observer = new ResizeObserver(measure);
    observer.observe(frame.current);
    return () => observer.disconnect();
  }, [rect, max, hidden]);
  const begin = (e: PointerEvent<HTMLDivElement>) => {
    if (max || e.button !== 0 || (e.target as HTMLElement).closest("button,select")) return;
    const r = frame.current!.getBoundingClientRect();
    gesture.current = { x: e.clientX, y: e.clientY, rect: { x: r.x, y: r.y, width: r.width, height: r.height } };
    e.currentTarget.setPointerCapture(e.pointerId);
  };
  const move = (e: PointerEvent<HTMLDivElement>) => {
    const g = gesture.current; if (!g) return;
    const dx = e.clientX - g.x, dy = e.clientY - g.y;
    if (Math.abs(dx) + Math.abs(dy) < 4 && !rect) return;
    if (g.resize) {
      setRect({ ...g.rect, width: Math.min(innerWidth - g.rect.x, Math.max(180, g.rect.width + dx)), height: Math.min(innerHeight - g.rect.y, Math.max(140, g.rect.height + dy)) });
      return;
    }
    setRect({ ...g.rect, x: Math.max(0, Math.min(innerWidth - g.rect.width, g.rect.x + dx)), y: Math.max(0, Math.min(innerHeight - 40, g.rect.y + dy)) });
  };
  const square = spec.kind === "ball-camera" || spec.params.view === "pov";
  const squareSize = Math.max(80, Math.min(viewport.width - 24, (viewport.height - 120) * 16 / 9));
  const style: CSSProperties = { zIndex: z, ...(max ? square ? { position: "fixed", top: 12, left: Math.max(12, (viewport.width - squareSize) / 2), width: squareSize, height: "auto" } : { position: "fixed", inset: 12, width: "auto", height: "auto" } : rect ? { position: "fixed", left: rect.x, top: rect.y, width: rect.width, height: rect.height } : {}), ...(hidden ? { display: "none" } : {}) };
  return <div className="mw-frame-slot" style={(rect || max || hidden) && dockHeight !== undefined ? { height: dockHeight, minHeight: dockHeight } : undefined}>
    <section ref={frame} className="mw-frame" style={style} data-testid={`panel-${spec.id}`} data-panel-kind={spec.kind} data-maximized={max || undefined} data-floating={!!rect || undefined} onPointerDown={() => setZ(++top)}>
      <div className="mw-frame-head" tabIndex={0} aria-label={`Move ${title}`} onPointerDown={begin} onPointerMove={move} onPointerUp={() => { gesture.current = null; }} onPointerCancel={() => { gesture.current = null; }}
        onKeyDown={e => { if (!e.altKey || !e.key.startsWith("Arrow")) return; e.preventDefault(); const r = frame.current!.getBoundingClientRect(); setRect({ x: Math.max(0,Math.min(innerWidth-r.width,r.x+(e.key==="ArrowRight"?16:e.key==="ArrowLeft"?-16:0))), y: Math.max(0,Math.min(innerHeight-40,r.y+(e.key==="ArrowDown"?16:e.key==="ArrowUp"?-16:0))), width:r.width,height:r.height }); }}>
        <span className="mw-frame-title">{title}</span>
        <span className="mw-frame-actions">{badge}
          {workspace && <button aria-label={`Minimize ${title}`} onClick={() => {
            if (frame.current?.contains(document.activeElement)) (document.activeElement as HTMLElement).blur();
            setHidden(true); workspace.hide({ id: spec.id, title, restore });
          }}>−</button>}
          <button aria-label={max ? "restore" : "maximize"} data-testid={`panel-${spec.id}-max`} onClick={() => { setMax(v => !v); setZ(++top); }}>⛶</button>
        </span>
      </div>
      <div className="mw-frame-body">{children}</div>
      {!max && <div className="mw-frame-resize" role="button" tabIndex={0} aria-label={`Resize ${title}`} data-panel-action
        onPointerDown={e => {
          if (e.button !== 0) return;
          e.stopPropagation(); e.preventDefault(); setZ(++top);
          const r = frame.current!.getBoundingClientRect();
          gesture.current = { x: e.clientX, y: e.clientY, rect: { x: r.x, y: r.y, width: r.width, height: r.height }, resize: true };
          e.currentTarget.setPointerCapture(e.pointerId);
        }} onPointerMove={move} onPointerUp={() => { gesture.current = null; }} onPointerCancel={() => { gesture.current = null; }}
        onClick={e => e.stopPropagation()}
        onKeyDown={e => {
          if (!e.key.startsWith("Arrow")) return;
          e.preventDefault(); e.stopPropagation(); const r = frame.current!.getBoundingClientRect();
          setRect({ x:r.x, y:r.y, width:Math.min(innerWidth-r.x,Math.max(180,r.width+(e.key==="ArrowRight"?16:e.key==="ArrowLeft"?-16:0))), height:Math.min(innerHeight-r.y,Math.max(140,r.height+(e.key==="ArrowDown"?16:e.key==="ArrowUp"?-16:0))) });
        }} />}

    </section>
  </div>;
}
