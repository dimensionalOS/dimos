import { useContext, useEffect, useRef, useState } from "react";
import { useStoreChannel } from "@dimos/sdk/react";
import { Badge, type DrawHealth, PanelFrame } from "./cockpit/PanelFrame.tsx";
import type { PanelProps } from "@dimos/cockpit/panels/registry.tsx";
import { startVideoSink, VIDEO_STALE_MS } from "@dimos/cockpit/panels/VideoPanel.tsx";
import { type CameraMode, WorldRenderer } from "./worldRenderer.ts";
import { fetchDefinition, readSnapshot, type WorldSnapshot } from "./worldModel.ts";
import { ViewerSession } from "./viewerSession.ts";
import styles from "./world.module.css";
import { PlayerDirectory } from "./players.ts";
import { playerName } from "./roster.ts";

export function WorldPanel({ spec, store }: PanelProps) {
  const session = useContext(ViewerSession);
  const players = useContext(PlayerDirectory);
  const [showNames, setShowNames] = useState(
    localStorage.getItem("world-show-names") !== "false",
  );
  const [followed, setFollowed] = useState("");
  const host = useRef<HTMLDivElement>(null);
  const jpegCanvas = useRef<HTMLCanvasElement>(null);
  const viewer = useRef<WorldRenderer | null>(null);
  const health = useRef<DrawHealth>({ lastDrawOkAtMs: Date.now(), failures: 0 }).current;
  const [mode, setMode] = useState<CameraMode>("explore");
  const [backend, setBackend] = useState<"three" | "jpeg">("three");
  const [fps, setFps] = useState(0);
  const [score, setScore] = useState<[number, number] | null>(null);
  const [status, setStatus] = useState("Waiting for the world…");
  const [ready, setReady] = useState(false);
  const view = spec.params?.view === "pov" ? "pov" : "world";
  const prefix = view === "pov" ? "duck3d" : "world3d";
  const ch = spec.channels[0];
  const jpegCh = String(spec.params?.jpeg ?? "");
  const { stats } = useStoreChannel(store, ch);
  const { slot: jpegSlot } = useStoreChannel(store, jpegCh);

  useEffect(() => {
    const element = host.current;
    if (!element || !ch) return;
    let renderer: WorldRenderer;
    try {
      renderer = new WorldRenderer(
        element,
        setMode,
        view,
        setFps,
        String(spec.params?.robot ?? "duck1"),
      );
    } catch {
      setStatus(
        "3D graphics are unavailable. Enable hardware acceleration or select MuJoCo JPEG.",
      );
      return;
    }
    viewer.current = renderer;
    let modelUrl = "";
    let loadedUrl = "";
    let latest: WorldSnapshot | null = null;
    let disposed = false;
    const contextLost = (event: Event) => {
      event.preventDefault();
      renderer.setActive(false);
      setReady(false);
      setStatus(
        "The graphics connection was lost. Reload or select MuJoCo JPEG.",
      );
    };
    renderer.renderer.domElement.addEventListener(
      "webglcontextlost",
      contextLost,
    );
    const ingest = () => {
      const snapshot = readSnapshot(store.get(ch)?.value);
      if (!snapshot) return;
      latest = snapshot;
      if (snapshot.football) {
        const next = snapshot.football.scores;
        setScore((old) => old?.[0] === next[0] && old?.[1] === next[1] ? old : next);
      }
      if (snapshot.model !== modelUrl) {
        modelUrl = snapshot.model;
        loadedUrl = "";
        setReady(false);
        setStatus("Loading the world…");
        const requestedUrl = modelUrl;
        fetchDefinition(requestedUrl).then((model) => {
          if (disposed || requestedUrl !== modelUrl) return;
          renderer.load(model);
          loadedUrl = requestedUrl;
          if (latest?.model === requestedUrl) renderer.push(latest);
          setReady(true);
        }).catch((error: unknown) => {
          if (disposed || requestedUrl !== modelUrl) return;
          setReady(false);
          setStatus(
            error instanceof Error ? error.message : "The world could not load. Reload to retry.",
          );
        });
      } else if (loadedUrl === modelUrl) {
        try {
          renderer.push(snapshot);
        } catch (error) {
          setReady(false);
          setStatus(
            error instanceof Error ? error.message : "Invalid world update.",
          );
        }
      }
    };
    const unsubscribe = store.subscribe(ch, ingest);
    ingest();
    return () => {
      disposed = true;
      unsubscribe();
      renderer.renderer.domElement.removeEventListener(
        "webglcontextlost",
        contextLost,
      );
      renderer.dispose();
      viewer.current = null;
    };
  }, [ch, store, view]);

  useEffect(() => {
    viewer.current?.setActive(backend === "three");
    if (backend !== "jpeg" || !session || !jpegCanvas.current || !jpegCh) {
      return;
    }
    let release: (() => void) | null = null;
    const visibility = () => {
      if (document.hidden) {
        release?.();
        release = null;
      } else if (!release) release = session.subscribe(jpegCh, () => {});
    };
    visibility();
    document.addEventListener("visibilitychange", visibility);
    const stopSink = startVideoSink(store, jpegCh, jpegCanvas.current, health);
    return () => {
      document.removeEventListener("visibilitychange", visibility);
      release?.();
      stopSink();
    };
  }, [backend, session, jpegCh, store, health]);

  useEffect(() => {
    viewer.current?.setPlayers(players, showNames);
  }, [players, showNames, ready]);

  const switchBackend = (value: "three" | "jpeg") => {
    if (value === "jpeg" && view === "world") viewer.current?.follow();
    setBackend(value);
  };
  const fpsBadge = (backend === "three"
            ? (
              <span
                className={styles.fps}
                data-testid={`${prefix}-fps`}
                title={`Completed client draws per second; simulation updates: ${
                  stats.hz.toFixed(1)
                }/s`}
              >
                {fps.toFixed(0)} FPS
              </span>
            )
            : (
              <Badge
                store={store}
                ch={jpegCh}
                health={health}
                staleMs={VIDEO_STALE_MS}
                unit="FPS"
                testId={`${prefix}-fps`}
              />
            ));
  return (
    <PanelFrame
      spec={spec}
      badge={
        <>
          {view !== "pov" && fpsBadge}
          <select
            className={styles.rendererSelect}
            aria-label={`${spec.title} renderer`}
            data-testid={`${prefix}-renderer`}
            value={backend}
            onChange={(event) => switchBackend(event.target.value as "three" | "jpeg")}
          >
            <option value="three">Three.js</option>
            {jpegCh && <option value="jpeg">MuJoCo JPEG</option>}
          </select>
        </>
      }
    >
      <div
        className={`${styles.world} ${view === "pov" ? styles.pov : ""}`}
        data-renderer={backend}
      >
        {view === "pov" && <div className="mw-feed-fps">{fpsBadge}</div>}
        <div
          ref={host}
          className={styles.viewport}
          data-testid={`${prefix}-viewport`}
          hidden={backend !== "three"}
        />
        <canvas
          ref={jpegCanvas}
          className={styles.jpeg}
          data-testid={`${prefix}-jpeg`}
          hidden={backend !== "jpeg"}
          role="img"
          aria-label={`${spec.title} rendered by MuJoCo`}
        />
        {backend === "three" && !ready && (
          <div className={styles.notice} role="status">{status}</div>
        )}
        {backend === "jpeg" && !jpegSlot && (
          <div className={styles.notice} role="status">Starting JPEG view…</div>
        )}
        {view === "world" && score && (
          <output
            className={styles.score}
            data-testid="football-score"
            aria-label={`Football score: Blue ${score[0]}, Red ${score[1]}`}
            title="Score against the opposite goal. Balls stay where physics takes them."
          >
            <span className={styles.blueTeam}>Blue</span>
            <b>{score[0]} : {score[1]}</b>
            <span className={styles.coralTeam}>Red</span>
          </output>
        )}
        {view === "world" && (
          <div className={styles.toolbar} aria-label="World camera">
            {backend === "three"
              ? (
                <>
                  <button
                    type="button"
                    onClick={() => viewer.current?.overview()}
                    disabled={!ready}
                    data-testid="world3d-overview"
                  >
                    Overview
                  </button>
                  <button
                    type="button"
                    onClick={() => viewer.current?.follow()}
                    disabled={!ready}
                    aria-pressed={mode === "follow"}
                    title="Lock a close camera behind the duck. Drag the view to unlock."
                    data-testid="world3d-follow"
                  >
                    Follow duck
                  </button>
                  <button
                    type="button"
                    onClick={() => viewer.current?.room("football")}
                    disabled={!ready}
                    data-testid="world3d-football"
                  >
                    Football field
                  </button>
                  <button
                    type="button"
                    onClick={() => viewer.current?.room("lockers")}
                    disabled={!ready}
                  >
                    Locker rooms
                  </button>
                  <button
                    type="button"
                    onClick={() => viewer.current?.room("benchmark")}
                    disabled={!ready}
                  >
                    Benchmark wing
                  </button>
                  <select
                    className={styles.playerSelect}
                    aria-label="Follow player"
                    data-testid="follow-player"
                    value={players.some((player) => player.id === followed && player.occupied)
                      ? followed
                      : ""}
                    onChange={(event) => {
                      setFollowed(event.target.value);
                      if (event.target.value) {
                        viewer.current?.follow(event.target.value);
                      }
                    }}
                  >
                    <option value="">Choose player…</option>
                    {players.filter((player) => player.occupied).map(
                      (player) => (
                        <option key={player.id} value={player.id}>
                          {player.displayName || playerName(player.id)} · {playerName(player.id)}
                        </option>
                      ),
                    )}
                  </select>
                  <label className={styles.namesToggle}>
                    <input
                      type="checkbox"
                      checked={showNames}
                      data-testid="show-player-names"
                      onChange={(event) => {
                        setShowNames(event.target.checked);
                        localStorage.setItem(
                          "world-show-names",
                          String(event.target.checked),
                        );
                      }}
                    />
                    Names
                  </label>
                  <span className={styles.help}>
                    {mode === "follow"
                      ? "Following duck · Drag to unlock camera"
                      : "Drag to orbit · Scroll to zoom · Right-drag to pan"}
                  </span>
                </>
              )
              : (
                <span className={styles.help}>
                  Shared follow camera · 640×360 · 12 FPS limit
                </span>
              )}
          </div>
        )}
      </div>
    </PanelFrame>
  );
}
