import { useState } from "react";
import type { ChannelSpec, PanelSpec } from "@dimos/shared";
import type { PanelProps } from "./panels/registry.tsx";
import { VideoPanel } from "./panels/VideoPanel.tsx";
import { MapPanel } from "./panels/MapPanel.tsx";
import { TeleopPanel } from "./panels/TeleopPanel.tsx";
import styles from "./OperatorView.module.css";
import { Go2Controls } from "./Go2Controls.tsx";
import { ChannelList } from "./ui/ChannelList.tsx";

export function OperatorView({ panels, channels = [], store, teleop, session }: {
  panels: PanelSpec[];
  channels?: ChannelSpec[];
  store: PanelProps["store"];
  teleop: PanelProps["teleop"];
  session?: PanelProps["session"];
}) {
  const [main, setMain] = useState("camera");
  const video = panels.find((panel) => panel.kind === "video");
  const map = panels.find((panel) => panel.kind === "map2d");
  const drive = panels.find((panel) => panel.kind === "teleop");
  return (
    <div className={styles.operator}>
      <header className={styles.header}>
        <div>
          <strong>DIMENSIONAL</strong>
          <span>LOCAL OPERATOR</span>
        </div>
        <span title="Focus the drive pad to request manual control">MANUAL</span>
        <a href="?">Blueprint layout ↗</a>
      </header>
      <div className={styles.workspace}>
        <section className={styles.stageSection} aria-label="Robot observation">
          <nav className={styles.toolbar} aria-label="Main view">
            {(["camera", "map"] as const).map((view) => (
              <button
                key={view}
                aria-pressed={main === view}
                onClick={() => setMain(view)}
              >
                {view === "camera" ? "Camera view" : "Map view"}
              </button>
            ))}
            <span>Live streams · local relay</span>
          </nav>
          <div className={styles.stage}>
            <div
              className={main === "camera" ? styles.primary : styles.inset}
              data-testid="operator-camera"
            >
              {video
                ? <VideoPanel spec={video} store={store} />
                : <p>No camera panel advertised</p>}
            </div>
            <div
              className={main === "map" ? styles.primary : styles.inset}
              data-testid="operator-map"
            >
              {map ? <MapPanel spec={map} store={store} /> : <p>No map panel advertised</p>}
            </div>
          </div>
          <footer className={styles.footer}>
            <h2>Teleop instructions</h2>
            <dl className={styles.instructions}>
              <dt>W / S</dt>
              <dd>Forward / backward</dd>
              <dt>A / D</dt>
              <dd>Turn left / right</dd>
              <dt>Q / E</dt>
              <dd>Strafe left / right</dd>
              <dt>Shift</dt>
              <dd>Speed boost</dd>
              <dt>Space</dt>
              <dd>Stop</dd>
              <dt>Escape</dt>
              <dd>Release control</dd>
            </dl>
            <p>
              Focus the drive pad to request control. Leaving the pad or hiding this tab releases
              control.
            </p>
          </footer>
        </section>
        <aside className={styles.sidebar} aria-label="Robot controls">
          {session && <Go2Controls session={session} />}
          {drive
            ? <TeleopPanel spec={drive} store={store} teleop={teleop} />
            : (
              <div className={styles.card}>
                No teleop panel advertised. Start a cockpit blueprint.
              </div>
            )}
        </aside>
      </div>
      <section className={styles.statistics} aria-label="Channel statistics">
        <h2>Channel statistics</h2>
        <ChannelList channels={channels} panels={panels} store={store} />
      </section>
    </div>
  );
}
