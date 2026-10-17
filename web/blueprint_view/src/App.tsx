// The blueprint view: ?name=<blueprint> (&run=<run id>, &view=logs, &hidden=1). The top bar runs it, the side lists its
// modules and topic rates, the graph shows how they connect. Hidden (Desktop preloading it), it loads once and polls nothing.
import {
  Badge,
  Button,
  Checkbox,
  framedByDesktop,
  IconButton,
  Label,
  Spinner,
  Text,
  toDesktop,
  TopBar,
  type TopicRate,
  useDesktopChrome,
  useDesktopVisible,
  useEscape,
} from "dimos-react";
import { GraphControls, type GraphViewHandle, type LayoutId } from "dimos-react/graph";
import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { errorText, getJson, send } from "./api.ts";
import { CodePane, type Where } from "./code.tsx";
import { ConfigModal, type Saved } from "./config.tsx";
import { BlueprintGraph, type Spot, useNoPagePinch } from "./graph.tsx";
import { LogsModal } from "./logs.tsx";
import { type Blueprint, byRarity, type Module, moduleUsage, topicOf } from "./model.ts";
import { type Launch, launchedWithOther, type Relaunching, relaunchLabel } from "./run.ts";
import { CodeButton, ModuleList, ModuleView } from "./side.tsx";

const FRAMED = framedByDesktop();
const PARAMS = new URLSearchParams(location.search);
const NAME = PARAMS.get("name") ?? "";
const RUN = PARAMS.get("run") || null;
const VIEW = PARAMS.get("view");

interface RatesAnswer {
  up?: boolean;
  error?: string;
  topics?: TopicRate[];
}

async function loadSaved(): Promise<Saved | null> {
  try {
    const [global, modules] = await Promise.all([
      getJson<{ overrides?: Saved["global"]; defaults?: Saved["defaults"] }>("global-config"),
      getJson<{ config?: { overrides?: Saved["modules"] } }>(
        `blueprints/${encodeURIComponent(NAME)}`,
      ),
    ]);
    return {
      global: global.overrides ?? {},
      modules: modules.config?.overrides ?? {},
      defaults: global.defaults ?? {},
    };
  } catch {
    return null;
  }
}

export function App() {
  const [modules, setModules] = useState<Module[] | null>(null);
  const [source, setSource] = useState<Where | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [selected, setSelected] = useState<string | null>(null);
  const [hovered, setHovered] = useState<string | null>(null);
  const [graphHover, setGraphHover] = useState<Spot>(null);
  const [pinnedTopic, setPinnedTopic] = useState<string | null>(null);
  const [streamSpot, setStreamSpot] = useState<Spot>(null);
  const [code, setCode] = useState<Where | null>(null);
  const [layout, setLayout] = useState<LayoutId>("hierarchy");
  const [launch, setLaunch] = useState<Launch | null>(null);
  const [saved, setSaved] = useState<Saved | null>(null);
  const [relaunching, setRelaunching] = useState<Relaunching>(false);
  const [relaunchedPid, setRelaunchedPid] = useState<number | null>(null);
  const [actionError, setActionError] = useState<string | null>(null);
  const [rateRowsHeard, setRateRowsHeard] = useState<TopicRate[]>([]);
  const [ratesError, setRatesError] = useState<string | null>(null);
  const [showExtras, setShowExtras] = useState(false);
  const [dialog, setDialog] = useState<"config" | "logs" | null>(null);
  const chrome = useDesktopChrome({ pausable: true });
  const visible = useDesktopVisible();
  const graph = useRef<GraphViewHandle>(null);
  const [graphArea, setGraphArea] = useState<HTMLElement | null>(null);
  useNoPagePinch(graphArea);

  const running = !!launch && (launch.phase === "running" || launch.phase === "starting");
  const logsRun = RUN ?? launch?.runId ?? null;

  useEffect(() => {
    document.title = `${NAME} · blueprint`;
    const usage = getJson<{ blueprints?: { modules?: string[] }[] }>("catalog").then(
      moduleUsage,
      () => new Map(),
    );
    (async () => {
      try {
        const detail = await getJson<Blueprint>(`blueprints/${encodeURIComponent(NAME)}`);
        setSource(detail.file ? { file: detail.file, line: detail.line ?? 1 } : null);
        await Promise.race([
          document.fonts.ready,
          new Promise((resolve) => setTimeout(resolve, 1500)),
        ]);
        setModules(detail.modules);
        setModules(byRarity(detail.modules, await usage));
      } catch (caught) {
        setError(errorText(caught));
      }
    })();
    loadSaved().then(setSaved);
  }, []);

  const pollRuns = useCallback(async () => {
    try {
      const answer = await getJson<{ launch: Launch | null }>("runs");
      const mine = answer.launch?.blueprint === NAME ? answer.launch : null;
      setLaunch(mine);
      setRelaunching((was) =>
        was === "starting" && mine?.pid !== relaunchedPidRef.current && mine?.phase !== "starting"
          ? false
          : was
      );
    } catch {
      setLaunch(null);
    }
  }, []);
  const relaunchedPidRef = useRef<number | null>(null);
  relaunchedPidRef.current = relaunchedPid;
  const busyRef = useRef(false);
  busyRef.current = !!relaunching || launch?.phase === "starting" || launch?.phase === "stopping";

  useEffect(() => {
    if (VIEW === "logs") {
      pollRuns().then(() => setDialog("logs"));
    }
  }, [pollRuns]);

  useEffect(() => {
    if (!visible) {
      return;
    }
    pollRuns();
    let tick = 0;
    const timer = setInterval(() => {
      tick += 1;
      if (busyRef.current || tick % 3 === 0) {
        pollRuns();
      }
    }, 1000);
    return () => clearInterval(timer);
  }, [pollRuns, visible]);

  useEffect(() => {
    if (!visible) {
      return;
    }
    const poll = async () => {
      try {
        const answer = await getJson<RatesAnswer>("../api/topics/rates");
        setRatesError(answer.up === false ? answer.error ?? null : null);
        setRateRowsHeard(
          (answer.topics ?? []).filter((row) => !String(row.topic).startsWith("/rpc/")),
        );
      } catch (caught) {
        setRatesError(errorText(caught));
      }
    };
    poll();
    const timer = setInterval(poll, 2000);
    return () => clearInterval(timer);
  }, [visible]);

  // Hz per topic (without its leading slash), and the live topics no module here is wired to
  const rates = useMemo(() => {
    const byTopic = new Map<string, number>();
    for (const row of rateRowsHeard) {
      const topic = String(row.topic).replace(/^\/+/, "");
      byTopic.set(topic, (byTopic.get(topic) ?? 0) + (row.hz ?? 0));
    }
    return byTopic;
  }, [rateRowsHeard]);
  const extras = useMemo(() => {
    if (!running || !modules) {
      return [];
    }
    const wired = new Set(modules.flatMap((m) => m.streams.map(topicOf)));
    return [...rates.keys()].filter((topic) => !wired.has(topic)).sort().map((topic) => ({
      topic,
      type: String(
        rateRowsHeard.find((row) => String(row.topic).replace(/^\/+/, "") === topic)?.type ?? "",
      ).replace(/\//g, "."),
    }));
  }, [running, modules, rates, rateRowsHeard]);

  const open = (name: string | null) => {
    setSelected(name);
    setCode(null);
    setHovered(null);
    setStreamSpot(null);
    setPinnedTopic(null);
    document.getElementById("side")?.scrollTo(0, 0);
    if (name) {
      graph.current?.reveal(`m:${name}`);
    }
  };

  useEscape(() => {
    if (dialog) {
      return;
    }
    if (code) {
      setCode(null);
    } else if (selected) {
      open(null);
    } else if (FRAMED) {
      toDesktop({ type: "dimos:close" });
    }
  });

  const spot: Spot = graphHover ?? (hovered ? { module: hovered } : null) ?? streamSpot ??
    (pinnedTopic ? { topic: pinnedTopic } : null) ?? (selected ? { module: selected } : null);

  const act = async (action: () => Promise<unknown>) => {
    setActionError(null);
    try {
      await action();
    } catch (caught) {
      setActionError(errorText(caught));
    }
    await pollRuns();
  };
  const relaunch = () => {
    setRelaunching(running ? "stopping" : "starting");
    setRelaunchedPid(launch?.pid ?? null);
    act(() =>
      send("POST", "runs/restart", { runId: launch?.runId }).then(
        () => setRelaunching("starting"),
        (caught) => {
          setRelaunching(false);
          throw caught;
        },
      )
    );
  };

  const phase = !launch
    ? "not running"
    : running
    ? launch.phase
    : launch.phase === "failed"
    ? "failed"
    : "last run";
  const stale = running && launchedWithOther(launch, saved);
  const busy = !!relaunching || launch?.phase === "starting" || launch?.phase === "stopping";
  const selectedModule = modules?.find((m) => m.name === selected);

  return (
    <>
      <TopBar
        hidden={!chrome}
        title={NAME}
        left={
          <>
            <Text
              small
              muted
              tone={phase === "running" ? "ok" : phase === "failed" ? "warn" : undefined}
            >
              {phase}
            </Text>
            {actionError && (
              <Text small tone="warn" className="failed" title={actionError}>
                {actionError}
              </Text>
            )}
          </>
        }
        right={
          <>
            {launch && (
              <Button
                size="sm"
                variant={busy || launch.phase === "running" || running ? "default" : "primary"}
                className={`relaunch${launch.phase === "running" && !busy ? " live" : ""}${
                  stale && !busy ? " stale" : ""
                }`}
                disabled={busy}
                data-bp-relaunch
                title={stale
                  ? "the saved config changed since this run started"
                  : running
                  ? "stops this live run, then starts it again"
                  : undefined}
                onClick={relaunch}
              >
                {busy ? <Spinner size={12} /> : <span aria-hidden="true">↻</span>}
                {relaunchLabel(launch, relaunching, relaunchedPid, stale)}
              </Button>
            )}
            {running && (
              <Button
                size="sm"
                data-bp-stop
                onClick={() => act(() => send("POST", "runs/stop", { runId: launch?.runId }))}
              >
                Stop
              </Button>
            )}
            <Button size="sm" data-bp-configure onClick={() => setDialog("config")}>
              Configure
            </Button>
            {source && (
              <CodeButton where={source} shown={code} onToggle={setCode} data-bp-blueprint-code />
            )}
            <Button
              size="sm"
              disabled={!logsRun}
              title={logsRun ? `${NAME}'s log` : "no log yet"}
              data-bp-logs
              onClick={() => setDialog("logs")}
            >
              Logs
            </Button>
            {FRAMED && (
              <IconButton
                icon="close"
                label="Close"
                size="sm"
                onClick={() => toDesktop({ type: "dimos:close" })}
              />
            )}
          </>
        }
      />
      <div id="app">
        <aside id="side" aria-label="Modules">
          {selectedModule
            ? (
              <ModuleView
                module={selectedModule}
                modules={modules ?? []}
                code={code}
                onCode={setCode}
                onOpen={open}
                onSpot={setStreamSpot}
              />
            )
            : (
              <ModuleList
                modules={modules}
                error={error}
                rates={rateRowsHeard}
                ratesError={ratesError}
                onOpen={open}
                onHover={setHovered}
              />
            )}
        </aside>
        <main id="graph" ref={setGraphArea}>
          <BlueprintGraph
            ref={graph}
            modules={modules ?? []}
            extras={showExtras ? extras : []}
            layout={layout}
            spot={spot}
            rates={running ? rates : null}
            onModule={open}
            onTopic={(topic) => setPinnedTopic((was) => was === topic ? null : topic)}
            onHover={setGraphHover}
          >
            <div className="graph-head">
              <Label>Module graph</Label>
              {running && <Badge tone="ok">live</Badge>}
            </div>
            {extras.length > 0 && (
              <details id="extras">
                <summary>Other live topics · {extras.length}</summary>
                <Checkbox label="draw them" value={showExtras} onChange={setShowExtras} />
                <ul>
                  {extras.map(({ topic }) => (
                    <li key={topic}>
                      /{topic}
                      <span className="hz">{(rates.get(topic) ?? 0).toFixed(1)} Hz</span>
                    </li>
                  ))}
                </ul>
              </details>
            )}
            <GraphControls graph={graph} layout={layout} onLayoutChange={setLayout} />
            {code && <CodePane where={code} onClose={() => setCode(null)} />}
          </BlueprintGraph>
        </main>
      </div>
      {dialog === "config" && (
        <ConfigModal
          name={NAME}
          onSaved={setSaved}
          onClose={() => setDialog(null)}
        />
      )}
      {dialog === "logs" && logsRun && (
        <LogsModal
          runId={logsRun}
          title={NAME}
          paused={!visible}
          onClose={() => setDialog(null)}
        />
      )}
    </>
  );
}
