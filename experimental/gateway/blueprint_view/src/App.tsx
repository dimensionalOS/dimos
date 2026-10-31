// The blueprint view (?name=, &run=, &view=logs, &hidden=1): the top bar runs it, the side lists modules and rates, the graph wires them.
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
  useConnectionState,
  useDesktopChrome,
  useDesktopVisible,
  useEscape,
  useZenoh,
} from "dimos-react";
import {
  blueprintCommentSource,
  BlueprintGraph,
  type BlueprintGraphHandle,
  type BlueprintSpot,
  putView,
  useNoPagePinch,
} from "dimos-react/graph";
import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { errorText, getJson, send } from "./api.ts";
import {
  BUSY_LAUNCH_POLL_MS,
  FONT_WAIT_MS,
  LAUNCH_POLL_WITHOUT_EVENTS_MS,
  RATES_POLL_MS,
  STATS_POLL_MS,
} from "./constants.ts";
import { CodePane, type Where } from "./code.tsx";
import { ConfigModal, type Saved } from "./config.tsx";
import { LaunchOutputModal, LogsModal } from "./logs.tsx";
import { type Blueprint, byRarity, type Module, moduleUsage, topicOf } from "./model.ts";
import {
  type Active,
  activeRuns,
  failedWhy,
  type Launch,
  launchedWithOther,
  liveRunOf,
  type RegistryRun,
  type Relaunching,
  relaunchLabel,
} from "./run.ts";
import type { StatsAnswer } from "./process_rows.ts";
import { CodeButton, ModuleList, ModuleView } from "./side.tsx";

const FRAMED = framedByDesktop();
const PARAMS = new URLSearchParams(location.search);
const NAME = PARAMS.get("name") ?? "";
const RUN = PARAMS.get("run") || null;
const VIEW = PARAMS.get("view");
/** This page's name on Desktop's comments, view reports and highlights (the dimos gateway serves it). */
export const APP = "dimos-gateway:blueprint_view";

interface RatesAnswer {
  up?: boolean;
  error?: string;
  topics?: TopicRate[];
}

async function loadSaved(): Promise<Saved | null> {
  try {
    const [global, mine] = await Promise.all([
      getJson<{ overrides?: Saved["global"]; defaults?: Saved["defaults"] }>(
        "../../apps/@launcher/config/global",
      ),
      getJson<{ global?: Saved["global"]; modules?: Saved["modules"] }>(
        `../../apps/@launcher/config/blueprints/${encodeURIComponent(NAME)}`,
      ),
    ]);
    return {
      global: { ...global.overrides, ...mine.global },
      modules: mine.modules ?? {},
      defaults: global.defaults ?? {},
    };
  } catch {
    return null;
  }
}

export function App() {
  const [modules, setModules] = useState<Module[] | null>(null);
  const [source, setSource] = useState<Where | null>(null);
  const [checkout, setCheckout] = useState<{ path: string | null; error: string | null }>({
    path: null,
    error: null,
  });
  const [error, setError] = useState<string | null>(null);
  const [selected, setSelected] = useState<string | null>(null);
  const [hovered, setHovered] = useState<string | null>(null);
  const [graphHover, setGraphHover] = useState<BlueprintSpot>(null);
  const [pinnedTopic, setPinnedTopic] = useState<string | null>(null);
  const [streamSpot, setStreamSpot] = useState<BlueprintSpot>(null);
  const [code, setCode] = useState<Where | null>(null);
  const [launch, setLaunch] = useState<Launch | null>(null);
  // undefined until the first poll; a run started outside Desktop (a terminal, tmux) is live too
  const [liveRun, setLiveRun] = useState<RegistryRun | null | undefined>(undefined);
  const [saved, setSaved] = useState<Saved | null>(null);
  const [relaunching, setRelaunching] = useState<Relaunching>(false);
  const [relaunchedPid, setRelaunchedPid] = useState<number | null>(null);
  const [actionError, setActionError] = useState<string | null>(null);
  const [rateRowsHeard, setRateRowsHeard] = useState<TopicRate[]>([]);
  const [ratesError, setRatesError] = useState<string | null>(null);
  const [stats, setStats] = useState<StatsAnswer | null>(null);
  const [statsError, setStatsError] = useState<string | null>(null);
  const [showExtras, setShowExtras] = useState(false);
  const [dialog, setDialog] = useState<"config" | "logs" | null>(null);
  const chrome = useDesktopChrome({ pausable: true });
  const visible = useDesktopVisible();
  const zenoh = useZenoh();
  const eventsArrive = useConnectionState() === "connected";
  const graph = useRef<BlueprintGraphHandle>(null);
  const [graphArea, setGraphArea] = useState<HTMLElement | null>(null);
  useNoPagePinch(graphArea);

  const running = !!launch && (launch.phase === "running" || launch.phase === "starting");
  const live = running || !!liveRun;
  const logsRun = RUN ?? launch?.runId ?? null;
  const busy = !!relaunching || launch?.phase === "starting" || launch?.phase === "stopping";
  // a launch that never became a run (a bad config, a build that failed) has no run log: what it printed stands in
  const launchOutput = !logsRun && launch?.output ? launch.output : null;
  const failed = !!launch && launch.phase === "failed" && !live && !busy;

  // the gateway reads the blueprint from the checkout its live run came from, so a new run can change it
  const loadBlueprint = useCallback(async () => {
    const usage = getJson<{ blueprints?: { modules?: string[] }[] }>(
      "catalog/blueprints?fields=blueprints.modules",
    ).then(
      moduleUsage,
      () => new Map(),
    );
    try {
      const detail = await getJson<Blueprint>(`catalog/blueprints/${encodeURIComponent(NAME)}`);
      setSource(detail.file ? { file: detail.file, line: detail.line ?? 1 } : null);
      setCheckout({ path: detail.checkout ?? null, error: detail.checkoutError ?? null });
      await Promise.race([
        document.fonts.ready,
        new Promise((resolve) => setTimeout(resolve, FONT_WAIT_MS)),
      ]);
      setModules(byRarity(detail.modules, await usage));
      setError(null);
    } catch (caught) {
      setError(errorText(caught));
    }
  }, []);

  useEffect(() => {
    document.title = `${NAME} · blueprint`;
    loadBlueprint();
    loadSaved().then(setSaved);
  }, [loadBlueprint]);

  const liveRunId = liveRun === undefined ? undefined : liveRun?.run_id ?? null;
  const shownRunId = useRef<string | null | undefined>(undefined);
  useEffect(() => {
    if (liveRunId === undefined) {
      return;
    }
    if (shownRunId.current !== undefined && shownRunId.current !== liveRunId) {
      loadBlueprint();
    }
    shownRunId.current = liveRunId;
  }, [liveRunId, loadBlueprint]);

  const pollRuns = useCallback(async () => {
    try {
      const answer = await getJson<Active>("active");
      const mine = answer.launch?.blueprint === NAME ? answer.launch : null;
      setLaunch(mine);
      setLiveRun(liveRunOf(activeRuns(answer), NAME, RUN));
      setRelaunching((was) =>
        was === "starting" && mine?.pid !== relaunchedPidRef.current && mine?.phase !== "starting"
          ? false
          : was
      );
    } catch {
      setLaunch(null);
      setLiveRun(null);
    }
  }, []);
  const relaunchedPidRef = useRef<number | null>(null);
  relaunchedPidRef.current = relaunchedPid;

  useEffect(() => {
    if (VIEW === "logs") {
      pollRuns().then(() => setDialog("logs"));
    }
  }, [pollRuns]);

  // GET active once on showing, then on each launch event and after a reconnect (events sent while down are gone)
  useEffect(() => {
    if (!visible) {
      return;
    }
    pollRuns();
    if (!zenoh) {
      return;
    }
    const offLaunch = zenoh.subscribeDimos("launch", () => pollRuns());
    const offReconnect = zenoh.onReconnect(() => pollRuns());
    return () => {
      offLaunch();
      offReconnect();
    };
  }, [pollRuns, visible, zenoh]);

  // polls only for what no event tells: a busy launch's steps, or anything while zenoh is down
  const launchPollMs = busy
    ? BUSY_LAUNCH_POLL_MS
    : eventsArrive
    ? null
    : LAUNCH_POLL_WITHOUT_EVENTS_MS;
  useEffect(() => {
    if (!visible || launchPollMs === null) {
      return;
    }
    const timer = setInterval(pollRuns, launchPollMs);
    return () => clearInterval(timer);
  }, [pollRuns, visible, launchPollMs]);

  useEffect(() => {
    if (!visible) {
      return;
    }
    const poll = async () => {
      try {
        const answer = await getJson<RatesAnswer>("../desktop/topics/rates");
        setRatesError(answer.up === false ? answer.error ?? null : null);
        setRateRowsHeard(
          (answer.topics ?? []).filter((row) => !String(row.topic).startsWith("/rpc/")),
        );
      } catch (caught) {
        setRatesError(errorText(caught));
      }
    };
    poll();
    const timer = setInterval(poll, RATES_POLL_MS);
    return () => clearInterval(timer);
  }, [visible]);

  // dtop's numbers for the processes table, while something runs (the gateway hears them only while asked)
  useEffect(() => {
    if (!visible || !running) {
      setStats(null);
      return;
    }
    const poll = async () => {
      try {
        const answer = await getJson<StatsAnswer>("active/stats");
        setStats(answer);
        setStatsError(
          answer.up ? null : "no stats yet: a run started outside Desktop needs --dtop",
        );
      } catch (caught) {
        setStatsError(errorText(caught));
      }
    };
    poll();
    const timer = setInterval(poll, STATS_POLL_MS);
    return () => clearInterval(timer);
  }, [visible, running]);

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
    if (!live || !modules) {
      return [];
    }
    const wired = new Set(modules.flatMap((module) => module.streams.map(topicOf)));
    return [...rates.keys()].filter((topic) => !wired.has(topic)).sort().map((topic) => ({
      topic,
      type: String(
        rateRowsHeard.find((row) => String(row.topic).replace(/^\/+/, "") === topic)?.type ?? "",
      ).replace(/\//g, "."),
    }));
  }, [live, modules, rates, rateRowsHeard]);
  // a comment on a module, topic or edge goes to the agent with the module's class
  const commentSource = useMemo(
    () =>
      blueprintCommentSource(
        modules ?? [],
        (file) =>
          getJson<{ text: string }>(`source?file=${encodeURIComponent(file)}`).then((answer) =>
            answer.text
          ),
      ),
    [modules],
  );

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

  const spot: BlueprintSpot = graphHover ?? (hovered ? { module: hovered } : null) ?? streamSpot ??
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
      // through Desktop, which keeps the saved config: it merges it in, so edits take effect
      (running
        ? send("POST", "../../apps/@launcher/relaunch")
        : send("POST", "../../apps/@launcher/launch", { blueprint: NAME })).then(
          () => setRelaunching("starting"),
          (caught) => {
            setRelaunching(false);
            throw caught;
          },
        )
    );
  };

  const phase = running
    ? launch.phase
    : liveRun
    ? "running"
    : !launch
    ? "not running"
    : launch.phase === "failed"
    ? "failed"
    : "last run";
  const stale = running && launchedWithOther(launch, saved);
  const selectedModule = modules?.find((module) => module.name === selected);

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
              title={!running && liveRun
                ? `started outside Desktop (pid ${liveRun.pid}, ${liveRun.run_id})`
                : undefined}
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
                <span className="label">
                  {relaunchLabel(launch, relaunching, relaunchedPid, stale)}
                </span>
              </Button>
            )}
            {running && (
              <Button
                size="sm"
                data-bp-stop
                onClick={() => act(() => send("POST", "active/stop"))}
              >
                <span className="label">Stop</span>
              </Button>
            )}
            <Button size="sm" data-bp-configure onClick={() => setDialog("config")}>
              <span className="label">Configure</span>
            </Button>
            {source && (
              <CodeButton where={source} shown={code} onToggle={setCode} data-bp-blueprint-code />
            )}
            <Button
              size="sm"
              disabled={!logsRun && !launchOutput}
              title={logsRun
                ? `${NAME}'s log`
                : launchOutput
                ? "no run started: what the launch printed"
                : "no log yet"}
              data-bp-logs
              onClick={() => setDialog("logs")}
            >
              <span className="label">{logsRun ? "Logs" : "Output"}</span>
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
      {failed && (
        <div className="bv-failed-bar" role="alert" data-bp-failed>
          <Text small tone="warn" className="bv-failed-head">Launch failed</Text>
          <Text small className="bv-failed-why" title={failedWhy(launch)}>{failedWhy(launch)}</Text>
          {(logsRun || launchOutput) && (
            <Button
              size="sm"
              data-bp-failed-logs
              onClick={() => setDialog("logs")}
            >
              <span className="label">{logsRun ? "Logs" : "Show output"}</span>
            </Button>
          )}
        </div>
      )}
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
                stats={running ? stats : null}
                statsError={running ? statsError : null}
                onOpen={open}
                onHover={setHovered}
              />
            )}
        </aside>
        <main id="graph" ref={setGraphArea}>
          <BlueprintGraph
            ref={graph}
            className="bv-graph"
            modules={modules ?? []}
            extras={showExtras ? extras : []}
            spot={spot}
            rates={live ? rates : null}
            onModule={open}
            onTopic={(topic) => setPinnedTopic((was) => was === topic ? null : topic)}
            onHover={setGraphHover}
            app={APP}
            blueprint={NAME}
            commentSource={commentSource}
            onViewChange={(view) => putView(APP, { ...view, blueprint: NAME })}
          >
            <div className="graph-head">
              <Label>Module graph</Label>
              {live && <Badge tone="ok">live</Badge>}
              {checkout.path && (
                <Text small muted title="the live run's checkout, which the graph is read from">
                  from {checkout.path}
                </Text>
              )}
              {checkout.error && (
                <Text small tone="warn" title={checkout.error}>
                  not the live run's checkout
                </Text>
              )}
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
      {dialog === "logs" && (logsRun
        ? (
          <LogsModal
            runId={logsRun}
            title={NAME}
            paused={!visible}
            onClose={() => setDialog(null)}
          />
        )
        : launchOutput
        ? (
          <LaunchOutputModal
            output={launchOutput}
            why={failedWhy(launch)}
            title={NAME}
            onClose={() => setDialog(null)}
          />
        )
        : null)}
    </>
  );
}
