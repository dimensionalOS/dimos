// The module graph: modules and the topics they write and read, laid out by dimos-react/graph, lit around what's
// hovered or picked, and showing each topic's rate while the blueprint runs.
import { GraphView, type GraphViewHandle, type LayoutId, textWidth } from "dimos-react/graph";
import { formatRate, messageTypeColor, useTheme } from "dimos-react";
import { type CSSProperties, forwardRef, type ReactNode, useEffect, useMemo } from "react";
import { className, type Module, topicOf } from "./model.ts";

const MODULE_H = 46;
const MODULE_BAR = 4;
const MODULE_PAD_X = 16;
const TAG_GAP = 10;
const CHEV_W = 22;
const TOPIC_H = 30;
const PAD_X = 9;
const PORT = 7;
const RATE_RESERVE = "999.9 Hz";

interface ModuleNode {
  id: string;
  kind: "module";
  name: string;
  label: string;
  tagW: number;
  w: number;
  h: number;
}

interface TopicNode {
  id: string;
  kind: "topic";
  topic: string;
  type: string;
  extra: boolean;
  loose: boolean;
  w: number;
  h: number;
}

type Node = ModuleNode | TopicNode;
type Edge = { from: string; to: string; topic: string };

/** What's lit: a module, or a topic. */
export type Spot = { module: string } | { topic: string } | null;

export interface BlueprintGraphProps {
  modules: Module[];
  extras: { topic: string; type: string }[];
  layout: LayoutId;
  spot: Spot;
  /** Hz per topic while the blueprint runs, else null. */
  rates: Map<string, number> | null;
  onModule: (name: string) => void;
  onTopic: (topic: string | null) => void;
  onHover: (spot: Spot) => void;
  children?: ReactNode;
}

function fontOf(name: string) {
  return getComputedStyle(document.documentElement).getPropertyValue(name).trim() || "sans-serif";
}

function buildGraph(modules: Module[], extras: { topic: string; type: string }[]) {
  const sans = fontOf("--sans");
  const mono = fontOf("--mono");
  const nodes: Node[] = [];
  const edges: Edge[] = [];
  const topics = new Map<string, { type: string; extra: boolean }>();
  for (const module of modules) {
    const label = className(module);
    const tagW = Math.ceil(textWidth("MOD", `600 9.5px ${sans}`) * 1.25);
    const labelW = Math.ceil(textWidth(label, `600 15px ${sans}`));
    nodes.push({
      id: `m:${module.name}`,
      kind: "module",
      name: module.name,
      label,
      tagW,
      w: MODULE_BAR + MODULE_PAD_X + tagW + TAG_GAP + labelW + CHEV_W,
      h: MODULE_H,
    });
    for (const stream of module.streams) {
      const topic = topicOf(stream);
      if (!topics.has(topic)) {
        topics.set(topic, { type: stream.type, extra: false });
      }
      if (stream.direction !== "in") {
        edges.push({ from: `m:${module.name}`, to: `t:${topic}`, topic });
      }
      if (stream.direction !== "out") {
        edges.push({ from: `t:${topic}`, to: `m:${module.name}`, topic });
      }
    }
  }
  for (const extra of extras) {
    if (!topics.has(extra.topic)) {
      topics.set(extra.topic, { type: extra.type, extra: true });
    }
  }
  const written = new Set(edges.filter((e) => e.from.startsWith("m:")).map((e) => e.topic));
  const read = new Set(edges.filter((e) => e.to.startsWith("m:")).map((e) => e.topic));
  for (const [topic, { type, extra }] of topics) {
    const width = Math.max(
      textWidth(topic, `500 12px ${mono}`),
      textWidth(RATE_RESERVE, `10px ${mono}`),
    );
    nodes.push({
      id: `t:${topic}`,
      kind: "topic",
      topic,
      type,
      extra,
      loose: !(written.has(topic) && read.has(topic)),
      w: Math.ceil(width) + PAD_X * 2,
      h: TOPIC_H,
    });
  }
  return { nodes, edges };
}

/** Two fingers over `element` zoom only the graph, never the page (even with Android's "force enable zoom", which ignores touch-action). */
export function useNoPagePinch(element: HTMLElement | null) {
  useEffect(() => {
    if (!element) {
      return;
    }
    const onTouchMove = (event: TouchEvent) => event.touches.length > 1 && event.preventDefault();
    // iOS Safari's own pinch
    const onGesture = (event: Event) => event.preventDefault();
    element.addEventListener("touchmove", onTouchMove, { passive: false });
    element.addEventListener("gesturestart", onGesture);
    element.addEventListener("gesturechange", onGesture);
    return () => {
      element.removeEventListener("touchmove", onTouchMove);
      element.removeEventListener("gesturestart", onGesture);
      element.removeEventListener("gesturechange", onGesture);
    };
  }, [element]);
}

export const BlueprintGraph = forwardRef<GraphViewHandle, BlueprintGraphProps>(
  function BlueprintGraph(props, ref) {
    const { modules, extras, layout, spot, rates, onModule, onTopic, onHover } = props;
    // sizes follow the skin's fonts
    const { skin } = useTheme();
    // keyed on content, not identity: a new nodes array re-binds GraphView's pointer handlers, which drops a pinch mid-gesture
    const shape = JSON.stringify([modules, extras]);
    const { nodes, edges } = useMemo(() => buildGraph(modules, extras), [shape, skin]);
    const types = useMemo(
      () =>
        new Map(nodes.flatMap((node) => node.kind === "topic" ? [[node.topic, node.type]] : [])),
      [nodes],
    );
    const focus = spot && "module" in spot ? `m:${spot.module}` : spot ? `t:${spot.topic}` : null;
    const hz = (topic: string) => rates?.get(topic) ?? 0;
    return (
      <GraphView<Node, Edge>
        ref={ref}
        className={rates ? "bv-graph running" : "bv-graph"}
        aria-label="Module graph"
        nodes={nodes}
        edges={edges}
        layout={layout}
        focus={focus}
        port={PORT}
        focusable={(node) => node.kind === "module"}
        nodeClassName={(node) =>
          [
            node.kind,
            node.kind === "topic" && node.extra && "extra",
            node.kind === "topic" && node.loose && "loose",
            node.kind === "topic" && hz(node.topic) > 0 && "live",
          ].filter(Boolean).join(" ")}
        nodeTitle={(node) =>
          node.kind === "module"
            ? node.name
            : `/${node.topic}\n${node.type}${node.extra ? "\n(not wired in this blueprint)" : ""}` +
              (node.loose ? "\n(only written or only read here)" : "")}
        edgeClassName={(edge) => hz(edge.topic) > 0 ? "flow" : undefined}
        nodeStyle={(node) =>
          node.kind === "topic"
            ? ({ "--type": messageTypeColor(node.type) }) as CSSProperties
            : undefined}
        edgeStyle={(edge) =>
          ({ "--dr-graph-edge": messageTypeColor(types.get(edge.topic)) }) as CSSProperties}
        onNodeClick={(node) => node.kind === "module" ? onModule(node.name) : onTopic(node.topic)}
        onBackgroundClick={() => onTopic(null)}
        onHover={(node) =>
          onHover(
            node ? (node.kind === "module" ? { module: node.name } : { topic: node.topic }) : null,
          )}
        renderNode={(node) => (
          <>
            <rect className="box" width={node.w} height={node.h} />
            {node.kind === "module"
              ? (
                <>
                  <rect className="bar" width={MODULE_BAR} height={node.h} />
                  <text x={MODULE_BAR + MODULE_PAD_X} y={node.h / 2} className="tag">MOD</text>
                  <text
                    x={MODULE_BAR + MODULE_PAD_X + node.tagW + TAG_GAP}
                    y={node.h / 2}
                    className="name"
                  >
                    {node.label}
                  </text>
                  <text x={node.w - CHEV_W / 2 - 2} y={node.h / 2} className="more">›</text>
                </>
              )
              : (
                <>
                  <text x={PAD_X} y={hz(node.topic) ? 11 : node.h / 2} className="name">
                    {node.topic}
                  </text>
                  <text x={PAD_X} y={22} className="rate">
                    {hz(node.topic) ? formatRate(hz(node.topic)) : ""}
                  </text>
                </>
              )}
            {[0, node.w].map((x) => (
              <rect
                key={x}
                className="port"
                x={x - PORT / 2}
                y={node.h / 2 - PORT / 2}
                width={PORT}
                height={PORT}
              />
            ))}
          </>
        )}
      >
        {props.children}
      </GraphView>
    );
  },
);
