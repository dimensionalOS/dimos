// The side panel: the blueprint's topic rates and its modules, or one module's streams, skills and RPCs.
import {
  Badge,
  Button,
  Disclosure,
  messageTypeColor,
  messageTypeName,
  Tooltip,
  type TopicRate,
  TopicRates,
} from "dimos-react";
import { type CSSProperties, useState } from "react";
import type { Where } from "./code.tsx";
import {
  aliasOf,
  className,
  type Method,
  type Module,
  reads,
  type Stream,
  topicOf,
  writes,
} from "./model.ts";

const typeStyle = (type: string | null | undefined) =>
  ({ "--type": messageTypeColor(type) }) as CSSProperties;

/** The bus's rates, then this blueprint's topics no one has heard yet. */
export function rateRows(heardRows: TopicRate[], modules: Module[] | null): TopicRate[] {
  const heard = new Set(heardRows.map((row) => `${row.topic} ${row.type}`));
  const known = new Set(heardRows.map((row) => row.topic));
  const unheard: TopicRate[] = [];
  for (const module of modules ?? []) {
    for (const stream of module.streams) {
      const topic = `/${topicOf(stream)}`;
      if (!known.has(topic) && !heard.has(`${topic} ${stream.type}`)) {
        heard.add(`${topic} ${stream.type}`);
        known.add(topic);
        unheard.push({ topic, type: stream.type, hz: 0, bps: 0 });
      }
    }
  }
  return [...heardRows, ...unheard.sort((a, b) => a.topic.localeCompare(b.topic))];
}

function StreamPills({ label, list }: { label: string; list: Stream[] }) {
  return (
    <section>
      <h4 className="dr-label">
        {label}
        <span className="count">{list.length}</span>
      </h4>
      <div className="pills">
        {list.length
          ? list.map((stream) => (
            <span
              key={stream.name}
              className="pill"
              title={stream.type}
              style={typeStyle(stream.type)}
            >
              {stream.name}
              <span className="stype">{messageTypeName(stream.type)}</span>
            </span>
          ))
          : <span className="note">none</span>}
      </div>
    </section>
  );
}

export function ModuleList(
  { modules, error, rates, ratesError, onOpen, onHover }: {
    modules: Module[] | null;
    error: string | null;
    rates: TopicRate[];
    ratesError: string | null;
    onOpen: (name: string) => void;
    onHover: (name: string | null) => void;
  },
) {
  const rows = rateRows(rates, modules);
  return (
    <div className="list">
      <Disclosure
        persistKey="bp.ratesOpen"
        title="Topic rates"
        count={rows.length
          ? `${rows.filter((row) => (row.hz ?? 0) > 0).length}/${rows.length} live`
          : null}
      >
        <TopicRates rows={rows} empty={ratesError} />
      </Disclosure>
      <Disclosure persistKey="bp.modulesOpen" title="Modules" count={modules?.length}>
        {error && <div className="note">{error}</div>}
        {!modules && !error && <div className="note">loading…</div>}
        <ul onMouseLeave={() => onHover(null)}>
          {(modules ?? []).map((module) => {
            const inputs = module.streams.filter(reads);
            const outputs = module.streams.filter(writes);
            return (
              <li
                key={module.name}
                onFocus={() => onHover(module.name)}
                onBlur={() => onHover(null)}
              >
                <Tooltip
                  placement="right-start"
                  delay={80}
                  content={
                    <div className="flyout">
                      <StreamPills label="Inputs" list={inputs} />
                      <StreamPills label="Outputs" list={outputs} />
                    </div>
                  }
                >
                  <button
                    type="button"
                    className="module"
                    data-bp-module={module.name}
                    onClick={() => onOpen(module.name)}
                    onMouseEnter={() => onHover(module.name)}
                  >
                    <span className="name">
                      {className(module)}
                      {aliasOf(module) && <span className="alias">{module.name}</span>}
                    </span>
                    {module.summary && <span className="summary">{module.summary}</span>}
                    <span className="io">
                      <Badge tone="ok">Inputs {inputs.length}</Badge>
                      <Badge tone="info">Outputs {outputs.length}</Badge>
                    </span>
                    <span className="chev" aria-hidden="true">›</span>
                  </button>
                </Tooltip>
              </li>
            );
          })}
        </ul>
      </Disclosure>
    </div>
  );
}

export function CodeButton(
  { where, shown, onToggle, ...data }: {
    where: Where;
    shown: Where | null;
    onToggle: (where: Where | null) => void;
    [data: `data-${string}`]: boolean;
  },
) {
  const on = shown?.file === where.file && shown?.line === where.line;
  return (
    <Button size="sm" pressed={on} onClick={() => onToggle(on ? null : where)} {...data}>
      {on ? "Hide code" : "Show code"}
    </Button>
  );
}

function Ends(
  { label, list, self, onOpen }: {
    label: string;
    list: Module[];
    self: string;
    onOpen: (name: string) => void;
  },
) {
  return (
    <div className="end">
      <span className="k">{label}</span>
      {list.length === 0 && <span className="note">nothing</span>}
      {list.map((m) =>
        m.name === self
          ? <span key={m.name} className="self">{className(m)}</span>
          : (
            <button key={m.name} type="button" data-bp-jump={m.name} onClick={() => onOpen(m.name)}>
              {className(m)}
            </button>
          )
      )}
    </div>
  );
}

function StreamRow(
  { module, stream, modules, expanded, onExpand, onOpen }: {
    module: Module;
    stream: Stream;
    modules: Module[];
    expanded: string | null;
    onExpand: (key: string | null, spot: { topic: string } | null) => void;
    onOpen: (name: string) => void;
  },
) {
  const topic = topicOf(stream);
  const topicKey = `topic:${topic}`;
  const typeKey = `type:${stream.type}`;
  const on = (test: (s: Stream) => boolean, match: (s: Stream) => boolean) =>
    modules.filter((m) => m.streams.some((s) => match(s) && test(s)));
  const sameTopic = (s: Stream) => topicOf(s) === topic;
  const sameType = (s: Stream) => s.type === stream.type;
  return (
    <div className="stream" style={typeStyle(stream.type)}>
      <div className="row">
        <i className="dot" />
        <button
          type="button"
          className={`sname${expanded === topicKey ? " on" : ""}`}
          title={`who else is on /${topic} (lit in the graph)`}
          data-bp-stream={stream.name}
          onClick={() => onExpand(expanded === topicKey ? null : topicKey, { topic })}
        >
          {stream.name}
        </button>
        <button
          type="button"
          className={`stype${expanded === typeKey ? " on" : ""}`}
          title={`${stream.type}: every module with one`}
          onClick={() => onExpand(expanded === typeKey ? null : typeKey, null)}
        >
          {messageTypeName(stream.type)}
        </button>
      </div>
      {expanded === topicKey && (
        <div className="ends">
          <div className="topic">/{topic}</div>
          <Ends label="from" list={on(writes, sameTopic)} self={module.name} onOpen={onOpen} />
          <Ends label="to" list={on(reads, sameTopic)} self={module.name} onOpen={onOpen} />
        </div>
      )}
      {expanded === typeKey && (
        <div className="ends">
          <div className="topic">{stream.type}</div>
          <Ends label="out" list={on(writes, sameType)} self={module.name} onOpen={onOpen} />
          <Ends label="in" list={on(reads, sameType)} self={module.name} onOpen={onOpen} />
        </div>
      )}
    </div>
  );
}

function MethodCard(
  { method, open, onToggle }: { method: Method; open: boolean; onToggle: () => void },
) {
  const doc = (method.doc ?? "").trim();
  const returns = method.return_type ?? "None";
  const params = method.params.length
    ? method.params.map((p) =>
      `${p.name}${p.type ? `: ${p.type}` : ""}${
        open && p.default !== null ? ` = ${p.default}` : ""
      }`
    ).join(", ")
    : "no params";
  return (
    <button
      type="button"
      className={`card${open ? " open" : ""}`}
      aria-expanded={open}
      title={doc || undefined}
      onClick={onToggle}
    >
      <span className="fn">
        {method.name.split(/(?<=_)/).map((part, i) => (
          <span key={i}>{i ? <wbr /> : null}{part}</span>
        ))}
      </span>
      <span className="ps">{params}</span>
      {doc && (
        <span className="d">{open ? doc : doc.split(/\n\s*\n/)[0].replace(/\s*\n\s*/g, " ")}</span>
      )}
      <span className="r" style={typeStyle(returns)}>
        → {messageTypeName(returns)}
      </span>
    </button>
  );
}

function Methods({ title, list, skill }: { title: string; list: Method[]; skill: boolean }) {
  const [open, setOpen] = useState<string | null>(null);
  return (
    <section className={skill ? "skills" : "rpcs"}>
      <h4 className="dr-label">
        {title}
        <span className="count">{list.length}</span>
      </h4>
      <div className="cards">
        {list.map((method) => (
          <MethodCard
            key={method.name}
            method={method}
            open={open === method.name}
            onToggle={() => setOpen(open === method.name ? null : method.name)}
          />
        ))}
      </div>
    </section>
  );
}

export function ModuleView(
  { module, modules, code, onCode, onOpen, onSpot }: {
    module: Module;
    modules: Module[];
    code: Where | null;
    onCode: (where: Where | null) => void;
    onOpen: (name: string | null) => void;
    onSpot: (spot: { topic: string } | null) => void;
  },
) {
  const [expanded, setExpanded] = useState<string | null>(null);
  const rpcs = module.rpcs ?? [];
  const skills = module.skills ?? [];
  const known = module.doc !== undefined;
  const expand = (key: string | null, spot: { topic: string } | null) => {
    setExpanded(key);
    onSpot(key && spot ? spot : null);
  };
  const streams = (title: string, list: Stream[]) => (
    <section>
      <h4 className="dr-label">{title}</h4>
      {list.length === 0 && <div className="note">none</div>}
      {list.map((stream) => (
        <StreamRow
          key={`${stream.direction}:${stream.name}`}
          module={module}
          stream={stream}
          modules={modules}
          expanded={expanded}
          onExpand={expand}
          onOpen={onOpen}
        />
      ))}
    </section>
  );
  return (
    <div className="module-view" data-bp-module-view={module.name}>
      <nav>
        <Button size="sm" variant="ghost" onClick={() => onOpen(null)} data-bp-back>
          ‹ Modules
        </Button>
      </nav>
      <div className="title">
        <div className="names">
          <h3 title={module.class}>{className(module)}</h3>
          {aliasOf(module) && <span className="alias">{module.name}</span>}
        </div>
        {module.file && (
          <CodeButton
            where={{ file: module.file, line: module.line ?? 1 }}
            shown={code}
            onToggle={onCode}
            data-bp-show-code
          />
        )}
      </div>
      {module.doc ? <div className="doc">{module.doc}</div> : (
        <div className="note">
          {known ? "No docstring." : "This dimos is too old to describe its modules."}
        </div>
      )}
      <div className="io">
        {streams("Inputs", module.streams.filter(reads))}
        {streams("Outputs", module.streams.filter(writes))}
      </div>
      {skills.length > 0 && <Methods title="Skills" list={skills} skill />}
      {rpcs.length > 0 && <Methods title="RPC methods" list={rpcs} skill={false} />}
      {known && skills.length + rpcs.length === 0 && (
        <section>
          <h4 className="dr-label">RPC methods</h4>
          <div className="note">none of its own</div>
        </section>
      )}
    </div>
  );
}
