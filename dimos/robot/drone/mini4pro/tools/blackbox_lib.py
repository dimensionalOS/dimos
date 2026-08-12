# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""The importable half of tools/blackbox — query mini4pro flight records from Python.

`tools/blackbox` is a thin shell over this module; everything the CLI can do is one
obvious call here, so an analysis snippet imports this instead of re-writing the same
json-loop for the fourth time in one night:

    sys.path.insert(0, "tools"); import blackbox_lib as bb
    r = bb.Record.open("landing06")
    r.count()["tag"]                          # 321
    t = r.t_of("landing_commit")              # 147.636
    r.lines(kind="tag", around=t, window=2.0) # every solved pose ±2 s of the commit
    r.lines(kind="stick_cmd", around_event="landing_commit", where="dji.throttle>0")
    for op, a, b in bb.diff_events(r, bb.Record.open("landing04")): ...

What a record IS, and what this module refuses to assume:

- One recorder run = one **session**, named by its timestamp prefix `YYYYMMDD-HHMMSS`.
  `.001/.002...jsonl` are parts of that one session — rotation, not restart — and are
  concatenated in name order with `t` continuous across the seam. `.vNNN.h264` are the
  session's video sidecars. A **new prefix means the recorder restarted** (app launch or
  DJI reconnect): a different session, a different `t` origin, a different record.
  `Record.open` therefore refuses to mix prefixes; two sessions only ever meet in
  `diff_events`/`diff_counts`, each opened separately.
- A line is a JSON object with `t` (seconds since recorder start) and `k` (kind) — but
  **kinds have heterogeneous shapes and both fields may be absent on weird lines**, so
  beyond t and k nothing about the schema is assumed. Unparseable lines (a torn last
  line is a normal power-loss artefact, per flightlog) are tolerated, counted, and
  reported — never silently dropped.
- The raw bytes of every line are kept, so `--json` output is byte-faithful and
  therefore quotable in measurement docs.

The `--where` evaluator is a restricted AST walk, ast.literal_eval-grade: names resolve
to row fields (dotted `f.vx` reaches nested objects), comparisons / and / or / not /
arithmetic / `in` only. Calls, subscripts, attributes on anything but a name chain —
refused by name, never worked around.

The phone half (`Phone`, `pull_session`, `rm_session`) reaches the recorder's output
directory on the phone through `tools/phoneadb` — the SSH-to-hyper1 tunnel is the ONLY
route (direct laptop↔phone adb is blackholed by the AP, see wifi-fix.md) — and shells
out to it rather than reimplementing the tunnel. Everything network-touching goes
through `Phone._run`, so the self-test can stand in a fake phoneadb and exercise the
rest for real. A dead tunnel is a normal condition (it dropped mid-command the night
this was written), so every phone path fails with a plain-words sentence naming
phoneadb and the wireless-debugging setting, not a stack trace.

`Record.sql` needs DuckDB, which lives in the DiMOS venv and nowhere else on this
machine; it refuses in plain words when absent. Everything else runs on stdlib
python3 alone.
"""

import ast
import difflib
import glob
import json
import os
import re
import shlex
import subprocess
import sys

TOOLS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TOOLS)
# BLACKBOX_DATASETS exists for worktrees and tests; day to day the datasets sit
# beside tools/ in the main checkout and nobody sets it.
DATASETS = os.environ.get("BLACKBOX_DATASETS") or os.path.join(REPO, "datasets")

PHONE_FLIGHTLOGS = "/sdcard/Android/data/com.dimensional.mini4pro/files/flightlogs"

# Codes that appear in every session regardless of what the flight did; --events
# hides them so the curated timeline is the story of the flight, not the plumbing.
NOISE_CODES = {"zenoh_phase", "zenoh_start", "signal_stale", "signal_fresh"}

DEFAULT_WINDOW = 2.0  # seconds each side of the centre; ±2 s covers a landing commit

_PART_RE = re.compile(r"^(\d{8}-\d{6})\.(\d{3})\.jsonl$")
_VIDEO_RE = re.compile(r"^(\d{8}-\d{6})\.v(\d{3})\.h264$")
_PREFIX_RE = re.compile(r"^(\d{8}-\d{6})\.")


class Refusal(Exception):
    """A named refusal — the tool knows what was asked and declines, saying why."""


# ─────────────────────────────── the session model ───────────────────────────────


class SessionFiles:
    """Every file one recorder run left behind, grouped by its timestamp prefix."""

    def __init__(self, prefix):
        self.prefix = prefix
        self.parts = []  # [(name, size, mtime)] .NNN.jsonl in part order
        self.videos = []  # [(name, size, mtime)] .vNNN.h264 in part order
        self.other = []  # anything else carrying the prefix

    @property
    def jsonl_bytes(self):
        return sum(s for _, s, _ in self.parts)

    @property
    def video_bytes(self):
        return sum(s for _, s, _ in self.videos)

    @property
    def mtime(self):
        all_files = self.parts + self.videos + self.other
        return max((m for _, _, m in all_files if m is not None), default=None)

    @property
    def likely_ground_stub(self):
        """No video sidecar usually means the recorder started and nothing flew.

        A heuristic only: video can also be missing because streaming never came up.
        Callers must print "likely", never assert flight/no-flight from a listing.
        """
        return not self.videos

    def all_names(self):
        return [n for n, _, _ in self.parts + self.videos + self.other]


def split_session(name):
    """basename -> (prefix, role, ordinal). Role is 'part', 'video' or 'other'."""
    m = _PART_RE.match(name)
    if m:
        return m.group(1), "part", int(m.group(2))
    m = _VIDEO_RE.match(name)
    if m:
        return m.group(1), "video", int(m.group(2))
    m = _PREFIX_RE.match(name)
    if m:
        return m.group(1), "other", 0
    return None, "other", 0


def group_sessions(entries):
    """[(name, size, mtime)] -> {prefix: SessionFiles}, prefixes sorted ascending.

    Accepts bare names too (size/mtime None) so tests and remote listings share one
    grouping. Files with no recognisable prefix are ignored — a dataset dir also
    holds .db/.rrd/README files that belong to the dataset, not to a session.
    """
    out = {}
    for entry in entries:
        name, size, mtime = entry if isinstance(entry, tuple) else (entry, None, None)
        prefix, role, ordinal = split_session(os.path.basename(name))
        if prefix is None:
            continue
        sess = out.setdefault(prefix, SessionFiles(prefix))
        bucket = {"part": sess.parts, "video": sess.videos, "other": sess.other}[role]
        bucket.append((name, size, mtime, ordinal))
    for sess in out.values():
        for bucket in (sess.parts, sess.videos, sess.other):
            bucket.sort(key=lambda x: (x[3], x[0]))
            bucket[:] = [(n, s, m) for n, s, m, _ in bucket]
    return dict(sorted(out.items()))


# ─────────────────────────────── record addressing ───────────────────────────────


def _one_session_only(paths, label):
    """The jsonl files of exactly one session, in part order — or a refusal naming
    every session found, because concatenating two recorder runs would splice two
    unrelated t axes into one pretend flight."""
    groups = {}
    for p in paths:
        prefix, role, _ = split_session(os.path.basename(p))
        key = prefix if prefix else os.path.basename(p).rsplit(".jsonl", 1)[0]
        if role in ("part", "other") or prefix is None:
            groups.setdefault(key, []).append(p)
    groups = {k: v for k, v in groups.items() if any(p.endswith(".jsonl") for p in v)}
    if not groups:
        raise Refusal(f"{label}: no .jsonl files there")
    if len(groups) > 1:
        raise Refusal(
            "%s holds %d distinct sessions (%s) — a session is one recorder run and "
            "their t axes are unrelated, so pick one part file explicitly."
            % (label, len(groups), ", ".join(sorted(groups)))
        )
    ((_prefix, files),) = groups.items()
    return sorted(p for p in files if p.endswith(".jsonl"))


def resolve(spec, datasets_dir=None):
    """A record spec -> (label, [jsonl paths of one session, in part order]).

    Accepts: a .jsonl path (its sibling parts are pulled in, flightlog's convention),
    a glob, a dataset directory, or a bare dataset name resolved under datasets/.
    """
    datasets_dir = datasets_dir or DATASETS
    if os.path.isfile(spec):
        base = os.path.basename(spec)
        prefix, role, _ = split_session(base)
        if role == "part":
            sibs = sorted(
                glob.glob(
                    os.path.join(os.path.dirname(spec) or ".", prefix + ".[0-9][0-9][0-9].jsonl")
                )
            )
            return spec, (sibs or [spec])
        return spec, [spec]
    if os.path.isdir(spec):
        files = sorted(glob.glob(os.path.join(spec, "*.jsonl")))
        return spec, _one_session_only(files, spec)
    hits = sorted(glob.glob(spec))
    if hits:
        return spec, _one_session_only([h for h in hits if h.endswith(".jsonl")], spec)
    named = os.path.join(datasets_dir, spec)
    if os.path.isdir(named):
        files = sorted(glob.glob(os.path.join(named, "*.jsonl")))
        return spec, _one_session_only(files, named)
    raise Refusal(
        f"no such record: {spec!r} — not a file, not a glob that matches, and no "
        f"dataset of that name under {datasets_dir}"
    )


# ─────────────────────────────── rows and records ───────────────────────────────


class Row(dict):
    """One parsed line, plus where it came from and its exact bytes."""

    __slots__ = ("file", "lineno", "raw")

    def __init__(self, obj, file, lineno, raw):
        super().__init__(obj)
        self.file = file
        self.lineno = lineno
        self.raw = raw


def field(row, path, default=None):
    """Dotted-path lookup: 'f.vx' walks nested objects; 'file'/'lineno' reach the
    row's provenance when the record itself has no such key."""
    cur = row
    for part in path.split("."):
        if isinstance(cur, dict) and part in cur:
            cur = cur[part]
        elif cur is row and isinstance(row, Row) and part in ("file", "lineno"):
            cur = getattr(row, part)
        else:
            return default
    return cur


class QueryResult(list):
    """The selected rows, plus how many matched before --every/--tail/--limit cut
    the list down — so a footer can distinguish 'shown' from 'matched'."""

    def __init__(self, rows, matched):
        super().__init__(rows)
        self.matched = matched


class Record:
    """One session's record: every line of every part, in file order.

    File order, not t order, deliberately: --json promises the lines as recorded,
    and the recorder's queue interleaves kinds with up to ~90 ms of t jitter between
    adjacent lines (measured on landing06: 454 backsteps, worst 0.091 s), so sorting
    would re-author the record for a cosmetic gain. Time filters test each row's own
    t, so windows are exact either way. flightlog sorts because it joins streams;
    a query tool quotes, so it must not."""

    def __init__(self, label, paths):
        self.label = label
        self.paths = paths
        self.rows = []  # Row per parsed line, header included — queryable
        self.bad = []  # (file, lineno, raw) — counted, never dropped silently
        self.total_lines = 0
        for path in paths:
            with open(path, "rb") as fh:
                for lineno, line in enumerate(fh, 1):
                    line = line.rstrip(b"\r\n")
                    if not line:
                        continue
                    self.total_lines += 1
                    try:
                        obj = json.loads(line)
                        if not isinstance(obj, dict):
                            raise ValueError("not an object")
                        self.rows.append(Row(obj, path, lineno, line))
                    except (ValueError, UnicodeDecodeError):
                        # A torn final line is the recorder's normal power-loss
                        # artefact (flightlog documents it); elsewhere, corruption.
                        # Either way it is counted and reported, not swallowed.
                        self.bad.append((path, lineno, line))

    @classmethod
    def open(cls, spec, datasets_dir=None):
        label, paths = resolve(spec, datasets_dir)
        return cls(label, paths)

    # ── selection ──

    def query(
        self,
        kind=None,
        code=None,
        name=None,
        t=None,
        around=None,
        around_event=None,
        nth=0,
        window=DEFAULT_WINDOW,
        where=None,
        limit=None,
        tail=None,
        every=None,
    ):
        """The one selection path — every filter the CLI has, in one call.

        `code` implies kind=event and `name` implies kind∈{mav_in, mav_out} unless
        `kind` narrows further. Time selection is `t=(a, b)` (either end open),
        `around=T` (±window s), or `around_event="landing_commit"` (±window s around
        the nth such event's own t). Rows with no t cannot be placed in a time range
        and are excluded by one, but still counted in the totals.

        Reduction order, deliberately fixed and documented: filters, then `every`
        (every Nth match, first kept), then `tail` (last N), then `limit` (first N) —
        so `tail=10, limit=5` is the first five of the last ten.
        """
        kinds = _as_set(kind)
        codes = _as_set(code)
        names = _as_set(name)
        if codes and kinds is None:
            kinds = {"event"}
        if names and kinds is None:
            kinds = {"mav_in", "mav_out"}

        if around_event is not None:
            ev = self.event(around_event, nth)
            if ev is None:
                raise Refusal(
                    "%s: no event with code %r (nth=%d) — try --events for "
                    "the codes this record actually has" % (self.label, around_event, nth)
                )
            if "t" not in ev:
                raise Refusal(
                    f"{self.label}: event {around_event!r} has no t to centre a window on"
                )
            around = ev["t"]
        t0, t1 = t if t is not None else (None, None)
        if around is not None:
            t0, t1 = around - window, around + window

        pred = compile_where(where) if isinstance(where, str) else where
        matched = []
        for row in self.rows:
            if kinds is not None and row.get("k") not in kinds:
                continue
            if codes and row.get("code") not in codes:
                continue
            if names and row.get("name") not in names:
                continue
            if t0 is not None or t1 is not None:
                rt = row.get("t")
                if not isinstance(rt, (int, float)):
                    continue
                if t0 is not None and rt < t0:
                    continue
                if t1 is not None and rt > t1:
                    continue
            if pred is not None and not pred(row):
                continue
            matched.append(row)

        rows = matched
        if every is not None and every > 1:
            rows = rows[::every]
        if tail is not None:
            rows = rows[-tail:] if tail > 0 else []
        if limit is not None:
            rows = rows[:limit]
        return QueryResult(rows, len(matched))

    def lines(self, **kw):
        """query() as a plain list, for callers that don't need the matched count."""
        return list(self.query(**kw))

    # ── conveniences the CLI presets sit on ──

    def events(self, all=False):
        """The curated event timeline; all=True includes the NOISE_CODES plumbing."""
        rows = self.lines(kind="event")
        if all:
            return rows
        return [r for r in rows if r.get("code") not in NOISE_CODES]

    def event(self, code, nth=0):
        """The nth event with this code (0-based), or None. Searches ALL events —
        noise curation is a display choice, not an addressing one."""
        n = 0
        for row in self.rows:
            if row.get("k") == "event" and row.get("code") == code:
                if n == nth:
                    return row
                n += 1
        return None

    def t_of(self, code, nth=0):
        ev = self.event(code, nth)
        return ev.get("t") if ev else None

    def count(self):
        """{kind: n} over every parsed line; kindless lines count under '(no k)'."""
        out = {}
        for row in self.rows:
            k = row.get("k") if isinstance(row.get("k"), str) else "(no k)"
            out[k] = out.get(k, 0) + 1
        return out

    def t_span(self):
        ts = [r["t"] for r in self.rows if isinstance(r.get("t"), (int, float))]
        return (min(ts), max(ts)) if ts else (None, None)

    # ── the SQL escape hatch ──

    def sql(self, query):
        """DuckDB over the record. View `rec` is the bookkeeping-only spine —
        (file, lineno, t, k, raw JSON) — and every kind present gets a same-named
        view whose JSON fields are REAL COLUMNS: `SELECT t, code, msg FROM event`,
        `sum(CASE WHEN px>=60 THEN 1 ELSE 0 END) FROM tag`. Returns (columns, rows).

        Each per-kind view is built by merging that kind's JSON structure across all
        its rows (DuckDB's json_group_structure — union-by-name, so heterogeneous
        rows just yield NULLs where a field is absent) and transforming `raw` with
        it. Nested objects become STRUCT columns under their own name: `f.vx` on
        mav_out reads the nested field directly. A field whose type varies across
        rows (the recorder writes non-finite floats as strings, e.g. tag.e2 =
        "Infinity") stays a JSON column rather than being coerced into a lie.
        `file`, `lineno` and `raw` ride along on every view.

        DuckDB lives in the DiMOS venv and nowhere else on this machine; the CLI
        re-execs itself there, but an API caller on plain python3 gets this refusal.
        """
        try:
            import duckdb
        except ImportError:
            raise Refusal(
                "DuckDB is not importable here. It lives in the DiMOS venv:\n"
                "    ~/coding/dimos/.venv/bin/python your_script.py\n"
                "or run the query through `tools/blackbox --sql`, which re-execs "
                "there by itself. Everything except .sql() runs on stdlib python3."
            )
        con = duckdb.connect()
        con.execute("CREATE TABLE rec (file VARCHAR, lineno BIGINT, t DOUBLE, k VARCHAR, raw JSON)")
        con.executemany(
            "INSERT INTO rec VALUES (?, ?, ?, ?, ?)",
            [
                (
                    r.file,
                    r.lineno,
                    r.get("t") if isinstance(r.get("t"), (int, float)) else None,
                    r.get("k") if isinstance(r.get("k"), str) else None,
                    r.raw.decode("utf-8", "replace"),
                )
                for r in self.rows
            ],
        )
        for k in sorted(self.count()):
            # CREATE VIEW cannot be a prepared statement, so everything is inlined —
            # the kind name is safe because it just matched a bare-identifier regex,
            # and the structure literal gets its quotes doubled.
            if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", k) or k.lower() == "rec":
                continue
            structure = con.execute(
                f"SELECT json_group_structure(raw) FROM rec WHERE k = '{k}'"
            ).fetchone()[0]
            con.execute(
                'CREATE VIEW "{}" AS SELECT file, lineno, raw, '
                "unnest(json_transform(raw, '{}')) FROM rec WHERE k = '{}'".format(
                    k, str(structure).replace("'", "''"), k
                )
            )
        cur = con.execute(query)
        cols = [d[0] for d in cur.description] if cur.description else []
        return cols, cur.fetchall()


def _as_set(v):
    if v is None:
        return None
    if isinstance(v, str):
        return {s for s in (p.strip() for p in v.split(",")) if s}
    return set(v)


# ─────────────────────────────── the --where evaluator ───────────────────────────────

# Deliberately literal-eval-grade. Anything not in this table is refused BY NAME —
# a predicate language that silently ignores what it can't do produces filters that
# lie, and one that eval()s produces shells.
_BOOL_OPS = {ast.And: all, ast.Or: any}
_BIN_OPS = {
    ast.Add: lambda a, b: a + b,
    ast.Sub: lambda a, b: a - b,
    ast.Mult: lambda a, b: a * b,
    ast.Div: lambda a, b: a / b,
    ast.FloorDiv: lambda a, b: a // b,
    ast.Mod: lambda a, b: a % b,
}
_CMP_OPS = {
    ast.Eq: lambda a, b: a == b,
    ast.NotEq: lambda a, b: a != b,
    ast.Lt: lambda a, b: a < b,
    ast.LtE: lambda a, b: a <= b,
    ast.Gt: lambda a, b: a > b,
    ast.GtE: lambda a, b: a >= b,
    ast.In: lambda a, b: a in b,
    ast.NotIn: lambda a, b: a not in b,
}
_REFUSE_NAMES = {
    ast.Call: "a function call",
    ast.Subscript: "subscripting",
    ast.Lambda: "a lambda",
    ast.IfExp: "a conditional expression",
    ast.Dict: "a dict literal",
    ast.Set: "a set literal",
    ast.ListComp: "a comprehension",
    ast.SetComp: "a comprehension",
    ast.DictComp: "a comprehension",
    ast.GeneratorExp: "a comprehension",
    ast.Starred: "star-unpacking",
    ast.FormattedValue: "an f-string",
    ast.JoinedStr: "an f-string",
    ast.NamedExpr: "the walrus operator",
    ast.Await: "await",
    ast.Yield: "yield",
    ast.YieldFrom: "yield",
    ast.Pow: "** (unbounded arithmetic)",
    ast.MatMult: "@",
    ast.LShift: "<<",
    ast.RShift: ">>",
    ast.BitOr: "|",
    ast.BitAnd: "&",
    ast.BitXor: "^",
    ast.Is: "'is' (compare with == instead)",
    ast.IsNot: "'is not'",
}


def _refuse(node, expr):
    name = _REFUSE_NAMES.get(type(node), type(node).__name__)
    raise Refusal(
        f"--where refused: {name} in {expr!r}. The predicate language is field names "
        "(dotted for nested, e.g. f.vx), literals, comparisons, and/or/not, "
        "in, and + - * / % arithmetic — nothing else, by design."
    )


def _dotted(node):
    """Attribute chains over a Name are the one 'attribute' allowed — they are field
    path sugar (f.vx), not Python attribute access."""
    parts = []
    while isinstance(node, ast.Attribute):
        parts.append(node.attr)
        node = node.value
    if isinstance(node, ast.Name):
        parts.append(node.id)
        return ".".join(reversed(parts))
    return None


class _Missing:
    def __repr__(self):
        return "<missing>"


_MISSING = _Missing()
_CONSTS = {"True": True, "False": False, "None": None, "true": True, "false": False, "null": None}


def compile_where(expr):
    """expr -> predicate(row). Raises Refusal at compile time for anything outside
    the language, so a bad predicate fails before the first row, not on row 19000.

    Missing fields become a sentinel: any comparison against it is False (and == None
    is True only for a field that is present and null), so `px>60` simply doesn't
    match rows that have no px, instead of crashing or matching."""
    try:
        tree = ast.parse(expr, mode="eval")
    except SyntaxError as e:
        raise Refusal(f"--where is not a valid expression: {expr} ({e})")

    for node in ast.walk(tree):
        if type(node) in _REFUSE_NAMES:
            _refuse(node, expr)
        if isinstance(node, ast.Attribute) and _dotted(node) is None:
            _refuse(node, expr)
        if isinstance(node, ast.Constant) and not isinstance(
            node.value, (int, float, str, bool, type(None))
        ):
            _refuse(node, expr)

    def ev(node, row):
        if isinstance(node, ast.Expression):
            return ev(node.body, row)
        if isinstance(node, ast.Constant):
            return node.value
        if isinstance(node, (ast.Name, ast.Attribute)):
            path = _dotted(node)
            if path in _CONSTS:
                return _CONSTS[path]
            v = field(row, path, _MISSING)
            return v
        if isinstance(node, (ast.Tuple, ast.List)):
            return [ev(e, row) for e in node.elts]
        if isinstance(node, ast.BoolOp):
            comb = _BOOL_OPS[type(node.op)]
            return comb(_truthy(ev(v, row)) for v in node.values)
        if isinstance(node, ast.UnaryOp):
            if isinstance(node.op, ast.Not):
                return not _truthy(ev(node.operand, row))
            v = ev(node.operand, row)
            if v is _MISSING or not isinstance(v, (int, float)):
                return _MISSING
            return -v if isinstance(node.op, ast.USub) else +v
        if isinstance(node, ast.BinOp):
            a, b = ev(node.left, row), ev(node.right, row)
            if a is _MISSING or b is _MISSING:
                return _MISSING
            try:
                return _BIN_OPS[type(node.op)](a, b)
            except (TypeError, ZeroDivisionError):
                return _MISSING
        if isinstance(node, ast.Compare):
            left = ev(node.left, row)
            for op, rnode in zip(node.ops, node.comparators, strict=False):
                right = ev(rnode, row)
                if left is _MISSING or right is _MISSING:
                    return False
                try:
                    if not _CMP_OPS[type(op)](left, right):
                        return False
                except TypeError:
                    return False
                left = right
            return True
        _refuse(node, expr)

    def pred(row):
        return _truthy(ev(tree, row))

    pred.expr = expr
    return pred


def _truthy(v):
    return False if v is _MISSING else bool(v)


# ─────────────────────────────── diffing two flights ───────────────────────────────


def diff_events(a, b, align="code-sequence"):
    """Compare two records' curated event timelines — 'did this flight take the same
    path as the last one'.

    Returns [(op, row_a, row_b)] where op is ' ' (same code in both), '-' (only in a),
    '+' (only in b); each row is {'t', 'dt', 'code', 'msg', 'sev'}, dt = seconds since
    the previous curated event in its OWN record. Alignment is difflib over the code
    sequences and nothing cleverer — 'code-sequence' is the only mode, on purpose: an
    alignment you can't predict is an alignment you can't trust in a measurement doc.
    """
    if align != "code-sequence":
        raise Refusal(
            f"diff_events: unknown align {align!r} — 'code-sequence' is the only "
            "alignment on offer, deliberately"
        )

    def timeline(rec):
        rows, prev_t = [], None
        for e in rec.events():
            t = e.get("t")
            dt = (t - prev_t) if isinstance(t, (int, float)) and prev_t is not None else None
            if isinstance(t, (int, float)):
                prev_t = t
            rows.append(
                {"t": t, "dt": dt, "code": e.get("code"), "msg": e.get("msg"), "sev": e.get("sev")}
            )
        return rows

    ta, tb = timeline(a), timeline(b)
    sm = difflib.SequenceMatcher(
        None, [r["code"] for r in ta], [r["code"] for r in tb], autojunk=False
    )
    out = []
    for tag, i1, i2, j1, j2 in sm.get_opcodes():
        if tag == "equal":
            out.extend(
                (" ", ta[i], tb[j]) for i, j in zip(range(i1, i2), range(j1, j2), strict=False)
            )
        else:
            out.extend(("-", ta[i], None) for i in range(i1, i2))
            out.extend(("+", None, tb[j]) for j in range(j1, j2))
    return out


def diff_counts(a, b):
    """Per-kind and per-event-code count deltas between two records.

    Returns {'kind': [(key, na, nb)], 'event_code': [(key, na, nb)]}, every key from
    either side, sorted; equal rows included so the caller decides what to show.
    """

    def codes(rec):
        out = {}
        for e in rec.lines(kind="event"):
            c = e.get("code") or "(no code)"
            out[c] = out.get(c, 0) + 1
        return out

    def merge(ca, cb):
        return [(k, ca.get(k, 0), cb.get(k, 0)) for k in sorted(set(ca) | set(cb))]

    return {"kind": merge(a.count(), b.count()), "event_code": merge(codes(a), codes(b))}


# ─────────────────────────────── the phone ───────────────────────────────


class PhoneDown(Exception):
    """The tunnel or the phone is not there. Normal, not exceptional — say what to do."""


_TUNNEL_ADVICE = (
    "Bring the tunnel up with `tools/phoneadb up`; if that fails, Wireless debugging "
    "(Settings > Developer options > Wireless debugging) is probably off on the phone, "
    "and only the phone can turn it back on."
)


class Phone:
    """The recorder's output directory on the phone, via the tools/phoneadb tunnel.

    Every adb invocation goes through `_run` — one choke point, so the self-test can
    substitute a fake phoneadb (BLACKBOX_PHONEADB or the ctor argument) and every
    parsing/verification branch above it runs for real, the flightlog planted-fault
    pattern. The serial is pinned to the wifi IP transport when the phone shows up
    twice (IP:port plus mDNS); the port changes between wireless-debugging toggles,
    so it is discovered from `adb devices` on every run, never remembered.
    """

    def __init__(self, phoneadb=None, prefer="10.55.1.15"):
        self.phoneadb = (
            phoneadb or os.environ.get("BLACKBOX_PHONEADB") or os.path.join(TOOLS, "phoneadb")
        )
        self.prefer = prefer
        self._serial = None

    def _run(self, *args, may_fail=False):
        env = dict(os.environ)
        if self._serial:
            env["ANDROID_SERIAL"] = self._serial
        try:
            cp = subprocess.run(
                [self.phoneadb, *args], capture_output=True, text=True, env=env, timeout=600
            )
        except FileNotFoundError:
            raise PhoneDown(f"cannot run {self.phoneadb} — is this checkout intact?")
        except subprocess.TimeoutExpired:
            raise PhoneDown(
                f"phoneadb {args[0]} timed out after 600 s — the tunnel is "
                f"probably half-dead. {_TUNNEL_ADVICE}"
            )
        if cp.returncode != 0 and not may_fail:
            detail = (cp.stderr or cp.stdout).strip() or "exit %d" % cp.returncode
            raise PhoneDown(
                f"the phone is not reachable (phoneadb {args[0]} failed: {detail.splitlines()[-1]}). {_TUNNEL_ADVICE}"
            )
        return cp

    def attach(self):
        """Pick the serial to pin. Prefers the 10.55.1.15:* transport — when wireless
        debugging re-registers, the same phone can appear as both IP:port and an mDNS
        name, and commands sent without a serial then fail with 'more than one
        device'."""
        if self._serial:
            return self._serial
        cp = self._run("devices")
        serials = []
        for line in cp.stdout.splitlines():
            parts = line.split()
            if len(parts) >= 2 and parts[1] == "device":
                serials.append(parts[0])
        if not serials:
            raise PhoneDown(
                "no phone attached to the adb server behind the tunnel. " + _TUNNEL_ADVICE
            )
        preferred = [s for s in serials if s.startswith(self.prefer + ":")]
        self._serial = (preferred or serials)[0]
        return self._serial

    def stat_all(self, directory=PHONE_FLIGHTLOGS):
        """[(basename, size, mtime_unix)] for every file in the directory."""
        self.attach()
        cp = self._run("shell", f"stat -c '%s %Y %n' {directory}/* 2>/dev/null; true")
        out = []
        for line in cp.stdout.splitlines():
            m = re.match(r"^(\d+) (\d+) (.+)$", line.strip())
            if m:
                out.append((os.path.basename(m.group(3)), int(m.group(1)), int(m.group(2))))
        return out

    def sessions(self, directory=PHONE_FLIGHTLOGS):
        """{prefix: SessionFiles} for the phone's flightlogs directory."""
        return group_sessions(self.stat_all(directory))

    def latest(self, directory=PHONE_FLIGHTLOGS):
        """Newest session prefix ON THE PHONE. Prefixes are timestamps, so newest is
        the lexicographic maximum — no mtime guessing."""
        sessions = self.sessions(directory)
        if not sessions:
            raise PhoneDown(
                f"no sessions in {directory} on the phone — nothing recorded, or "
                "the wrong directory"
            )
        return max(sessions)

    def pull_file(self, remote, local):
        """adb pull one file, and return adb's own transfer chatter for printing."""
        self.attach()
        cp = self._run("pull", remote, local)
        return (cp.stdout or cp.stderr).strip()

    def rm_files(self, remotes):
        self.attach()
        self._run("shell", "rm -- " + " ".join(shlex.quote(r) for r in remotes))


def pull_session(
    phone, prefix, dest, into_existing=False, out=sys.stdout, directory=PHONE_FLIGHTLOGS
):
    """Pull every part and sidecar of one session into dest/, then verify sizes.

    The verification is not paranoia: adb pull over the tunnel can be cut mid-file
    when the wifi-debugging link drops and leaves a silently truncated local copy —
    measured live on 2026-07-29. Every pulled file's size is compared against the
    phone's stat; a mismatch names the file and both sizes and fails the pull.

    Refuses a non-empty dest by name unless into_existing — pulling one session on
    top of another is how a dataset dir ends up holding two unrelated t axes.
    """
    if prefix == "latest":
        prefix = phone.latest(directory)
        out.write(f"latest on the phone: {prefix}\n")
    sessions = phone.sessions(directory)
    if prefix not in sessions:
        raise Refusal(
            "no session {} on the phone. There: {}".format(prefix, ", ".join(sessions) or "nothing")
        )
    sess = sessions[prefix]
    if os.path.isdir(dest) and os.listdir(dest) and not into_existing:
        raise Refusal(
            f"{dest} exists and is not empty — refusing to mix sessions into "
            "one dataset dir. Pick another --as name, or pass "
            "--into-existing if this is deliberate."
        )
    os.makedirs(dest, exist_ok=True)

    expected = [(n, s) for n, s, _ in sess.parts + sess.videos + sess.other]
    total = 0
    for name, size in expected:
        remote = f"{directory}/{name}"
        local = os.path.join(dest, name)
        chatter = phone.pull_file(remote, local)
        out.write("  %s\n" % (chatter or (f"pulled {name}")))
        have = os.path.getsize(local) if os.path.exists(local) else -1
        if size is not None and have != size:
            raise Refusal(
                "%s: pulled %d bytes but the phone has %d — the pull was truncated "
                "(the tunnel dropping mid-transfer does exactly this, silently). "
                "Re-run the pull; do not analyse this copy." % (name, have, size)
            )
        total += have
    out.write(
        "pulled %d files, %d bytes, verified against the phone's sizes\n" % (len(expected), total)
    )
    out.write(
        "mem2 store NOT generated (deliberate step): tools/memexport {}\n".format(
            os.path.join(dest, f"{prefix}.001.jsonl")
        )
    )
    return prefix, [n for n, _ in expected]


def rm_session(phone, prefix, directory=PHONE_FLIGHTLOGS):
    """Delete one session's files from the PHONE. The caller (CLI) owns the
    are-you-sure gate; this just refuses vagueness — an explicit prefix, never
    'latest', because deleting the flight you just flew is a one-keystroke mistake."""
    if prefix == "latest":
        raise Refusal(
            "rm latest is refused: name the prefix explicitly (blackbox ls "
            "--phone shows them) — 'latest' is one flight-you-just-flew "
            "away from being a disaster."
        )
    sessions = phone.sessions(directory)
    if prefix not in sessions:
        raise Refusal(
            "no session {} on the phone. There: {}".format(prefix, ", ".join(sessions) or "nothing")
        )
    names = sessions[prefix].all_names()
    phone.rm_files([f"{directory}/{n}" for n in names])
    return names
