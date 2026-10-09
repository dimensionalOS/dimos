// traceback.js on an ExceptionGroup's traceback and a chained one: what each line is. Run by test_blueprint_view.py
// (`deno run traceback_check.js`); prints one JSON line.

import { exceptionsOf, parseTraceback, projectPath } from "./traceback.js"

const GROUPED = [
    "  + Exception Group Traceback (most recent call last):\n",
    '  |   File "/r/.venv/bin/dimos", line 10, in <module>\n  |     sys.exit(main())\n  |              ^^^^^^\n',
    '  |   File "/r/dimos/utils/safe_thread_map.py", line 84, in safe_thread_map\n  |     raise ExceptionGroup("safe_thread_map failed", errors)\n',
    "  | ExceptionGroup: safe_thread_map failed (1 sub-exception)\n",
    "  +-+---------------- 1 ----------------\n",
    "    | Traceback (most recent call last):\n",
    '    |   File "/u/lib/python3.12/concurrent/futures/thread.py", line 59, in run\n    |     result = self.fn()\n',
    '    |   File "/r/dimos/core/coordination/python_worker.py", line 246, in deploy_module\n    |     raise RuntimeError("no")\n',
    "    | RuntimeError: no\n",
    "    | second line of the message\n",
    "    +------------------------------------\n",
].join("")

const CHAINED = [
    "Traceback (most recent call last):",
    '  File "/r/dimos/a.py", line 3, in f',
    "    g()",
    "KeyError: 'x'",
    "",
    "The above exception was the direct cause of the following exception:",
    "",
    "Traceback (most recent call last):",
    '  File "/r/.venv/lib/python3.12/site-packages/click/core.py", line 9, in main',
    "    f()",
    "ValueError: bad",
    "",
].join("\n")

const shape = (items) =>
    items.map((item) => item.kind === "frame" ? `frame:${item.library ? "lib" : "own"}:${item.line}:${item.code.length}` : item.kind)

console.log(JSON.stringify({
    grouped: shape(parseTraceback(GROUPED)),
    groupedRaised: exceptionsOf(parseTraceback(GROUPED)),
    markers: parseTraceback(GROUPED)[1].code.map((code) => code.marker),
    gutter: parseTraceback(GROUPED)[6].gutter,
    chained: shape(parseTraceback(CHAINED)),
    chainedRaised: exceptionsOf(parseTraceback(CHAINED)),
    path: projectPath("/Users/x/repos/dimos_live/dimos/cli/entry.py"),
}))
