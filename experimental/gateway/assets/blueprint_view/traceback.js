// A Python traceback as text (a log record's `traceback_lines` joined, or its `exception`) split into what the Logs draw:
// frames (library or the project's own), their code and ^^^ marker lines, the exception lines, and an ExceptionGroup's
// "| " gutters and "+-+---- 1 ----" separators kept as they are. Pure (no DOM): checked by traceback_check.js.

const FRAME = /^(\s*)File "(.+)", line (\d+)(?:, in (.*))?$/
const MARKER = /^\s*[\^~]+[\^~\s]*$/
const HEAD = /^(?:Exception Group )?Traceback \(most recent call last\):|^During handling of the above exception|^The above exception was the direct cause/
// an installed package, a venv, the standard library or an interpreter's own frames
const LIBRARY = /\/site-packages\/|\/dist-packages\/|\/\.venv\/|\/lib\/python\d[\d.]*\/|\/uv\/python\/|\/nix\/store\/|^<frozen |^<string>/

/** whether `text` holds a Python traceback */
export const isTraceback = (text) => typeof text === "string" && /Traceback \(most recent call last\):/.test(text)

/** a frame's file as the project names it: from its last `dimos/` folder on (else the whole path) */
export function projectPath(file) {
    const at = file.lastIndexOf("/dimos/")
    return at >= 0 ? file.slice(at + 1) : file
}

/**
 * `text`'s lines as items, in order:
 *   {kind: "sep" | "head" | "text", gutter, text}
 *   {kind: "frame", gutter, file, line, func, library, code: [{text, marker}]}
 *   {kind: "exception", gutter, depth, lines: [text]}  (a message over several lines is one item)
 * `depth`: how deep in an ExceptionGroup (0 at the top)
 */
export function parseTraceback(text) {
    const grouped = /^\s*\+ Exception Group Traceback/m.test(text)
    const items = []
    const last = () => items[items.length - 1]
    for (const raw of text.replace(/\n$/, "").split("\n")) {
        const gutterMatch = grouped ? /^(\s*[|+](?: |$))?(.*)$/.exec(raw) : null
        let gutter = gutterMatch?.[1] ?? ""
        let body = gutterMatch ? gutterMatch[2] : raw
        if (grouped && /^\s*\+-[-+]*(\s.*)?$/.test(raw) && /-{4}/.test(raw)) {
            items.push({ kind: "sep", gutter: "", text: raw })
            continue
        }
        const frame = FRAME.exec(body)
        if (frame) {
            items.push({
                kind: "frame",
                gutter,
                indent: frame[1],
                file: frame[2],
                line: Number(frame[3]),
                func: frame[4] ?? "",
                library: LIBRARY.test(frame[2]),
                code: [],
            })
        } else if (HEAD.test(body)) {
            items.push({ kind: "head", gutter, text: body })
        } else if (last()?.kind === "frame" && /^\s/.test(body) && body.trim()) {
            last().code.push({ text: body, marker: MARKER.test(body) })
        } else if (!body.trim()) {
            items.push({ kind: "text", gutter, text: body })
        } else if (!/^\s/.test(body) && last()?.kind === "exception" && last().gutter === gutter) {
            last().lines.push(body)
        } else if (!/^\s/.test(body)) {
            items.push({ kind: "exception", gutter, depth: depthOf(gutter), lines: [body] })
        } else {
            items.push({ kind: "text", gutter, text: body })
        }
    }
    return items
}

const depthOf = (gutter) => Math.max(0, Math.round((gutter.search(/[|+]/) - 2) / 2))

/** every exception the traceback raised, in order: the outermost first, then an ExceptionGroup's sub-exceptions */
export const exceptionsOf = (items) =>
    items.filter((item) => item.kind === "exception").map(({ depth, lines }) => ({ depth, text: lines.join("\n") }))
