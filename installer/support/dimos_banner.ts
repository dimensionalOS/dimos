import logUpdate from "https://esm.sh/log-update@7.0.2 ";
import ansiEscapes from "https://esm.sh/ansi-escapes@7.2.0";

const DEFAULT_BANNER = [
    " ██████╗ ██╗███╗   ███╗███████╗███╗   ██╗███████╗██╗ ██████╗ ███╗   ██╗ █████╗ ██╗           ██████╗ ███████╗",
    "██╔══██╗██║████╗ ████║██╔════╝████╗  ██║██╔════╝██║██╔═══██╗████╗  ██║██╔══██╗██║          ██╔═══██╗██╔════╝",
    "██║  ██║██║██╔████╔██║█████╗  ██╔██╗ ██║███████╗██║██║   ██║██╔██╗ ██║███████║██║          ██║   ██║███████╗",
    "██║  ██║██║██║╚██╔╝██║██╔══╝  ██║╚██╗██║╚════██║██║██║   ██║██║╚██╗██║██╔══██║██║          ██║   ██║╚════██║",
    "██████╔╝██║██║ ╚═╝ ██║███████╗██║ ╚████║███████║██║╚██████╔╝██║ ╚████║██║  ██║███████╗     ╚██████╔╝███████║",
    "╚═════╝ ╚═╝╚═╝     ╚═╝╚══════╝╚═╝  ╚═══╝╚══════╝╚═╝ ╚═════╝ ╚═╝  ╚═══╝╚═╝  ╚═╝╚══════╝      ╚═════╝ ╚══════╝",
    "",
    "                                       D I M E N S I O N A L   O S                                ",
];

export class RenderLogo {
    constructor({
        banner = DEFAULT_BANNER,
        glitchyness = 10,
        stickyness = 14,
        fps = 30,
        waveStrength = 10,
        waveSpeed = 0.1,
        waveFreq = 0.08,
        glitchMutateChance = 0.08,
        scrollable = true,
        maxStoredLines = 50_000,
        separatorChar = "─",
        wrapLongWords = true, // if false, a single long word will be hard-cut
    } = {}) {
        this.banner = banner;
        this.glitchyness = glitchyness;
        this.stickyness = stickyness;
        this.fps = fps;
        this.waveStrength = waveStrength;
        this.waveSpeed = waveSpeed;
        this.waveFreq = waveFreq;
        this.glitchMutateChance = glitchMutateChance;

        this.scrollable = scrollable;
        this.maxStoredLines = maxStoredLines;
        this.separatorChar = separatorChar;
        this.wrapLongWords = wrapLongWords;

        this.frameMs = Math.max(1, Math.floor(1000 / this.fps));
        this.reset = "\x1b[0m";
        this.fg256 = (n) => `\x1b[38;5;${n}m`;

        this.GLITCH_CHARS = "▓▒░█#@$%&*+=-_:;!?/~\\|()[]{}<>^";
        this.glitches = new Map(); // key "y,x" -> { orig, ch, ttl }

        /** @type {string[]} All stored, already-wrapped log lines */
        this.logLines = [];

        // precompute mutable positions (non-space)
        this.mutable = [];
        for (let y = 0; y < this.banner.length; y++) {
            const line = this.banner[y];
            for (let x = 0; x < line.length; x++) {
                const ch = line[x];
                if (ch !== " " && ch !== "\t") this.mutable.push([y, x]);
            }
        }

        this.t = 0;
        this.timer = setInterval(() => this._tick(), this.frameMs);
    }

    _tick() {
        this.spawnGlitches();
        this.tickGlitches();
        logUpdate(this.render(this.t++));
    }

    randInt(n) {
        return (Math.random() * n) | 0;
    }
    pick(arr) {
        return arr[this.randInt(arr.length)];
    }

    spawnGlitches() {
        // requested probability gate
        if (this.glitchyness < 1 && this.glitchyness > 0) {
            if (Math.random() > this.glitchyness) return;
        }

        const count = this.glitchyness >= 1 ? Math.floor(this.glitchyness) : 1;

        for (let i = 0; i < count; i++) {
            const [y, x] = this.pick(this.mutable);
            const key = `${y},${x}`;
            const orig = this.banner[y][x];

            const existing = this.glitches.get(key);
            if (existing) {
                existing.ttl = Math.max(existing.ttl, this.stickyness);
                continue;
            }

            let ch = orig;
            for (let tries = 0; tries < 6 && ch === orig; tries++) {
                ch = this.GLITCH_CHARS[this.randInt(this.GLITCH_CHARS.length)];
            }

            this.glitches.set(key, { orig, ch, ttl: this.stickyness });
        }
    }

    tickGlitches() {
        for (const [key, g] of this.glitches) {
            g.ttl -= 1;
            if (g.ttl <= 0) this.glitches.delete(key);
            else if (Math.random() < this.glitchMutateChance) {
                g.ch = this.GLITCH_CHARS[this.randInt(this.GLITCH_CHARS.length)];
            }
        }
    }

    colorFor(x, y, t, isGlitched) {
        const rowPhase = (((y * 1103515245 + 12345) >>> 0) % 1000) / 1000;

        const base = 38;
        const w =
            Math.sin(
                t * this.waveSpeed + x * this.waveFreq + rowPhase * Math.PI * 2
            ) *
            0.5 +
            0.5;
        const c = base + Math.round(w * this.waveStrength);

        if (isGlitched) return 51;
        return Math.max(16, Math.min(231, c));
    }

    /**
     * Wrap a single logical line to width.
     * - word-wraps when possible
     * - preserves existing indentation at the start of the line
     * - if a "word" is longer than width:
     *    - if wrapLongWords: breaks it
     *    - else: hard-cuts the whole line into chunks
     */
    _wrapLine(line, width) {
        if (width <= 1) return [line];

        // preserve leading indentation
        const indentMatch = line.match(/^\s*/);
        const indent = indentMatch ? indentMatch[0] : "";
        const content = line.slice(indent.length);

        // If content is empty, keep it as a blank/indent line
        if (!content) return [indent];

        // If no spaces to wrap on and content is huge, treat as long-word case
        const hasSpaces = /\s/.test(content);

        const hardChunk = (s, w) => {
            const out = [];
            for (let i = 0; i < s.length; i += w) out.push(s.slice(i, i + w));
            return out;
        };

        if (!hasSpaces && content.length > width - indent.length) {
            const chunks = hardChunk(content, Math.max(1, width - indent.length));
            return chunks.map((c) => indent + c);
        }

        const words = content.split(/\s+/).filter(Boolean);
        const lines = [];
        let cur = indent;
        let curLen = indent.length;

        const pushCur = () => {
            lines.push(cur.trimEnd());
            cur = indent;
            curLen = indent.length;
        };

        const maxContent = Math.max(1, width - indent.length);

        for (const word of words) {
            if (word.length > maxContent) {
                // flush current line first
                if (curLen > indent.length) pushCur();

                if (this.wrapLongWords) {
                    // break the long word into pieces
                    const chunks = hardChunk(word, maxContent);
                    for (let i = 0; i < chunks.length; i++) {
                        lines.push(indent + chunks[i]);
                    }
                } else {
                    // hard-cut the entire word as-is (still chunked)
                    const chunks = hardChunk(word, maxContent);
                    for (const c of chunks) lines.push(indent + c);
                }
                continue;
            }

            const sep = curLen > indent.length ? " " : "";
            const addLen = sep.length + word.length;

            if (curLen + addLen <= width) {
                cur += sep + word;
                curLen += addLen;
            } else {
                pushCur();
                cur += word;
                curLen += word.length;
            }
        }

        if (curLen > indent.length) lines.push(cur.trimEnd());
        return lines.length ? lines : [indent];
    }

    /**
     * Public logging API.
     * Stores ALL log lines (wrapped) so they can be reprinted on error.
     * The renderer will decide how many fit on-screen.
     */
    log(...args) {
        const msg = args
            .map((a) => {
                if (typeof a === "string") return a;
                try {
                    return JSON.stringify(a);
                } catch {
                    return String(a);
                }
            })
            .join(" ");

        // Determine current usable width for log wrapping.
        // (If terminal resizes later, render() will still show a tail; stored lines remain wrapped
        // to the width at time of logging, which is usually what you want for “reprint on error”.)
        const cols = Math.min(110, process.stdout.columns || 80);
        const maxLen = Math.max(...this.banner.map((l) => l.length));
        const leftPad = Math.max(0, Math.floor((cols - maxLen) / 2));
        const contentWidth = Math.max(10, cols - leftPad); // keep sane minimum

        for (const rawLine of msg.split(/\r?\n/)) {
            const wrapped = this._wrapLine(rawLine, contentWidth);
            for (const wl of wrapped) this.logLines.push(wl);
        }

        if (this.logLines.length > this.maxStoredLines) {
            this.logLines.splice(0, this.logLines.length - this.maxStoredLines);
        }
    }

    getLogLines() {
        return this.logLines.slice();
    }

    stop() {
        if (this.timer) {
            clearInterval(this.timer);
            this.timer = null;
        }
        logUpdate.clear();
        process.stdout.write(ansiEscapes.cursorShow + this.reset + "\n");
    }

    render(t) {
        const cols = process.stdout.columns || 80;
        const rows = process.stdout.rows || 24;

        const maxLen = Math.max(...this.banner.map((l) => l.length));
        const leftPad = Math.max(0, Math.floor((cols - maxLen) / 2));
        const contentWidth = Math.max(1, cols - leftPad);

        const lines = [];
        lines.push(ansiEscapes.cursorHide);
        lines.push(ansiEscapes.clearScreen);

        // Logo
        for (let y = 0; y < this.banner.length; y++) {
            const line = this.banner[y];
            let out = " ".repeat(leftPad);

            for (let x = 0; x < line.length; x++) {
                const key = `${y},${x}`;
                const g = this.glitches.get(key);
                const ch = g ? g.ch : line[x];
                out += this.fg256(this.colorFor(x, y, t, Boolean(g))) + ch + this.reset;
            }
            lines.push(out);
        }

        // Separator + blank
        lines.push("");
        const sep = this.separatorChar.repeat(Math.min(maxLen, contentWidth));
        lines.push(" ".repeat(leftPad) + "\x1b[2m" + sep + this.reset);

        if (this.scrollable) {
            const used =
                2 + // hide+clear
                this.banner.length +
                1 + // blank
                1; // separator
            const availableRows = Math.max(1, rows - used);

            // Tail that fits
            const start = Math.max(0, this.logLines.length - availableRows);
            const visible = this.logLines.slice(start);

            for (const l of visible) {
                // Since we store wrapped lines, only a final hard-cut is needed if terminal shrunk.
                const trimmed =
                    l.length > contentWidth
                        ? l.slice(0, Math.max(0, contentWidth - 1)) + "…"
                        : l;
                lines.push(" ".repeat(leftPad) + trimmed);
            }

            for (let i = visible.length; i < availableRows; i++) {
                lines.push(" ".repeat(leftPad));
            }
        }

        return lines.join("\n");
    }
}
