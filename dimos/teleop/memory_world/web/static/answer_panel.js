// The answer drawn onto a canvas, for the one place HTML cannot go: an immersive XR
// session, where the DOM is not composited into the scene. Everywhere else the same
// text goes onto the page as real, selectable, unblurred HTML -- see `onAnswerText`.

/** Paint `text` into `canvas`, wrapped to the panel's four lines. */
const LINE_WIDTH = 930;

/** A token too wide for one line, cut into pieces that each fit. */
function breakLong(ctx, word) {
    const pieces = [];
    let piece = '';
    for (const ch of word) {
        if (piece && ctx.measureText(piece + ch).width > LINE_WIDTH) {
            pieces.push(piece);
            piece = ch;
        } else {
            piece += ch;
        }
    }
    // A single character wider than the whole line: keep it rather than loop forever.
    if (piece) pieces.push(piece);
    return pieces;
}

export function drawAnswer(canvas, text) {
    const ctx = canvas.getContext('2d');
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.fillStyle = 'rgba(5, 10, 16, 0.92)';
    ctx.fillRect(0, 0, canvas.width, canvas.height);
    ctx.strokeStyle = '#7af0a8';
    ctx.lineWidth = 8;
    ctx.strokeRect(4, 4, canvas.width - 8, canvas.height - 8);
    ctx.fillStyle = '#d8e6f4';
    ctx.font = '42px monospace';
    // The panel holds four lines. This used to stop taking words the moment the THIRD
    // was complete, so the fourth was drawn holding the single word that had just
    // started it and everything after that vanished with nothing to say it had -- an
    // answer of any length looked like a finished sentence three and a bit lines long.
    const MAX_LINES = 4;
    // One token wider than the panel -- a URL, a hash, an unspaced identifier out of an
    // analysis answer -- has no space to break at, and the wrap test below only fires
    // when `line` is already non-empty, so the first word of a line was accepted however
    // wide it was and drawn straight off the edge of the canvas with nothing to say so.
    // Break such a token by characters first; every later step then works on words that
    // fit.
    const words = String(text)
        .split(/\s+/)
        .flatMap((word) => (ctx.measureText(word).width <= LINE_WIDTH ? [word] : breakLong(ctx, word)));
    const lines = [];
    let line = '';
    let dropped = false;
    for (const word of words) {
        const candidate = line ? `${line} ${word}` : word;
        if (ctx.measureText(candidate).width > LINE_WIDTH && line) {
            lines.push(line);
            if (lines.length === MAX_LINES) { dropped = true; break; }
            line = word;
        } else {
            line = candidate;
        }
    }
    if (!dropped && line) lines.push(line);
    if (dropped) {
        // Say so, and make room for it by shedding CHARACTERS, not whole words. Shedding
        // words emptied the line outright whenever it held one long token -- which
        // `breakLong` above guarantees for exactly the text that needs an ellipsis most:
        // 'z' x 400 drew lines of [93, 93, 93, 4] characters, the fourth being literally
        // " ...", with 89 characters' worth of room going spare on it.
        let last = lines[MAX_LINES - 1];
        while (last && ctx.measureText(`${last} ...`).width > LINE_WIDTH) {
            // By CODE POINT: `slice(0, -1)` cuts UTF-16 units, so trimming an answer
            // ending in an astral character left a lone surrogate, drawn as tofu.
            // `breakLong` above iterates `for (const ch of word)` for the same reason.
            last = [...last].slice(0, -1).join('');
        }
        lines[MAX_LINES - 1] = `${last} ...`;
    }
    lines.forEach((lineText, i) => ctx.fillText(lineText, 42, 62 + i * 50));
}
