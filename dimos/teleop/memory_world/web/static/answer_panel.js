// The answer drawn onto a canvas, for the one place HTML cannot go: an immersive XR
// session, where the DOM is not composited into the scene. Everywhere else the same
// text goes onto the page as real, selectable, unblurred HTML -- see `onAnswerText`.

/** Paint `text` into `canvas`, wrapped to the panel's four lines. */
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
    const words = String(text).split(/\s+/);
    const lines = [];
    let line = '';
    let dropped = false;
    for (const word of words) {
        const candidate = line ? `${line} ${word}` : word;
        if (ctx.measureText(candidate).width > 930 && line) {
            lines.push(line);
            if (lines.length === MAX_LINES) { dropped = true; break; }
            line = word;
        } else {
            line = candidate;
        }
    }
    if (!dropped && line) lines.push(line);
    if (dropped) {
        // Say so. Give the ellipsis room by shedding whole words, and stop at the
        // empty string rather than looping on a single word too long to shrink.
        let last = lines[MAX_LINES - 1];
        while (last && ctx.measureText(`${last} ...`).width > 930) {
            last = last.replace(/\s*\S+$/, '');
        }
        lines[MAX_LINES - 1] = `${last} ...`;
    }
    lines.forEach((lineText, i) => ctx.fillText(lineText, 42, 62 + i * 50));
}
