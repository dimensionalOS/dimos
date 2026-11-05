// The view's timings, each named once with why.

/** a starting or stopping launch's step progress lives in its output, which no event announces */
export const BUSY_LAUNCH_POLL_MS = 1000;
/** without zenoh (the gateway's own port, or the link down) no launch event comes, so GET active this often */
export const LAUNCH_POLL_WITHOUT_EVENTS_MS = 3000;
/** Desktop's topic rates have no event; every 2 s keeps the Hz column fresh without flooding Desktop */
export const RATES_POLL_MS = 2000;
/** dtop's process numbers have no event; every 2 s matches the rates beside them */
export const STATS_POLL_MS = 2000;
/** the graph waits for the fonts to measure its labels, but a font that never loads mustn't hold it longer */
export const FONT_WAIT_MS = 1500;
/** a run's log has no event, so the next lines are asked for this often */
export const LOG_POLL_MS = 1500;
/** a log search waits this long after the last keystroke before asking */
export const LOG_SEARCH_DEBOUNCE_MS = 250;
