export const TRACKING_INTERVAL_MS = 1000 / 50;

export function scheduleTrackingFrame(time, nextDeadline) {
    if (nextDeadline === null) {
        return { send: true, nextDeadline: time + TRACKING_INTERVAL_MS };
    }
    if (time < nextDeadline) {
        return { send: false, nextDeadline };
    }

    const elapsedPeriods = Math.floor((time - nextDeadline) / TRACKING_INTERVAL_MS) + 1;
    return {
        send: true,
        nextDeadline: nextDeadline + elapsedPeriods * TRACKING_INTERVAL_MS,
    };
}
