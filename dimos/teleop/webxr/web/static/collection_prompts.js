// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

export const RECORDING_PROMPTS = {
    start: 'Recording started',
    save: 'Episode saved',
    discard: 'Recording canceled',
};

export class CollectionPrompts {
    constructor() {
        this.reset();
    }

    reset() {
        this.previous = null;
        this.signature = null;
    }

    update(status) {
        const signature = JSON.stringify([
            status.ts, status.state, status.last_event,
            status.episodes_saved, status.episodes_discarded,
        ]);
        if (signature === this.signature) return null;
        const previous = this.previous;
        this.previous = status;
        this.signature = signature;
        if (status.snapshot || status.last_event === 'init') return null;
        if (status.last_event === 'start' && status.state === 'recording') {
            return RECORDING_PROMPTS.start;
        }
        if (status.last_event === 'save' && previous &&
            status.episodes_saved > previous.episodes_saved) return RECORDING_PROMPTS.save;
        if (status.last_event === 'discard' && previous &&
            status.episodes_discarded > previous.episodes_discarded) return RECORDING_PROMPTS.discard;
        return null;
    }
}
