// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

// Play WAV audio pushed by the server using standard browser audio APIs.
export class SpeechPlayer {
    constructor(onUnavailable, AudioContextClass = globalThis.AudioContext) {
        this.onUnavailable = onUnavailable;
        this.AudioContextClass = AudioContextClass;
        this.context = null;
        this.source = null;
        this.generation = 0;
    }

    // Invoke directly in the Connect click, before awaiting other work.
    initialize() {
        try {
            this.context ??= new this.AudioContextClass();
            return this.context.resume().catch(error => this.failed(error));
        } catch (error) {
            this.failed(error);
            return Promise.resolve();
        }
    }

    failed(error) {
        if (error.name === 'AbortError') return;
        console.warn('Speech audio unavailable', error);
        this.onUnavailable(true);
    }

    async play(audio) {
        this.stop();
        const generation = this.generation;
        try {
            if (!this.context || this.context.state !== 'running') {
                throw new Error('Browser audio is not running');
            }
            const bytes = Uint8Array.from(atob(audio), char => char.charCodeAt(0));
            const buffer = await this.context.decodeAudioData(bytes.buffer);
            if (generation !== this.generation) return;
            const source = this.context.createBufferSource();
            source.buffer = buffer;
            source.connect(this.context.destination);
            source.onended = () => {
                source.disconnect();
                if (this.source === source) this.source = null;
            };
            this.source = source;
            source.start();
            this.onUnavailable(false);
        } catch (error) {
            if (generation === this.generation) this.failed(error);
        }
    }

    stop() {
        this.generation += 1;
        if (this.source) {
            this.source.stop();
            this.source.disconnect();
            this.source = null;
        }
    }
}
