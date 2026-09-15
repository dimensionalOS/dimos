// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

// Local speech service client. Playback uses browser audio APIs, independently
// of the XR device and the synthesis engine on the server.
export class SpeechPlayer {
    constructor(url, onUnavailable, AudioContextClass = globalThis.AudioContext) {
        this.url = url;
        this.onUnavailable = onUnavailable;
        this.AudioContextClass = AudioContextClass;
        this.context = null;
        this.source = null;
        this.requests = new Set();
        this.cache = new Map();
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

    async load(text) {
        if (this.cache.has(text)) {
            const audio = this.cache.get(text);
            this.cache.delete(text);
            this.cache.set(text, audio);
            return audio;
        }
        const controller = new AbortController();
        this.requests.add(controller);
        try {
            const response = await fetch(this.url, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ text }),
                signal: controller.signal,
            });
            if (!response.ok) throw new Error(`Speech request failed (${response.status})`);
            const audio = await response.arrayBuffer();
            this.cache.set(text, audio);
            if (this.cache.size > 128) this.cache.delete(this.cache.keys().next().value);
            return audio;
        } finally {
            this.requests.delete(controller);
        }
    }

    async preload(texts) {
        try {
            await Promise.all(texts.map(text => this.load(text)));
        } catch (error) {
            this.failed(error);
        }
    }

    async speak(text) {
        this.stop();
        const generation = this.generation;
        try {
            if (!this.context || this.context.state !== 'running') {
                throw new Error('Browser audio is not running');
            }
            const bytes = await this.load(text);
            if (generation !== this.generation) return;
            // decodeAudioData can detach its input; retain a reusable cache copy.
            const buffer = await this.context.decodeAudioData(bytes.slice(0));
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
        for (const request of this.requests) request.abort();
        this.requests.clear();
        if (this.source) {
            this.source.stop();
            this.source.disconnect();
            this.source = null;
        }
    }
}
