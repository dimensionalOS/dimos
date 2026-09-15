// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

import assert from 'node:assert/strict';
import { test } from 'node:test';
import { SpeechPlayer } from './static/speech.js';
import { CollectionPrompts } from './static/collection_prompts.js';

function status(event, saved = 0, discarded = 0, ts = 1, snapshot = false) {
    return {
        last_event: event, episodes_saved: saved, episodes_discarded: discarded,
        ts, snapshot, state: event === 'start' ? 'recording' : 'idle',
    };
}

test('confirmed events speak once; snapshots, init and idle commands stay silent', () => {
    const prompts = new CollectionPrompts();
    assert.equal(prompts.update(status('init')), null);
    assert.equal(prompts.update(status('save', 0, 0, 2)), null);
    assert.equal(prompts.update(status('start', 0, 0, 3)), 'Recording started');
    assert.equal(prompts.update(status('start', 0, 0, 3)), null);
    assert.equal(prompts.update(status('save', 1, 0, 4)), 'Episode saved');
    assert.equal(prompts.update(status('save', 1, 0, 5)), null);
    assert.equal(prompts.update(status('start', 1, 0, 6)), 'Recording started');
    assert.equal(prompts.update(status('discard', 1, 1, 7)), 'Recording canceled');
    assert.equal(prompts.update(status('discard', 1, 1, 8)), null);
    prompts.reset();
    assert.equal(prompts.update(status('discard', 1, 1, 7, true)), null);
    assert.equal(prompts.update(status('start', 1, 1, 9)), 'Recording started');
    assert.equal(prompts.update(status('start', 2, 1, 10)), 'Recording started');
});

test('connecting while recording does not announce the cached start', () => {
    const prompts = new CollectionPrompts();
    assert.equal(prompts.update(status('start', 2, 0, 10, true)), null);
    assert.equal(prompts.update(status('start', 2, 0, 10)), null);
    assert.equal(prompts.update(status('save', 3, 0, 11)), 'Episode saved');
});

class AudioContext {
    state = 'suspended';
    destination = {};
    sources = [];
    resume() { this.state = 'running'; return Promise.resolve(); }
    decodeAudioData(bytes) { return Promise.resolve(bytes); }
    createBufferSource() {
        const source = {
            started: false, stopped: false,
            connect() {}, disconnect() {},
            start() { this.started = true; },
            stop() { this.stopped = true; },
        };
        this.sources.push(source);
        return source;
    }
}

function deferred() {
    let resolve;
    const promise = new Promise(r => { resolve = r; });
    return { promise, resolve };
}

test('preload is silent, repeated speech is cached, new speech interrupts playback', async t => {
    const fetch = t.mock.method(globalThis, 'fetch', async () => new Response(new Uint8Array([1])));
    const errors = [];
    const player = new SpeechPlayer('/teleop/speech', value => errors.push(value), AudioContext);
    t.after(() => player.stop());
    await player.preload(['Recording started', 'Episode saved']);
    assert.equal(player.context, null);
    await player.initialize();
    await player.speak('Recording started');
    const first = player.source;
    await player.speak('Episode saved');
    assert.equal(first.stopped, true);
    assert.equal(player.source.started, true);
    assert.equal(fetch.mock.calls.length, 2);
    assert.deepEqual(errors, [false, false]);
    player.stop();
    assert.equal(player.source, null);
});

test('older HTTP responses cannot speak after a newer event or disconnect', async t => {
    const first = deferred();
    let firstSignal;
    t.mock.method(globalThis, 'fetch', async (_url, options) => {
        if (JSON.parse(options.body).text === 'start') {
            firstSignal = options.signal;
            return first.promise;
        }
        return new Response(new Uint8Array([2]));
    });
    const player = new SpeechPlayer('/teleop/speech', () => {}, AudioContext);
    t.after(() => player.stop());
    await player.initialize();
    const pending = player.speak('start');
    await player.speak('save');
    assert.equal(firstSignal.aborted, true);
    first.resolve(new Response(new Uint8Array([1])));
    await pending;
    assert.equal(player.context.sources.length, 1);
    player.stop();
    assert.equal(player.context.sources[0].stopped, true);
});

test('disconnect invalidates audio even when decoding is already in flight', async t => {
    const decoding = deferred();
    const entered = deferred();
    t.mock.method(globalThis, 'fetch', async () => new Response(new Uint8Array([1])));
    const player = new SpeechPlayer('/teleop/speech', () => {}, AudioContext);
    await player.initialize();
    t.mock.method(player.context, 'decodeAudioData', () => {
        entered.resolve();
        return decoding.promise;
    });
    const pending = player.speak('start');
    await entered.promise;
    player.stop();
    decoding.resolve(new ArrayBuffer(1));
    await pending;
    assert.equal(player.context.sources.length, 0);
});

test('audio failures are reported and a later successful request recovers', async t => {
    t.mock.method(console, 'warn', () => {});
    const fetch = t.mock.method(globalThis, 'fetch', async () => new Response('', { status: 503 }));
    const unavailable = [];
    const player = new SpeechPlayer('/teleop/speech', value => unavailable.push(value), AudioContext);
    t.after(() => player.stop());
    await player.initialize();
    await player.speak('start');
    assert.deepEqual(unavailable, [true]);
    fetch.mock.mockImplementation(async () => new Response(new Uint8Array([1])));
    await player.speak('start');
    assert.deepEqual(unavailable, [true, false]);
});

test('suspended browser audio drops the prompt without fetching or queueing', async t => {
    t.mock.method(console, 'warn', () => {});
    const fetch = t.mock.method(globalThis, 'fetch', async () => new Response(''));
    const unavailable = [];
    const player = new SpeechPlayer('/teleop/speech', value => unavailable.push(value), AudioContext);
    await player.speak('start');
    assert.equal(fetch.mock.calls.length, 0);
    assert.deepEqual(unavailable, [true]);
});
