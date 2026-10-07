// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

import assert from 'node:assert/strict';
import { test } from 'node:test';
import { SpeechPlayer } from './static/speech.js';

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


test('pushed audio plays without HTTP and newer audio interrupts playback', async t => {
    const fetch = t.mock.method(globalThis, 'fetch', () => { throw new Error('Unexpected HTTP'); });
    const errors = [];
    const player = new SpeechPlayer(value => errors.push(value), AudioContext);
    t.after(() => player.stop());
    await player.initialize();
    await player.play('AQ==');
    const first = player.source;
    assert.deepEqual(new Uint8Array(first.buffer), new Uint8Array([1]));
    await player.play('Ag==');
    assert.equal(first.stopped, true);
    assert.equal(player.source.started, true);
    assert.equal(fetch.mock.calls.length, 0);
    assert.deepEqual(errors, [false, false]);
});

test('newer audio invalidates older asynchronous decoding', async t => {
    const old = deferred();
    const entered = deferred();
    const player = new SpeechPlayer(() => {}, AudioContext);
    t.after(() => player.stop());
    await player.initialize();
    t.mock.method(player.context, 'decodeAudioData', bytes => {
        if (new Uint8Array(bytes)[0] === 1) { entered.resolve(); return old.promise; }
        return Promise.resolve(bytes);
    });
    const pending = player.play('AQ==');
    await entered.promise;
    await player.play('Ag==');
    old.resolve(new ArrayBuffer(1));
    await pending;
    assert.equal(player.context.sources.length, 1);
    assert.deepEqual(new Uint8Array(player.source.buffer), new Uint8Array([2]));
});

test('disconnect stops playback and invalidates pending decoding', async t => {
    const player = new SpeechPlayer(() => {}, AudioContext);
    t.after(() => player.stop());
    await player.initialize();
    await player.play('AQ==');
    const source = player.source;
    player.stop();
    assert.equal(source.stopped, true);
    const decoding = deferred();
    t.mock.method(player.context, 'decodeAudioData', () => decoding.promise);
    const pending = player.play('Ag==');
    player.stop();
    decoding.resolve(new ArrayBuffer(1));
    await pending;
    assert.equal(player.context.sources.length, 1);
    assert.equal(player.source, null);
});

test('bad audio reports failure and a later prompt recovers', async t => {
    t.mock.method(console, 'warn', () => {});
    const unavailable = [];
    const player = new SpeechPlayer(value => unavailable.push(value), AudioContext);
    t.after(() => player.stop());
    await player.initialize();
    await player.play('not valid base64!');
    assert.deepEqual(unavailable, [true]);
    await player.play('AQ==');
    assert.deepEqual(unavailable, [true, false]);
});

test('suspended audio drops the prompt without queueing', async t => {
    t.mock.method(console, 'warn', () => {});
    const unavailable = [];
    const player = new SpeechPlayer(value => unavailable.push(value), AudioContext);
    await player.play('AQ==');
    assert.equal(player.context, null);
    assert.deepEqual(unavailable, [true]);
});
