// Copyright 2026 Dimensional Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Tests the browser client against a fake WebSocket — no network, no server.
//   deno test --allow-read dimos/web/ts_bridge/client/dimos.test.js

import { assertEquals, assertExists } from "jsr:@std/assert@1"
import { Dimos } from "./dimos.js"

// A drop-in WebSocket that scripts the bridge's side of the protocol:
// dispatches "open", answers "hello" with "ready", and "rpc" with "rpc_result".
class FakeWebSocket {
    static lastInstance
    static ready = { config: {}, modules: {} }

    constructor(url) {
        this.url = url
        this.binaryType = "blob"
        this.sent = []
        this._listeners = { open: [], message: [], error: [], close: [] }
        FakeWebSocket.lastInstance = this
        queueMicrotask(() => this._dispatch("open", {}))
    }

    addEventListener(type, handler, options = {}) {
        this._listeners[type].push({ handler, once: !!options.once })
    }

    removeEventListener(type, handler) {
        this._listeners[type] = this._listeners[type].filter((entry) => entry.handler !== handler)
    }

    _dispatch(type, event) {
        const listeners = this._listeners[type]
        this._listeners[type] = listeners.filter((entry) => !entry.once)
        for (const entry of listeners) {
            entry.handler(event)
        }
    }

    send(raw) {
        const message = JSON.parse(raw)
        this.sent.push(message)
        if (message.type === "hello") {
            this.emitJson({ type: "ready", ...FakeWebSocket.ready })
        } else if (message.type === "rpc") {
            this.emitJson({
                type: "rpc_result",
                id: message.id,
                result: [message.module, message.method, message.args],
            })
        }
    }

    emitJson(object) {
        queueMicrotask(() => this._dispatch("message", { data: JSON.stringify(object) }))
    }

    emitBinary(buffer) {
        this._dispatch("message", { data: buffer })
    }

    close() {
        this.closed = true
    }
}

function frame(stream, payload) {
    const name = new TextEncoder().encode(stream)
    const buffer = new Uint8Array(2 + name.length + payload.length)
    new DataView(buffer.buffer).setUint16(0, name.length, false)
    buffer.set(name, 2)
    buffer.set(payload, 2 + name.length)
    return buffer.buffer
}

async function connect(options = {}) {
    FakeWebSocket.ready = { config: options.config ?? {}, modules: options.modules ?? {} }
    globalThis.WebSocket = FakeWebSocket
    const app = await Dimos.connect({
        dimosWs: { host: "localhost", port: 1234, ...(options.dimosWs ?? {}) },
        decode: options.decode,
    })
    return { app, socket: FakeWebSocket.lastInstance }
}

Deno.test("connect sends a hello and resolves once the bridge is ready", async () => {
    const { app, socket } = await connect({
        config: { listen_host: "0.0.0.0" },
        modules: { GO2Connection: ["standup"] },
        dimosWs: { whitelist: ["/tf"], qos: { "/tf": { rate: 5 } } },
    })
    const hello = socket.sent[0]
    assertEquals(hello.type, "hello")
    assertEquals(hello.whitelist, ["/tf"])
    assertEquals(hello.qos, { "/tf": { rate: 5 } })
    assertEquals(app.config, { listen_host: "0.0.0.0" })
    assertExists(app.modules.GO2Connection.standup)
    app.close()
})

Deno.test("modules proxy round-trips an RPC call to a result", async () => {
    const { app } = await connect({ modules: { Localization: ["get_pose"] } })
    const result = await app.modules.Localization.get_pose(1, 2)
    assertEquals(result, ["Localization", "get_pose", [1, 2]])
    app.close()
})

Deno.test("subscribe receives decoded frames and unsubscribe stops them", async () => {
    const decode = (payload) => ({ bytes: [...payload] })
    const { app, socket } = await connect({ decode })
    const seen = []
    const unsub = app.subscribe("/tf", (message) => seen.push(message))

    socket.emitBinary(frame("/tf", new Uint8Array([7, 8])))
    assertEquals(seen.length, 1)
    assertEquals(seen[0].stream, "/tf")
    assertEquals(seen[0].data, { bytes: [7, 8] })
    assertEquals(typeof seen[0].ts, "number")

    unsub()
    socket.emitBinary(frame("/tf", new Uint8Array([9])))
    assertEquals(seen.length, 1) // unsubscribed: no further delivery
    app.close()
})

Deno.test("without a decoder, subscribers get the raw payload bytes", async () => {
    const { app, socket } = await connect()
    let data
    app.subscribe("/tf", (message) => (data = message.data))
    socket.emitBinary(frame("/tf", new Uint8Array([1, 2, 3])))
    assertEquals(data instanceof Uint8Array, true)
    assertEquals([...data], [1, 2, 3])
    app.close()
})

Deno.test("subscribeAll sees every stream by name", async () => {
    const { app, socket } = await connect()
    const streams = []
    app.subscribeAll((message) => streams.push(message.stream))
    socket.emitBinary(frame("/tf", new Uint8Array([1])))
    socket.emitBinary(frame("/odom", new Uint8Array([2])))
    assertEquals(streams, ["/tf", "/odom"])
    app.close()
})

Deno.test("peek resolves with the next message's data", async () => {
    const { app, socket } = await connect({ decode: (payload) => [...payload] })
    const pending = app.peek("/tf", { timeoutMs: 1000 })
    socket.emitBinary(frame("/tf", new Uint8Array([42])))
    assertEquals(await pending, [42])
    app.close()
})

Deno.test("peek resolves null after the timeout when nothing arrives", async () => {
    const { app } = await connect()
    assertEquals(await app.peek("/tf", { timeoutMs: 5 }), null)
    app.close()
})

Deno.test("setQos sends a set_qos control message", async () => {
    const { app, socket } = await connect()
    app.setQos("/tf", { rate: 30, durability: "transient_local" })
    const last = socket.sent[socket.sent.length - 1]
    assertEquals(last, {
        type: "set_qos",
        qos: { "/tf": { rate: 30, durability: "transient_local" } },
    })
    app.close()
})
