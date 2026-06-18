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

// Browser/Deno/Node client for the dimos ts_bridge. No build step: native ESM,
// native WebSocket and fetch. Types live in dimos.d.ts.
//
// Pass `decode` (the `decode` export from @dimos/msgs) to receive messages as
// raw LCM binary frames decoded client-side — no JSON. Without it the bridge
// falls back to JSON.

/**
 * @typedef {object} QosProfile
 * @property {"best_effort"|"reliable"} [reliability]
 * @property {"volatile"|"transient_local"} [durability]
 * @property {number} [depth]
 * @property {number} [rate] - max messages/sec
 *
 * @typedef {object} DimosWsConfig
 * @property {string} host
 * @property {number} port
 * @property {string} [wsPath]
 * @property {string[]} [whitelist] - only one of whitelist / blacklist may be set
 * @property {string[]} [blacklist]
 * @property {Record<string, number>} [rateLimit] - stream (or glob) -> max msgs/sec
 * @property {Record<string, QosProfile>} [qos] - stream (or glob) -> QoS profile
 *
 * @typedef {object} StreamMessage
 * @property {string} stream
 * @property {unknown} data
 * @property {number} ts
 *
 * @typedef {(message: StreamMessage) => void} StreamCallback
 * @typedef {(payload: Uint8Array) => unknown} DecodeFn
 */

let nextRequestId = 1

export class Dimos {
    /** @type {Record<string, unknown>} JSON-able snapshot of the coordinator's GlobalConfig. */
    config
    /** @type {Record<string, Record<string, (...args: unknown[]) => Promise<unknown>>>} RPC proxy. */
    modules

    #socket
    #decode
    #subscribers = new Map()
    #allSubscribers = new Set()
    #queues = new Set()
    #pending = new Map()

    /**
     * @param {WebSocket} socket
     * @param {Record<string, unknown>} config
     * @param {Record<string, string[]>} moduleNames
     * @param {DecodeFn | undefined} decode
     */
    constructor(socket, config, moduleNames, decode) {
        this.#socket = socket
        this.#decode = decode
        this.config = config
        this.modules = this.#buildModules(moduleNames)
        socket.addEventListener("message", (event) => this.#onMessage(event))
    }

    /**
     * Attach to a running dimos websocket bridge.
     * @param {{ dimosWs: DimosWsConfig, decode?: DecodeFn }} options
     * @returns {Promise<Dimos>}
     */
    static async connect(options) {
        const { dimosWs, decode } = options
        const wsPath = dimosWs.wsPath ?? "/ws"
        const url = `ws://${dimosWs.host}:${dimosWs.port}${wsPath}`
        const socket = new WebSocket(url)
        socket.binaryType = "arraybuffer"

        await new Promise((resolve, reject) => {
            socket.addEventListener("open", () => resolve(), { once: true })
            socket.addEventListener("error", () => reject(new Error(`ts_bridge connect failed: ${url}`)), {
                once: true,
            })
        })

        const ready = new Promise((resolve, reject) => {
            socket.addEventListener(
                "message",
                (event) => {
                    const message = JSON.parse(event.data)
                    if (message.type === "ready") {
                        resolve(message)
                    } else {
                        reject(new Error(`expected ready, got ${message.type}`))
                    }
                },
                { once: true },
            )
        })

        socket.send(
            JSON.stringify({
                type: "hello",
                whitelist: dimosWs.whitelist ?? [],
                blacklist: dimosWs.blacklist ?? [],
                rateLimit: dimosWs.rateLimit ?? {},
                qos: dimosWs.qos ?? {},
            }),
        )

        const info = await ready
        return new Dimos(socket, info.config, info.modules, decode)
    }

    /**
     * Resolve with the next message on a stream, or null after timeoutMs.
     * (Only streams your subscription/QoS allow; transient_local makes it instant.)
     * @param {string} stream
     * @param {{ timeoutMs?: number }} [options]
     * @returns {Promise<unknown>}
     */
    peek(stream, options = {}) {
        return new Promise((resolve) => {
            const unsub = this.subscribe(stream, (message) => {
                clearTimeout(timer)
                unsub()
                resolve(message.data)
            })
            const timer = setTimeout(() => {
                unsub()
                resolve(null)
            }, options.timeoutMs ?? 1000)
        })
    }

    /**
     * Subscribe with a callback; returns an unsubscribe handle.
     * @param {string} stream
     * @param {StreamCallback} callback
     * @returns {() => void}
     */
    subscribe(stream, callback) {
        let set = this.#subscribers.get(stream)
        if (!set) {
            set = new Set()
            this.#subscribers.set(stream, set)
        }
        set.add(callback)
        return () => set.delete(callback)
    }

    /**
     * Subscribe to every incoming message, including streams not known ahead of
     * time. The callback's `message.stream` says which stream it came from.
     * (You still only receive streams your subscription/QoS allow — connect with
     * no whitelist to see everything the bridge exposes.)
     * @param {StreamCallback} callback
     * @returns {() => void}
     */
    subscribeAll(callback) {
        this.#allSubscribers.add(callback)
        return () => this.#allSubscribers.delete(callback)
    }

    /**
     * Set/replace QoS for a stream (or glob) at runtime.
     * @param {string} stream
     * @param {QosProfile} profile
     */
    setQos(stream, profile) {
        this.#socket.send(JSON.stringify({ type: "set_qos", qos: { [stream]: profile } }))
    }

    /**
     * Async iteration over a stream. Back-pressure-aware, LATEST-coalesced.
     * @param {string} stream
     * @returns {AsyncGenerator<StreamMessage>}
     */
    async *stream(stream) {
        let latest
        let wake
        const entry = {
            stream,
            push: (message) => {
                latest = message
                wake?.()
            },
        }
        this.#queues.add(entry)
        try {
            while (true) {
                if (latest === undefined) {
                    await new Promise((resolve) => (wake = resolve))
                }
                const message = latest
                latest = undefined
                yield message
            }
        } finally {
            this.#queues.delete(entry)
        }
    }

    close() {
        this.#socket.close()
    }

    #buildModules(moduleNames) {
        const modules = {}
        for (const [moduleName, methods] of Object.entries(moduleNames)) {
            const methodMap = {}
            for (const method of methods) {
                methodMap[method] = (...args) =>
                    this.#request({ type: "rpc", module: moduleName, method, args })
            }
            modules[moduleName] = methodMap
        }
        return modules
    }

    #request(payload) {
        const id = nextRequestId++
        return new Promise((resolve, reject) => {
            this.#pending.set(id, { resolve, reject })
            this.#socket.send(JSON.stringify({ ...payload, id }))
        })
    }

    #dispatch(message) {
        this.#subscribers.get(message.stream)?.forEach((callback) => callback(message))
        this.#allSubscribers.forEach((callback) => callback(message))
        for (const entry of this.#queues) {
            if (entry.stream === message.stream) entry.push(message)
        }
    }

    #onMessage(event) {
        // Data plane: a binary frame — uint16 name length, name, encoded payload.
        // The bridge forwards the bus's wire bytes; we decode with the matching codec.
        if (event.data instanceof ArrayBuffer) {
            const view = new DataView(event.data)
            const nameLen = view.getUint16(0, false)
            const stream = new TextDecoder().decode(new Uint8Array(event.data, 2, nameLen))
            const payload = new Uint8Array(event.data, 2 + nameLen)
            const data = this.#decode ? this.#decode(payload) : payload
            this.#dispatch({ stream, data, ts: Date.now() / 1000 })
            return
        }

        // Control plane: JSON (ready handled in connect, rpc_result here).
        const message = JSON.parse(event.data)
        if (message.type === "rpc_result") {
            const pending = this.#pending.get(message.id)
            this.#pending.delete(message.id)
            if (message.error) {
                pending?.reject(new Error(message.error))
            } else {
                pending?.resolve(message.result)
            }
        }
    }
}
