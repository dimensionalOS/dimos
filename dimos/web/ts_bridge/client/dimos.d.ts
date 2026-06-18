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

export interface QosProfile {
    reliability?: "best_effort" | "reliable"
    durability?: "volatile" | "transient_local"
    /** History / outbox buffer depth. */
    depth?: number
    /** Max messages per second. */
    rate?: number
}

export interface DimosWsConfig {
    host: string
    port: number
    wsPath?: string
    /** Only one of whitelist / blacklist may be set. */
    whitelist?: string[]
    blacklist?: string[]
    /** Stream name (or glob) -> max messages per second (sugar for qos.rate). */
    rateLimit?: Record<string, number>
    /** Stream name (or glob) -> QoS profile. */
    qos?: Record<string, QosProfile>
}

/** Decodes a raw LCM payload to a typed message (the `decode` export of @dimos/msgs). */
export type DecodeFn = (payload: Uint8Array) => unknown

export interface ConnectOptions {
    dimosWs: DimosWsConfig
    /** Decode binary frames (e.g. the `decode` export of @dimos/msgs). Omit to receive raw bytes. */
    decode?: DecodeFn
}

export interface StreamMessage {
    stream: string
    data: unknown
    ts: number
}

export type StreamCallback = (message: StreamMessage) => void

export type ModuleProxy = Record<string, Record<string, (...args: unknown[]) => Promise<unknown>>>

export class Dimos {
    readonly config: Record<string, unknown>
    readonly modules: ModuleProxy
    static connect(options: ConnectOptions): Promise<Dimos>
    peek(stream: string, options?: { timeoutMs?: number }): Promise<unknown>
    subscribe(stream: string, callback: StreamCallback): () => void
    /** Subscribe to every incoming message, including streams not known ahead of time. */
    subscribeAll(callback: StreamCallback): () => void
    /** Set/replace QoS for a stream (or glob) at runtime. */
    setQos(stream: string, profile: QosProfile): void
    stream(stream: string): AsyncGenerator<StreamMessage>
    close(): void
}
