export type Field = [name: string, type: string, dims: (number | string)[] | null]
export type MsgValue = Record<string, unknown>

export type MsgInput<T> = T extends bigint ? bigint | number
    : T extends BigInt64Array ? ArrayLike<bigint | number>
    : T extends Uint8Array | Int8Array | Int16Array | Int32Array | Float32Array | Float64Array ? ArrayLike<number>
    : T extends (infer E)[] ? MsgInput<E>[]
    : T extends object ? { [K in keyof T]?: MsgInput<T[K]> }
    : T

export interface MsgType<T = MsgValue> {
    readonly name: string
    readonly fingerprint: string | null
    decode(bytes: Uint8Array): T
    encode(value: MsgInput<T>): Uint8Array
    zenohKey(topic: string): string
    lcmChannel(topic: string): string
}

export interface KeyedSample {
    key: string
    bytes: Uint8Array
    kind?: string
}

const SIZE: Record<string, number> = {
    int8_t: 1,
    int16_t: 2,
    int32_t: 4,
    int64_t: 8,
    float: 4,
    double: 8,
    boolean: 1,
    byte: 1,
}
const TYPED: Record<string, { new (n: number): ArrayLike<unknown> & { [i: number]: unknown } }> = {
    int16_t: Int16Array,
    int32_t: Int32Array,
    int64_t: BigInt64Array,
    float: Float32Array,
    double: Float64Array,
}
const utf8Decoder = new TextDecoder("utf-8", { fatal: true, ignoreBOM: true })
const utf8Encoder = new TextEncoder()
const TYPE_NAME = /^[A-Za-z_][A-Za-z0-9_]*\.[A-Za-z_][A-Za-z0-9_]*$/

type Reader = { view: DataView; bytes: Uint8Array; off: number }
type FieldDecoder = (r: Reader, out: MsgValue) => void

class Writer {
    bytes = new Uint8Array(256)
    view = new DataView(this.bytes.buffer)
    off = 0
    need(n: number): void {
        if (this.off + n <= this.bytes.length) {
            return
        }
        const grown = new Uint8Array(Math.max(this.bytes.length * 2, this.off + n))
        grown.set(this.bytes)
        this.bytes = grown
        this.view = new DataView(grown.buffer)
    }
}

function readScalar(r: Reader, type: string): unknown {
    const { view } = r
    const at = r.off
    if (type === "string") {
        const len = view.getInt32(at, false)
        if (len < 1 || at + 4 + len > r.bytes.byteLength) {
            throw new Error(`bad string length ${len} at ${at}`)
        }
        r.off = at + 4 + len
        return utf8Decoder.decode(r.bytes.subarray(at + 4, at + 3 + len))
    }
    r.off = at + SIZE[type]
    switch (type) {
        case "double":
            return view.getFloat64(at, false)
        case "float":
            return view.getFloat32(at, false)
        case "int64_t":
            return view.getBigInt64(at, false)
        case "int32_t":
            return view.getInt32(at, false)
        case "int16_t":
            return view.getInt16(at, false)
        case "int8_t":
            return view.getInt8(at)
        case "byte":
            return view.getUint8(at)
        case "boolean":
            return view.getUint8(at) !== 0
    }
    throw new Error(`unknown LCM primitive ${type}`)
}

function writeScalar(w: Writer, type: string, value: unknown): void {
    if (type === "string") {
        const text = utf8Encoder.encode(value === undefined ? "" : String(value))
        w.need(5 + text.length)
        w.view.setInt32(w.off, text.length + 1, false)
        w.bytes.set(text, w.off + 4)
        w.bytes[w.off + 4 + text.length] = 0
        w.off += 5 + text.length
        return
    }
    const at = w.off
    w.need(SIZE[type])
    w.off += SIZE[type]
    const v = w.view
    switch (type) {
        case "double":
            return v.setFloat64(at, Number(value ?? 0), false)
        case "float":
            return v.setFloat32(at, Number(value ?? 0), false)
        case "int64_t":
            return v.setBigInt64(at, typeof value === "bigint" ? value : BigInt(Math.trunc(Number(value ?? 0))), false)
        case "int32_t":
            return v.setInt32(at, Number(value ?? 0), false)
        case "int16_t":
            return v.setInt16(at, Number(value ?? 0), false)
        case "int8_t":
            return v.setInt8(at, Number(value ?? 0))
        case "byte":
            return v.setUint8(at, Number(value ?? 0))
        case "boolean":
            return v.setUint8(at, value ? 1 : 0)
    }
    throw new Error(`unknown LCM primitive ${type}`)
}

class Codec {
    private decoders = new Map<string, FieldDecoder[] | null>()
    constructor(private structs: Record<string, Field[]>) {}

    private rows(name: string): Field[] {
        const rows = this.structs[name]
        if (!Array.isArray(rows)) {
            throw new Error(`no schema for struct ${name}`)
        }
        return rows
    }

    private isPrimitive(type: string): boolean {
        return type === "string" || Object.hasOwn(SIZE, type)
    }

    private count(dim: number | string, value: MsgValue): number {
        const n = typeof dim === "number" ? dim : value[dim]
        if (typeof n !== "number" || n < 0 || !Number.isInteger(n)) {
            throw new Error(`bad array length ${String(n)}`)
        }
        return n
    }

    fields(name: string): FieldDecoder[] {
        const done = this.decoders.get(name)
        if (done) {
            return done
        }
        if (done === null) {
            throw new Error(`recursive struct ${name}`)
        }
        this.decoders.set(name, null)
        const decoders = this.rows(name).map(([field, type, dims]): FieldDecoder => {
            if (field === "__proto__") {
                throw new Error(`field __proto__ is not supported (${name})`)
            }
            const primitive = this.isPrimitive(type)
            const one = primitive ? (r: Reader) => readScalar(r, type) : (r: Reader) => this.readStruct(r, type)
            if (dims === null) {
                return (r, out) => {
                    out[field] = one(r)
                }
            }
            if (dims.length !== 1) {
                throw new Error(`multi-dimensional arrays are not supported (${name}.${field})`)
            }
            const [dim] = dims
            return (r, out) => {
                const n = this.count(dim, out)
                const size = primitive ? SIZE[type] ?? 5 : 1
                if (r.off + n * size > r.bytes.byteLength) {
                    throw new Error(`${name}.${field}: ${n} x ${type} overruns the frame`)
                }
                if (type === "byte" || type === "int8_t") {
                    const view = r.bytes.subarray(r.off, r.off + n)
                    out[field] = type === "byte" ? view : new Int8Array(view.buffer as ArrayBuffer, view.byteOffset, n)
                    r.off += n
                    return
                }
                const Typed = TYPED[type]
                const items = Typed ? new Typed(n) : new Array(n)
                for (let i = 0; i < n; i++) {
                    items[i] = one(r)
                }
                out[field] = items
            }
        })
        this.decoders.set(name, decoders)
        return decoders
    }

    readStruct(r: Reader, name: string): MsgValue {
        const out: MsgValue = {}
        for (const decode of this.fields(name)) {
            decode(r, out)
        }
        return out
    }

    writeStruct(w: Writer, name: string, value: MsgValue): void {
        const rows = this.rows(name)
        const counts: Record<string, number> = {}
        for (const [field, type, dims] of rows) {
            const item = value?.[field]
            const primitive = this.isPrimitive(type)
            const one = (x: unknown) =>
                primitive ? writeScalar(w, type, x) : this.writeStruct(w, type, (x ?? {}) as MsgValue)
            if (dims === null) {
                const counted = rows.filter(([, , d]) => d?.[0] === field).map(([f]) => value?.[f])
                    .filter((a) => a !== undefined) as ArrayLike<unknown>[]
                if (counted.length > 0) {
                    const n = counted[0].length
                    if (counted.some((a) => a.length !== n)) {
                        throw new Error(`${name}: the arrays counted by ${field} differ in length`)
                    }
                    counts[field] = n
                    writeScalar(w, type, n)
                } else {
                    counts[field] = Number(item ?? 0)
                    one(item)
                }
                continue
            }
            if (dims.length !== 1) {
                throw new Error(`multi-dimensional arrays are not supported (${name}.${field})`)
            }
            const [dim] = dims
            const n = typeof dim === "number" ? dim : counts[dim]
            const items = (item ?? []) as ArrayLike<unknown>
            if (item !== undefined && items.length !== n) {
                throw new Error(`${name}.${field}: ${items.length} items, the schema wants ${n}`)
            }
            for (let i = 0; i < n; i++) {
                one(items[i])
            }
        }
    }
}

function fingerprintBytes(hex: string): Uint8Array {
    return Uint8Array.from({ length: 8 }, (_, i) => parseInt(hex.slice(i * 2, i * 2 + 2), 16))
}

function fingerprintOf(bytes: Uint8Array): string {
    if (bytes.byteLength < 8) {
        throw new Error("frame shorter than its 8-byte fingerprint")
    }
    return Array.from(bytes.subarray(0, 8), (b) => b.toString(16).padStart(2, "0")).join("")
}

function withName<T>(name: string, fingerprint: string | null, decode: (bytes: Uint8Array) => T, encode: (value: MsgInput<T>) => Uint8Array): MsgType<T> {
    return Object.freeze({
        name,
        fingerprint,
        decode,
        encode,
        zenohKey: (topic: string) => `${topic.replace(/\/+$/, "")}/${name}`,
        lcmChannel: (topic: string) => `${topic}#${name}`,
    })
}

export function createRegistry(structs: Record<string, Field[]>, types: Record<string, string>, missing: Record<string, string>) {
    const codec = new Codec(structs)
    const byName = new Map<string, MsgType<unknown>>()
    const byFingerprint = new Map<string, MsgType<unknown>[]>()
    const unsupported = new Map(Object.entries(missing))

    const add = (type: MsgType<unknown>) => {
        byName.set(type.name, type)
        unsupported.delete(type.name)
        if (type.fingerprint !== null) {
            const list = (byFingerprint.get(type.fingerprint) ?? []).filter((t) => t.name !== type.name)
            byFingerprint.set(type.fingerprint, [type, ...list])
        }
        return type
    }

    for (const [name, fingerprint] of Object.entries(types)) {
        const expected = fingerprintBytes(fingerprint)
        add(withName<MsgValue>(
            name,
            fingerprint,
            (bytes) => {
                for (let i = 0; i < 8; i++) {
                    if (bytes[i] !== expected[i]) {
                        throw new Error(`fingerprint mismatch: the frame is not a ${name}`)
                    }
                }
                const r = { view: new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength), bytes, off: 8 }
                const value = codec.readStruct(r, name)
                if (r.off !== bytes.byteLength) {
                    throw new Error(`${name}: ${bytes.byteLength - r.off} trailing bytes`)
                }
                return value
            },
            (value) => {
                const w = new Writer()
                w.need(8)
                w.bytes.set(expected)
                w.off = 8
                codec.writeStruct(w, name, value as MsgValue)
                return w.bytes.slice(0, w.off)
            },
        ))
    }

    const typeOfChannel = (channel: string): string | undefined => {
        const hash = channel.lastIndexOf("#")
        const name = hash >= 0 ? channel.slice(hash + 1) : channel.slice(channel.lastIndexOf("/") + 1)
        return TYPE_NAME.test(name) ? name : undefined
    }

    const lookup = (name: string): MsgType<unknown> => {
        const type = byName.get(name)
        if (type) {
            return type
        }
        const reason = unsupported.get(name)
        throw new Error(reason ? `${name} has no LCM schema (${reason}): register() a decoder for it` : `unknown message type ${name}`)
    }

    const decode = (bytes: Uint8Array): unknown => {
        const fingerprint = fingerprintOf(bytes)
        const type = byFingerprint.get(fingerprint)?.[0]
        if (!type) {
            throw new Error(`no message type has fingerprint ${fingerprint}`)
        }
        return type.decode(bytes)
    }

    const decodeChannel = (channel: string, bytes: Uint8Array): unknown => {
        const name = typeOfChannel(channel)
        if (name !== undefined && (byName.has(name) || unsupported.has(name))) {
            return lookup(name).decode(bytes)
        }
        return decode(bytes)
    }

    const decodeMessage = (message: KeyedSample): unknown =>
        message.kind === "delete" ? undefined : decodeChannel(message.key, message.bytes)

    const register = <T = unknown>(
        name: string,
        fingerprint: string | null,
        decodeFn: (bytes: Uint8Array) => T,
        encodeFn?: (value: MsgInput<T>) => Uint8Array,
    ): MsgType<T> => {
        if (!TYPE_NAME.test(name)) {
            throw new Error(`register: the name must be "<package>.<Type>", got ${JSON.stringify(name)}`)
        }
        if (fingerprint !== null && !/^[0-9a-f]{16}$/.test(fingerprint)) {
            throw new Error(`register: the fingerprint must be 16 lowercase hex chars or null, got ${JSON.stringify(fingerprint)}`)
        }
        const encode = encodeFn ?? (() => {
            throw new Error(`${name} was registered without an encoder`)
        })
        return add(withName(name, fingerprint, decodeFn, encode) as MsgType<unknown>) as MsgType<T>
    }

    return {
        decode,
        decodeChannel,
        decodeMessage,
        typeOfChannel,
        lookup,
        register,
        getTypeNames: (): string[] => [...byName.keys()].sort(),
        getMissingTypes: (): Record<string, string> => Object.fromEntries(unsupported),
    }
}
