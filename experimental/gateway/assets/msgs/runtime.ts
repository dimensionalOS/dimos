import { compileLcmDecoder, type LcmField as Field } from "lcm-decoder"

type MsgValue = Record<string, unknown>
export interface MsgType<T = MsgValue> {
    readonly name: string
    readonly fingerprint: string | null
    decode(bytes: Uint8Array): T
    encode(value: unknown): Uint8Array
    zenohKey(topic: string): string
    lcmChannel(topic: string): string
}

const TYPE_NAME = /^[A-Za-z_][A-Za-z0-9_]*\.[A-Za-z_][A-Za-z0-9_]*$/
const utf8Encoder = new TextEncoder()
const SIZE: Record<string, number> = { int8_t: 1, int16_t: 2, int32_t: 4, int64_t: 8, float: 4, double: 8, boolean: 1, byte: 1 }
const SET: Record<string, (view: DataView, at: number, value: unknown) => void> = {
    double: (view, at, value) => view.setFloat64(at, Number(value ?? 0)),
    float: (view, at, value) => view.setFloat32(at, Number(value ?? 0)),
    int64_t: (view, at, value) =>
        view.setBigInt64(at, typeof value === "bigint" ? value : BigInt(Math.trunc(Number(value ?? 0)))),
    int32_t: (view, at, value) => view.setInt32(at, Number(value ?? 0)),
    int16_t: (view, at, value) => view.setInt16(at, Number(value ?? 0)),
    int8_t: (view, at, value) => view.setInt8(at, Number(value ?? 0)),
    byte: (view, at, value) => view.setUint8(at, Number(value ?? 0)),
    boolean: (view, at, value) => view.setUint8(at, value ? 1 : 0),
}

class Writer {
    bytes = new Uint8Array(256)
    view = new DataView(this.bytes.buffer)
    off = 0

    constructor(private structs: Record<string, Field[]>) {}

    take(n: number): number {
        const at = this.off
        this.off += n
        if (this.off > this.bytes.length) {
            const grown = new Uint8Array(Math.max(this.bytes.length * 2, this.off))
            grown.set(this.bytes)
            this.bytes = grown
            this.view = new DataView(grown.buffer)
        }
        return at
    }

    scalar(type: string, value: unknown): void {
        if (type === "string") {
            const text = utf8Encoder.encode(value === undefined ? "" : String(value))
            const at = this.take(5 + text.length) // the NUL after the text is already 0
            this.view.setInt32(at, text.length + 1)
            this.bytes.set(text, at + 4)
            return
        }
        const at = this.take(SIZE[type])
        SET[type](this.view, at, value)
    }

    struct(name: string, value: MsgValue): void {
        const rows = this.structs[name]
        if (!Array.isArray(rows)) {
            throw new Error(`no schema for struct ${name}`)
        }
        const counts: Record<string, number> = {}
        for (const [field, type, dims] of rows) {
            const item = value?.[field]
            const one = (x: unknown) =>
                type === "string" || Object.hasOwn(SIZE, type) ? this.scalar(type, x) : this.struct(type, (x ?? {}) as MsgValue)
            if (dims === null) {
                const counted = rows.filter(([, , d]) => d?.[0] === field).map(([f]) => value?.[f])
                    .filter((a) => a !== undefined) as ArrayLike<unknown>[]
                counts[field] = counted.length > 0 ? counted[0].length : Number(item ?? 0)
                if (counted.some((a) => a.length !== counts[field])) {
                    throw new Error(`${name}: the arrays counted by ${field} differ in length`)
                }
                one(counted.length > 0 ? counts[field] : item)
                continue
            }
            if (dims.length !== 1) {
                throw new Error(`multi-dimensional arrays are not supported (${name}.${field})`)
            }
            const n = typeof dims[0] === "number" ? dims[0] : counts[dims[0]]
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

function msgType<T>(
    name: string,
    fingerprint: string | null,
    decode: (bytes: Uint8Array) => T,
    encode: (value: unknown) => Uint8Array,
): MsgType<T> {
    return Object.freeze({
        name,
        fingerprint,
        decode,
        encode,
        zenohKey: (topic: string) => `${topic.replace(/\/+$/, "")}/${name}`,
        lcmChannel: (topic: string) => `${topic}#${name}`,
    })
}

export function createRegistry(
    structs: Record<string, Field[]>,
    types: Record<string, string>,
    missing: Record<string, string>,
) {
    const byName = new Map<string, MsgType<unknown>>()
    const byFingerprint = new Map<string, MsgType<unknown>[]>()
    const unsupported = new Map(Object.entries(missing))

    const add = (type: MsgType<unknown>) => {
        byName.set(type.name, type)
        unsupported.delete(type.name)
        if (type.fingerprint !== null) {
            const others = (byFingerprint.get(type.fingerprint) ?? []).filter((t) => t.name !== type.name)
            byFingerprint.set(type.fingerprint, [type, ...others])
        }
        return type
    }

    for (const [name, fp] of Object.entries(types)) {
        let decoder: ((bytes: Uint8Array) => MsgValue) | undefined
        const prefix = Uint8Array.from({ length: 8 }, (_, i) => parseInt(fp.slice(i * 2, i * 2 + 2), 16))
        add(msgType(
            name,
            fp,
            (bytes) => (decoder ??= compileLcmDecoder({ type: name, fp, structs }, { maxArrayElements: Infinity }))(bytes),
            (value) => {
                const w = new Writer(structs)
                w.bytes.set(prefix, w.take(8))
                w.struct(name, value as MsgValue)
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
        if (bytes.byteLength < 8) {
            throw new Error("frame shorter than its 8-byte fingerprint")
        }
        const fingerprint = Array.from(bytes.subarray(0, 8), (b) => b.toString(16).padStart(2, "0")).join("")
        const type = byFingerprint.get(fingerprint)?.[0]
        if (!type) {
            throw new Error(`no message type has fingerprint ${fingerprint}`)
        }
        return type.decode(bytes)
    }

    const decodeChannel = (channel: string, bytes: Uint8Array): unknown => {
        const name = typeOfChannel(channel)
        return name !== undefined && (byName.has(name) || unsupported.has(name)) ? lookup(name).decode(bytes) : decode(bytes)
    }

    const register = <T = unknown>(
        name: string,
        fingerprint: string | null,
        decodeFn: (bytes: Uint8Array) => T,
        encodeFn?: (value: unknown) => Uint8Array,
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
        return add(msgType(name, fingerprint, decodeFn, encode) as MsgType<unknown>) as MsgType<T>
    }

    return {
        decode,
        decodeChannel,
        decodeMessage: (message: { key: string; bytes: Uint8Array; kind?: string }): unknown =>
            message.kind === "delete" ? undefined : decodeChannel(message.key, message.bytes),
        typeOfChannel,
        lookup,
        register,
        getTypeNames: (): string[] => [...byName.keys()].sort(),
        getMissingTypes: (): Record<string, string> => Object.fromEntries(unsupported),
    }
}

/** One object per message package (`geometry_msgs.Twist`, ...), as the generated module exports them. */
export function packages(types: Record<string, string>, lookup: (name: string) => MsgType<unknown>) {
    const out: Record<string, Record<string, MsgType<unknown>>> = {}
    for (const name of Object.keys(types)) {
        const [pkg, short] = name.split(".")
        out[pkg] = { ...out[pkg], [short]: lookup(name) }
    }
    return out
}
