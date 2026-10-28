import rosmsg from "@foxglove/rosmsg";
import serialization from "@foxglove/rosmsg2-serialization";
const { MessageWriter } = serialization;

// The same data-frame envelope used by the DimOS relay; the payload is raw CDR.
export class CommandEncoder {
    constructor(schemas) {
        this.schemas = schemas;
        this.writers = Object.fromEntries(Object.entries(schemas).map(([channel, schema]) =>
            [channel, new MessageWriter(rosmsg.parse(schema.definition, { ros2: true }))]));
        this.sequence = 0;
    }

    encode(channel, message, timestamp = Date.now() / 1000) {
        const writer = this.writers[channel];
        if (!writer) throw new Error(`Unknown WebXR command channel: ${channel}`);
        const payload = writer.writeMessage(message);
        const header = new TextEncoder().encode(JSON.stringify({
            ch: channel, seq: this.sequence++, ts: timestamp, delivery: "latest",
            meta: { encoding: "cdr", type: this.schemas[channel].type }
        }));
        const frame = new Uint8Array(8 + header.length + payload.length);
        const view = new DataView(frame.buffer);
        view.setUint32(0, header.length, true);
        view.setUint32(4, payload.length, true);
        frame.set(header, 8);
        frame.set(payload, 8 + header.length);
        return frame;
    }
}
