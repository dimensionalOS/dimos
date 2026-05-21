// Wire format mirror of `messages.py` on the memory-world server.
//
// Server -> client:
//   binary frames: [1 byte type][4 bytes hdr len LE][hdr JSON][payload bytes]
//   text frames:   JSON object with a "type" field
//
// Client -> server:
//   text frames only — JSON objects (mostly diagnostics).

export const MSG_POINT_CLOUD = 0x01;
export const MSG_IMAGE_POSES = 0x02;
export const MSG_ODOM_TRAIL = 0x03;
export const MSG_TOP_DOWN_MAP = 0x04;
export const MSG_IMAGE_THUMBNAIL = 0x05;

export function decodeBinary(buffer) {
    const view = new DataView(buffer);
    const msgType = view.getUint8(0);
    const hdrLen = view.getUint32(1, true);
    const hdrBytes = new Uint8Array(buffer, 5, hdrLen);
    const header = JSON.parse(new TextDecoder('utf-8').decode(hdrBytes));
    const payload = buffer.slice(5 + hdrLen);
    return { msgType, header, payload };
}

export function decodeText(text) {
    try {
        const obj = JSON.parse(text);
        return (obj && typeof obj === 'object') ? obj : null;
    } catch (_) {
        return null;
    }
}

export function encodeText(type, fields = {}) {
    return JSON.stringify({ type, ...fields });
}
