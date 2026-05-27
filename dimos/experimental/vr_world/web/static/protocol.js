// Wire format mirror of vr_world/messages.py.
//
// Server -> client:
//   binary: [1B type][4B hdr len LE][hdr JSON][payload]
//   text:   JSON object with "type"
// Client -> server: text JSON only.

export const MSG_VOXEL_MAP = 0x01;
export const MSG_CAMERA = 0x02;

export function decodeBinary(buffer) {
    const view = new DataView(buffer);
    const msgType = view.getUint8(0);
    const hdrLen = view.getUint32(1, true);
    const hdrBytes = new Uint8Array(buffer, 5, hdrLen);
    const header = hdrLen ? JSON.parse(new TextDecoder('utf-8').decode(hdrBytes)) : {};
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
