package com.dimensional.mini4pro.handshake

import io.dronefleet.mavlink.common.FileTransferProtocol

/**
 * Just enough MAVLink FTP to say "no, and here is why" quickly.
 *
 * **No longer on QGroundControl's critical path.** This was written for the ArduPilot
 * masquerade: `ParameterManager`'s constructor sets `_tryftp = vehicle->apmFirmware()`
 * (`ParameterManager.cc:42`), so an APM-flavoured vehicle's *first* parameter download attempt
 * is an FTP fetch of `@PARAM/param.pck?withdefaults=1` — ignoring
 * `MAV_PROTOCOL_CAPABILITY_FTP` entirely — and while `_tryftp` is set QGC even *drops* incoming
 * `PARAM_VALUE` from the autopilot component (`ParameterManager.cc:101`). Silence cost ~18 s
 * before `PARAM_REQUEST_LIST` appeared: `OpenFileRO` retried 4× at 1 s
 * (`FTPManager.h:_ackOrNakTimeoutMsecs`, `_maxRetry`), then a 5 s `_paramRequestListTimer`, then
 * the whole FTP attempt again. A NAK collapsed that to nothing, because `_ftpDownloadComplete`
 * special-cases an error message containing "File Not Found" and falls straight through to
 * `PARAM_REQUEST_LIST` (`ParameterManager.cc:543`), and
 * `MavlinkFTP::errorCodeToString(kErrFailFileNotFound)` is exactly that string.
 *
 * `_tryftp` is `apmFirmware()`, not `px4Firmware()`, so identifying as `MAV_AUTOPILOT_PX4` keeps
 * this off the parameter path exactly as `MAV_AUTOPILOT_GENERIC` did — the identity change did
 * not bring it back. Kept anyway because it is cheap, self-contained and tested, and because an
 * honest "no" beats silence for any other client that asks — MAVProxy, Mission Planner, or QGC's
 * onboard-log browser, which is the one place `MAV_PROTOCOL_CAPABILITY_FTP` is actually read
 * (`OnboardLogController.cc:500`) and a reason not to claim that bit. It can be switched off with
 * `nakFileTransfers = false`, and it would be needed again if anyone re-enabled an ArduPilot
 * identity.
 *
 * We do not implement MAVLink FTP and do not advertise `MAV_PROTOCOL_CAPABILITY_FTP`, so every
 * request gets the NAK error code that best describes it.
 *
 * Wire format is `MavlinkFTP::RequestHeader` (`src/MAVLink/MAVLinkFTP.h`), a packed 12-byte
 * header at the start of the 251-byte `FILE_TRANSFER_PROTOCOL.payload`:
 * `seqNumber:u16, session:u8, opcode:u8, size:u8, req_opcode:u8, burstComplete:u8, padding:u8,
 * offset:u32`, then 239 data bytes.
 */
object MavlinkFtp {

    const val PAYLOAD_LENGTH = 251
    private const val HEADER_LENGTH = 12

    // Opcodes we care about naming (MavlinkFTP::OpCode_t).
    const val CMD_TERMINATE_SESSION = 1
    const val CMD_RESET_SESSIONS = 2
    const val CMD_READ_FILE = 5
    const val CMD_WRITE_FILE = 7
    const val CMD_BURST_READ_FILE = 15
    const val RSP_ACK = 128
    const val RSP_NAK = 129

    // Error codes (MavlinkFTP::ErrorCode_t).
    const val ERR_INVALID_SESSION = 4
    const val ERR_UNKNOWN_COMMAND = 7
    const val ERR_FILE_NOT_FOUND = 10

    /** The fields of an inbound request that we need in order to answer it. */
    data class Request(val sequenceNumber: Int, val session: Int, val opcode: Int)

    /** Returns null when the payload is too short to be a request header. */
    fun parse(payload: ByteArray?): Request? {
        if (payload == null || payload.size < HEADER_LENGTH) return null
        val seq = (payload[0].toInt() and 0xFF) or ((payload[1].toInt() and 0xFF) shl 8)
        return Request(
            sequenceNumber = seq,
            session = payload[2].toInt() and 0xFF,
            opcode = payload[3].toInt() and 0xFF,
        )
    }

    /**
     * The honest error for a request we will never serve.
     *
     * Session-scoped operations get `kErrInvalidSession` (we hold no sessions), a response to
     * one of our own responses gets `kErrUnknownCommand`, and everything that names a path gets
     * `kErrFailFileNotFound` — which is also the code an APM-identifying QGC turns into an
     * immediate `PARAM_REQUEST_LIST`.
     */
    fun errorCodeFor(opcode: Int): Int = when (opcode) {
        CMD_TERMINATE_SESSION, CMD_RESET_SESSIONS, CMD_READ_FILE, CMD_WRITE_FILE,
        CMD_BURST_READ_FILE -> ERR_INVALID_SESSION
        RSP_ACK, RSP_NAK -> ERR_UNKNOWN_COMMAND
        else -> ERR_FILE_NOT_FOUND
    }

    /**
     * Builds the NAK for [request].
     *
     * `seqNumber` must be the request's sequence number plus one: `FTPManager` sends with
     * `hdr.seqNumber = _expectedIncomingSeqNumber + 1` and then advances the expected value by
     * two, so it matches replies against `requestSeq + 1`
     * (`FTPManager.cc:_sendRequestExpectAck`, `_openFileROAckOrNak`).
     *
     * `size = 1` with one data byte is the format `_errorMsgFromNak` requires; any other size
     * is reported as "Invalid Nak format" and loses us the "File Not Found" fast path.
     */
    fun nak(request: Request, targetSystem: Int, targetComponent: Int): FileTransferProtocol {
        val payload = ByteArray(PAYLOAD_LENGTH)
        val seq = (request.sequenceNumber + 1) and 0xFFFF
        payload[0] = (seq and 0xFF).toByte()
        payload[1] = ((seq shr 8) and 0xFF).toByte()
        payload[2] = request.session.toByte()
        payload[3] = RSP_NAK.toByte()
        payload[4] = 1                                          // size: one error byte
        payload[5] = request.opcode.toByte()                    // req_opcode
        payload[6] = 0                                          // burstComplete
        payload[7] = 0                                          // padding
        // offset stays 0
        payload[HEADER_LENGTH] = errorCodeFor(request.opcode).toByte()

        return FileTransferProtocol.builder()
            .targetNetwork(0)
            .targetSystem(targetSystem)
            .targetComponent(targetComponent)
            .payload(payload)
            .build()
    }
}
