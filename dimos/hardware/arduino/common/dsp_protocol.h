/*
 * dsp_protocol.h — DimOS Serial Protocol (DSP)
 *
 * Framed binary protocol for Arduino ↔ Host communication over USB serial.
 *
 * Frame format:
 *   [0xD1] [TOPIC 2B LE] [LENGTH 2B LE] [PAYLOAD 0-1024B] [CRC8 1B]
 *
 * Topic 0 is always DEBUG (UTF-8 text from Serial.print shim).
 * Topics 1..N are data streams, assigned by the generated dimos_arduino.h.
 *
 * This file provides:
 *   - CRC-8/MAXIM table + computation
 *   - dimos_init(baud)
 *   - dimos_send(topic, data, len)
 *   - dimos_check_message() / dimos_message_topic() / dimos_message_data() / dimos_message_len()
 *   - DimosSerial class (Serial.print shim → debug frames)
 *
 * Copyright 2025-2026 Dimensional Inc.  Apache-2.0.
 */

#ifndef DIMOS_DSP_PROTOCOL_H
#define DIMOS_DSP_PROTOCOL_H

#include <stdint.h>
#include <string.h>

/* ======================================================================
 * Constants
 * ====================================================================== */

#define DSP_START_BYTE    0xD1
#define DSP_TOPIC_DEBUG   0
#define DSP_HEADER_SIZE   5   /* START + TOPIC(2) + LENGTH(2) */
#define DSP_OVERHEAD      6   /* HEADER + CRC8 */

/* Maximum payload size.
 *
 * The host-side bridge keeps a generous 1024-byte limit so it can
 * forward any fixed-size LCM message we support (the biggest is
 * PoseWithCovariance at 344 bytes, with room to spare for future
 * types).
 *
 * On AVR, `_dsp_rx_buf` is a static buffer sized at this value — so a
 * 1024-byte default on an Arduino Uno would eat 50% of the chip's 2KB
 * SRAM before the user's sketch has a chance to allocate anything.
 * Default to 256 on AVR (enough for any geometry_msg we ship that's
 * <=256 bytes, which covers every type except PoseWithCovariance and
 * its friends).  Users who need a larger buffer on a chip with more
 * SRAM (Mega 2560 has 8KB) can override via
 * `-DDSP_MAX_PAYLOAD=<bigger>` in their sketch's compile flags. */
#ifndef DSP_MAX_PAYLOAD
#  ifdef __AVR__
#    define DSP_MAX_PAYLOAD   256
#  else
#    define DSP_MAX_PAYLOAD   1024
#  endif
#endif

/* Maximum number of topic handlers */
#ifndef DSP_MAX_TOPICS
#define DSP_MAX_TOPICS    16
#endif

/* Debug buffer size (Serial.print lines) */
#ifndef DSP_DEBUG_BUF_SIZE
#define DSP_DEBUG_BUF_SIZE 128
#endif

/* ======================================================================
 * CRC-8/MAXIM (polynomial 0x31, init 0x00)
 *
 * Lookup table — 256 bytes of flash on AVR.
 * ====================================================================== */

#ifdef __AVR__
#include <avr/pgmspace.h>
#define DSP_CRC_TABLE_ATTR PROGMEM
#define DSP_CRC_READ(addr) pgm_read_byte(addr)
#else
#define DSP_CRC_TABLE_ATTR
#define DSP_CRC_READ(addr) (*(addr))
#endif

static const uint8_t _dsp_crc8_table[256] DSP_CRC_TABLE_ATTR = {
    0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83,
    0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41,
    0x9D, 0xC3, 0x21, 0x7F, 0xFC, 0xA2, 0x40, 0x1E,
    0x5F, 0x01, 0xE3, 0xBD, 0x3E, 0x60, 0x82, 0xDC,
    0x23, 0x7D, 0x9F, 0xC1, 0x42, 0x1C, 0xFE, 0xA0,
    0xE1, 0xBF, 0x5D, 0x03, 0x80, 0xDE, 0x3C, 0x62,
    0xBE, 0xE0, 0x02, 0x5C, 0xDF, 0x81, 0x63, 0x3D,
    0x7C, 0x22, 0xC0, 0x9E, 0x1D, 0x43, 0xA1, 0xFF,
    0x46, 0x18, 0xFA, 0xA4, 0x27, 0x79, 0x9B, 0xC5,
    0x84, 0xDA, 0x38, 0x66, 0xE5, 0xBB, 0x59, 0x07,
    0xDB, 0x85, 0x67, 0x39, 0xBA, 0xE4, 0x06, 0x58,
    0x19, 0x47, 0xA5, 0xFB, 0x78, 0x26, 0xC4, 0x9A,
    0x65, 0x3B, 0xD9, 0x87, 0x04, 0x5A, 0xB8, 0xE6,
    0xA7, 0xF9, 0x1B, 0x45, 0xC6, 0x98, 0x7A, 0x24,
    0xF8, 0xA6, 0x44, 0x1A, 0x99, 0xC7, 0x25, 0x7B,
    0x3A, 0x64, 0x86, 0xD8, 0x5B, 0x05, 0xE7, 0xB9,
    0x8C, 0xD2, 0x30, 0x6E, 0xED, 0xB3, 0x51, 0x0F,
    0x4E, 0x10, 0xF2, 0xAC, 0x2F, 0x71, 0x93, 0xCD,
    0x11, 0x4F, 0xAD, 0xF3, 0x70, 0x2E, 0xCC, 0x92,
    0xD3, 0x8D, 0x6F, 0x31, 0xB2, 0xEC, 0x0E, 0x50,
    0xAF, 0xF1, 0x13, 0x4D, 0xCE, 0x90, 0x72, 0x2C,
    0x6D, 0x33, 0xD1, 0x8F, 0x0C, 0x52, 0xB0, 0xEE,
    0x32, 0x6C, 0x8E, 0xD0, 0x53, 0x0D, 0xEF, 0xB1,
    0xF0, 0xAE, 0x4C, 0x12, 0x91, 0xCF, 0x2D, 0x73,
    0xCA, 0x94, 0x76, 0x28, 0xAB, 0xF5, 0x17, 0x49,
    0x08, 0x56, 0xB4, 0xEA, 0x69, 0x37, 0xD5, 0x8B,
    0x57, 0x09, 0xEB, 0xB5, 0x36, 0x68, 0x8A, 0xD4,
    0x95, 0xCB, 0x29, 0x77, 0xF4, 0xAA, 0x48, 0x16,
    0xE9, 0xB7, 0x55, 0x0B, 0x88, 0xD6, 0x34, 0x6A,
    0x2B, 0x75, 0x97, 0xC9, 0x4A, 0x14, 0xF6, 0xA8,
    0x74, 0x2A, 0xC8, 0x96, 0x15, 0x4B, 0xA9, 0xF7,
    0xB6, 0xE8, 0x0A, 0x54, 0xD7, 0x89, 0x6B, 0x35,
};

static inline uint8_t dsp_crc8(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0x00;
    uint16_t i;
    for (i = 0; i < len; i++) {
        crc = DSP_CRC_READ(&_dsp_crc8_table[crc ^ data[i]]);
    }
    return crc;
}

/* ======================================================================
 * Shared parser state machine
 *
 * The wire-framing parser is identical on the Arduino and on the host
 * bridge — both reconstruct a DSP frame from a byte stream — so the
 * state struct and the single-byte step function live here, in the
 * portable section of the header, and are #included by both sides.
 * Historically these were duplicated in main.cpp and drifted, so
 * protocol changes had to be made in two places with no CI guard on
 * drift.  One definition, one place to change.
 *
 * Public API:
 *   - ``struct dsp_parser``        — callers keep one per stream
 *   - ``dsp_parser_init(&p)``      — zero-initialize before first feed
 *   - ``dsp_feed_byte(&p, b)``     — step machine with one byte, returns
 *                                    a ``dsp_parse_event`` telling the
 *                                    caller what to do next
 *
 * Event semantics:
 *   DSP_PARSE_NONE      — keep feeding
 *   DSP_PARSE_MESSAGE   — complete frame; read ``p.rx_topic/rx_len/rx_buf``
 *                         before the next ``dsp_feed_byte`` call
 *   DSP_PARSE_CRC_FAIL  — framing OK but CRC mismatch; parser is already
 *                         reset and ready for more
 *   DSP_PARSE_OVERFLOW  — declared length exceeds ``DSP_MAX_PAYLOAD``;
 *                         parser is reset
 * ====================================================================== */

enum dsp_parse_state {
    DSP_WAIT_START,
    DSP_READ_TOPIC_LO,
    DSP_READ_TOPIC_HI,
    DSP_READ_LEN_LO,
    DSP_READ_LEN_HI,
    DSP_READ_PAYLOAD,
    DSP_READ_CRC
};

enum dsp_parse_event {
    DSP_PARSE_NONE = 0,
    DSP_PARSE_MESSAGE,
    DSP_PARSE_CRC_FAIL,
    DSP_PARSE_OVERFLOW
};

struct dsp_parser {
    enum dsp_parse_state state;
    uint16_t rx_topic;
    uint16_t rx_len;
    uint16_t rx_payload_pos;
    uint8_t  rx_buf[DSP_MAX_PAYLOAD];
};

static inline void dsp_parser_init(struct dsp_parser *p)
{
    p->state = DSP_WAIT_START;
    p->rx_topic = 0;
    p->rx_len = 0;
    p->rx_payload_pos = 0;
}

static inline enum dsp_parse_event dsp_feed_byte(struct dsp_parser *p, uint8_t b)
{
    switch (p->state) {
    case DSP_WAIT_START:
        if (b == DSP_START_BYTE) {
            p->state = DSP_READ_TOPIC_LO;
        }
        return DSP_PARSE_NONE;

    case DSP_READ_TOPIC_LO:
        p->rx_topic = b;
        p->state = DSP_READ_TOPIC_HI;
        return DSP_PARSE_NONE;

    case DSP_READ_TOPIC_HI:
        p->rx_topic |= ((uint16_t)b << 8);
        p->state = DSP_READ_LEN_LO;
        return DSP_PARSE_NONE;

    case DSP_READ_LEN_LO:
        p->rx_len = b;
        p->state = DSP_READ_LEN_HI;
        return DSP_PARSE_NONE;

    case DSP_READ_LEN_HI:
        p->rx_len |= ((uint16_t)b << 8);
        if (p->rx_len > DSP_MAX_PAYLOAD) {
            p->state = DSP_WAIT_START;
            return DSP_PARSE_OVERFLOW;
        }
        p->rx_payload_pos = 0;
        p->state = (p->rx_len == 0) ? DSP_READ_CRC : DSP_READ_PAYLOAD;
        return DSP_PARSE_NONE;

    case DSP_READ_PAYLOAD:
        p->rx_buf[p->rx_payload_pos++] = b;
        if (p->rx_payload_pos >= p->rx_len) {
            p->state = DSP_READ_CRC;
        }
        return DSP_PARSE_NONE;

    case DSP_READ_CRC: {
        /* CRC-8/MAXIM over TOPIC_LO + TOPIC_HI + LEN_LO + LEN_HI + PAYLOAD,
         * computed incrementally via the table.  No temporary buffer needed. */
        uint8_t crc = 0x00;
        crc = DSP_CRC_READ(&_dsp_crc8_table[crc ^ (uint8_t)(p->rx_topic & 0xFF)]);
        crc = DSP_CRC_READ(&_dsp_crc8_table[crc ^ (uint8_t)((p->rx_topic >> 8) & 0xFF)]);
        crc = DSP_CRC_READ(&_dsp_crc8_table[crc ^ (uint8_t)(p->rx_len & 0xFF)]);
        crc = DSP_CRC_READ(&_dsp_crc8_table[crc ^ (uint8_t)((p->rx_len >> 8) & 0xFF)]);
        for (uint16_t k = 0; k < p->rx_len; k++) {
            crc = DSP_CRC_READ(&_dsp_crc8_table[crc ^ p->rx_buf[k]]);
        }

        p->state = DSP_WAIT_START;
        if (crc == b) {
            return DSP_PARSE_MESSAGE;
        }
        return DSP_PARSE_CRC_FAIL;
    }
    }
    /* Unreachable — every enum value is handled above.  Reset defensively. */
    p->state = DSP_WAIT_START;
    return DSP_PARSE_NONE;
}

/* ======================================================================
 * Platform abstraction (Arduino vs host C++)
 *
 * On Arduino: uses HardwareSerial directly.
 * On host (for testing): stubs can be provided.
 * ====================================================================== */

#ifdef ARDUINO

/* ======================================================================
 * Arduino Implementation
 *
 * Two back-ends:
 *
 * 1. **HardwareSerial** (default on real hardware) — uses Arduino's
 *    ``Serial`` object, which has a 64-byte interrupt-driven RX buffer.
 *    At 115200 baud bytes arrive every ~87µs; the 2-byte hardware FIFO
 *    overflows whenever the sketch does any non-trivial work (encoding a
 *    reply, printing debug text, even a 1ms delay).  HardwareSerial
 *    handles this transparently via the RXCIE0 ISR.
 *
 * 2. **Direct USART register access** (QEMU / simulator builds) — used
 *    when ``DSP_DIRECT_USART`` is defined.  QEMU's AVR USART model
 *    does not fire interrupts, so HardwareSerial's ISR never triggers
 *    and the RX buffer stays empty.  Direct register access works
 *    because QEMU simulates AVR instructions far faster than real time,
 *    so the 2-byte FIFO never overflows in practice.
 *
 * To force the direct path on real hardware (e.g. bare-metal without
 * the Arduino core), ``-DDSP_DIRECT_USART`` in compile flags.
 * ====================================================================== */

#include <Arduino.h>

#ifdef DSP_DIRECT_USART

/* --- Direct USART0 (QEMU / bare-metal) --- */

#include <avr/io.h>

#if !defined(__AVR_ATmega328P__) && !defined(__AVR_ATmega328PB__) && \
    !defined(__AVR_ATmega2560__) && !defined(__AVR_ATmega1280__)
#error "DSP_DIRECT_USART only supports ATmega328P / 328PB / 1280 / 2560 USART0."
#endif

static inline void _dsp_usart_init(uint32_t baud) {
    uint16_t ubrr = (uint16_t)((F_CPU / 8 / baud) - 1);
    UBRR0H = (uint8_t)(ubrr >> 8);
    UBRR0L = (uint8_t)(ubrr & 0xFF);
    UCSR0A = (1 << U2X0);
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

static inline void _dsp_usart_write(uint8_t b) {
    while (!(UCSR0A & (1 << UDRE0))) {}
    UDR0 = b;
}

static inline bool _dsp_usart_available(void) {
    return (UCSR0A & (1 << RXC0)) != 0;
}

static inline uint8_t _dsp_usart_read(void) {
    return UDR0;
}

#else /* !DSP_DIRECT_USART — HardwareSerial (real hardware) */

static inline void _dsp_usart_init(uint32_t baud) {
    Serial.begin(baud);
}

static inline void _dsp_usart_write(uint8_t b) {
    Serial.write(b);
}

static inline bool _dsp_usart_available(void) {
    return Serial.available() > 0;
}

static inline uint8_t _dsp_usart_read(void) {
    return (uint8_t)Serial.read();
}

#endif /* DSP_DIRECT_USART */

/* --- Internal state ---
 *
 * The parser state must be shared across all translation units that
 * include this header, otherwise a sketch split across multiple .cpp /
 * .ino files ends up with one independent state machine per TU — the
 * second TU's `dimos_check_message()` would see an empty buffer.
 *
 * We expose the parser via a plain (non-``static``) ``inline`` function
 * whose function-local static is guaranteed by the C++ standard to
 * resolve to a single object across TUs.  The parser struct and the
 * step function themselves live above, in the portable section shared
 * with the host bridge. */

inline struct dsp_parser &_dsp_state_ref(void)
{
    static struct dsp_parser s = {
        /* state          */ DSP_WAIT_START,
        /* rx_topic       */ 0,
        /* rx_len         */ 0,
        /* rx_payload_pos */ 0,
        /* rx_buf         */ {0},
    };
    return s;
}

/**
 * Initialize DimOS serial protocol.
 * Call this in setup() before any other dimos_* calls.
 *
 * When the LCM serial adapter is used (dimos_lcm_serial.h), that header
 * provides its own dimos_init() that also initialises the pubsub layer.
 */
#ifndef DIMOS_LCM_SERIAL_H
static inline void dimos_init(uint32_t baud)
{
    _dsp_usart_init(baud);
    dsp_parser_init(&_dsp_state_ref());
}
#endif

/**
 * Send a DSP frame.
 *
 * @param topic  Topic enum value (DIMOS_TOPIC_DEBUG, DIMOS_TOPIC__*, etc.)
 * @param data   Payload bytes (LCM-encoded for data topics, UTF-8 for debug)
 * @param len    Payload length in bytes
 */
static inline void dimos_send(uint16_t topic, const uint8_t *data, uint16_t len)
{
    if (len > DSP_MAX_PAYLOAD) return;

    /* Build header: START, TOPIC (2B LE), LENGTH (2B LE) */
    uint8_t header[DSP_HEADER_SIZE];
    header[0] = DSP_START_BYTE;
    header[1] = (uint8_t)(topic & 0xFF);
    header[2] = (uint8_t)((topic >> 8) & 0xFF);
    header[3] = (uint8_t)(len & 0xFF);
    header[4] = (uint8_t)((len >> 8) & 0xFF);

    /* CRC over TOPIC + LENGTH + PAYLOAD */
    uint8_t crc = 0x00;
    uint16_t i;
    for (i = 1; i < DSP_HEADER_SIZE; i++) {
        crc = DSP_CRC_READ(&_dsp_crc8_table[crc ^ header[i]]);
    }
    for (i = 0; i < len; i++) {
        crc = DSP_CRC_READ(&_dsp_crc8_table[crc ^ data[i]]);
    }

    /* Write frame byte-by-byte via direct USART */
    uint16_t k;
    for (k = 0; k < DSP_HEADER_SIZE; k++) _dsp_usart_write(header[k]);
    for (k = 0; k < len; k++)              _dsp_usart_write(data[k]);
    _dsp_usart_write(crc);
}

/**
 * Check for the next incoming DSP message.
 *
 * Reads available serial bytes and attempts to parse a complete frame.
 * Returns true if a valid message is ready.  Use dimos_message_topic(),
 * dimos_message_data(), and dimos_message_len() to access it.
 *
 * Typical usage in loop():
 *
 *   while (dimos_check_message()) {
 *       switch (dimos_message_topic()) {
 *       case DIMOS_TOPIC__MY_INPUT:
 *           MyType msg;
 *           MyType_decode(dimos_message_data(), 0, dimos_message_len(), &msg);
 *           // use msg...
 *           break;
 *       }
 *   }
 */
/* Maximum bytes `dimos_check_message` will process in one call.  Prevents
 * a flood of 1-byte frames from starving the user's loop().  Override by
 * defining DSP_CHECK_MAX_BYTES before including this header. */
#ifndef DSP_CHECK_MAX_BYTES
#define DSP_CHECK_MAX_BYTES 256
#endif

static inline bool dimos_check_message(void)
{
    struct dsp_parser &s = _dsp_state_ref();

    uint16_t bytes_processed = 0;
    while (_dsp_usart_available() && bytes_processed < DSP_CHECK_MAX_BYTES) {
        uint8_t b = _dsp_usart_read();
        bytes_processed++;

        enum dsp_parse_event ev = dsp_feed_byte(&s, b);
        if (ev == DSP_PARSE_MESSAGE) {
            return true;
        }
        /* DSP_PARSE_CRC_FAIL / DSP_PARSE_OVERFLOW / DSP_PARSE_NONE:
         * the parser has already reset itself; just keep reading. */
    }

    return false;  /* no complete message available (yet) */
}

/**
 * Get the topic of the last received message.
 * Only valid after dimos_check_message() returned true.
 */
static inline enum dimos_topic dimos_message_topic(void)
{
    return (enum dimos_topic)_dsp_state_ref().rx_topic;
}

/**
 * Get a pointer to the payload of the last received message.
 * Only valid after dimos_check_message() returned true.
 */
static inline const uint8_t *dimos_message_data(void)
{
    return _dsp_state_ref().rx_buf;
}

/**
 * Get the payload length of the last received message.
 * Only valid after dimos_check_message() returned true.
 */
static inline uint16_t dimos_message_len(void)
{
    return _dsp_state_ref().rx_len;
}

/* ======================================================================
 * Serial.print shim
 *
 * Intercepts Serial.print/println and sends output as DSP debug frames
 * (topic 0).  Flushes on newline or buffer full.
 * ====================================================================== */

class DimosSerial_ : public Print {
public:
    size_t write(uint8_t b) override {
        _buf[_pos++] = b;
        if (b == '\n' || _pos >= DSP_DEBUG_BUF_SIZE) {
            _flush();
        }
        return 1;
    }

    size_t write(const uint8_t *buffer, size_t size) override {
        size_t i;
        for (i = 0; i < size; i++) {
            write(buffer[i]);
        }
        return size;
    }

    void flush() {
        if (_pos > 0) _flush();
    }

private:
    uint8_t _buf[DSP_DEBUG_BUF_SIZE];
    uint8_t _pos = 0;

    void _flush() {
        dimos_send((uint16_t)DSP_TOPIC_DEBUG, _buf, _pos);
        _pos = 0;
    }
};

/* Must be `inline` (not `static inline`) so the function-local static
 * below collapses to one object across every TU that includes this
 * header — the same trick used by `_dsp_state_ref` above.  A plain
 * `static DimosSerial_ DimosSerial;` here would give each TU its own
 * buffer, and a library-backed `.cpp` writing through the name would
 * see independent `_buf`/`_pos` state from the `.ino`. */
inline DimosSerial_ &_dimos_serial_ref(void)
{
    static DimosSerial_ s;
    return s;
}
#define DimosSerial (_dimos_serial_ref())

/*
 * IMPORTANT: use `DimosSerial.print/println(...)` in your sketch, not
 * `Serial.print/println(...)`.
 *
 * Earlier versions of this header installed `#define Serial DimosSerial`
 * so that existing `Serial.print` calls would transparently route through
 * the DSP debug channel.  That was removed because macro-replacing
 * `Serial` breaks any third-party library (Wire, SPI, motor drivers,
 * etc.) that references `Serial` internally — those libraries would try
 * to call `DimosSerial.available()` / `.read()` which don't exist, and
 * fail to compile deep inside the library header.
 *
 * If you want a shim in your own sketch, add this AFTER all library
 * includes:
 *
 *     #define Serial DimosSerial
 */

#endif /* ARDUINO */

/* ======================================================================
 * Host-side (C++) utilities
 *
 * The C++ bridge doesn't use dimos_init/dimos_send/dimos_poll (it has
 * its own implementation with termios).  But it shares the constants
 * and CRC function.
 * ====================================================================== */

#endif /* DIMOS_DSP_PROTOCOL_H */
