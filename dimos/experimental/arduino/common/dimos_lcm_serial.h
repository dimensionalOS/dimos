/*
 * dimos_lcm_serial.h — Serial transport adapter for the DimOS LCM pubsub layer.
 *
 * Bridges the transport-agnostic LCM pubsub engine (dimos_lcm_pubsub.h) with
 * the DSP serial framing protocol (dsp_protocol.h), providing a user-friendly
 * API that matches standard LCM patterns:
 *
 *   dimos_init(baud)                       — initialise serial + pubsub
 *   dimos_subscribe(ch, type, handler, ud) — register a typed callback
 *   dimos_publish(ch, type, data, len)     — send a message
 *   dimos_handle(timeout_ms)               — poll serial, dispatch, send
 *
 * The generated dimos_arduino.h must define the following BEFORE including
 * this header:
 *
 *   DIMOS_NUM_TOPICS     — number of topic mappings
 *   _dimos_topic_map[]   — array of { topic_id, channel_name } structs
 *
 * Copyright 2025-2026 Dimensional Inc.  Apache-2.0.
 */

#ifndef DIMOS_LCM_SERIAL_H
#define DIMOS_LCM_SERIAL_H

#include "dimos_lcm_pubsub.h"
#include "dsp_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ======================================================================
 * Topic mapping (populated by generated dimos_arduino.h)
 * ====================================================================== */

#ifndef DIMOS_TOPIC_MAPPING_DEFINED
#define DIMOS_TOPIC_MAPPING_DEFINED
typedef struct {
    uint16_t    topic_id;
    const char *channel;
} dimos_topic_mapping_t;
#endif

/* These must be defined by the generated header before including us:
 *   static const dimos_topic_mapping_t _dimos_topic_map[];
 *   #define DIMOS_NUM_TOPICS <count>
 */

/* ======================================================================
 * Global state
 *
 * Uses the same inline-function-with-static-local trick as dsp_protocol.h
 * to ensure a single instance across translation units.
 * ====================================================================== */

inline dimos_lcm_t &_dimos_lcm_ref(void)
{
    static dimos_lcm_t s;
    return s;
}

/* ======================================================================
 * Channel ↔ topic ID lookup
 * ====================================================================== */

static inline const char *_dimos_channel_for_topic(uint16_t topic_id)
{
    for (int i = 0; i < DIMOS_NUM_TOPICS; i++) {
        if (_dimos_topic_map[i].topic_id == topic_id)
            return _dimos_topic_map[i].channel;
    }
    return (const char *)0;
}

static inline uint16_t _dimos_topic_for_channel(const char *channel)
{
    for (int i = 0; i < DIMOS_NUM_TOPICS; i++) {
        if (strcmp(_dimos_topic_map[i].channel, channel) == 0)
            return _dimos_topic_map[i].topic_id;
    }
    return 0xFFFF;  /* not found */
}

/* ======================================================================
 * Public API
 * ====================================================================== */

/**
 * Initialise DimOS serial + LCM pubsub.
 * Call this in setup() before any other dimos_* calls.
 */
static inline void dimos_init(uint32_t baud)
{
    _dsp_usart_init(baud);
    dsp_parser_init(&_dsp_state_ref());
    dimos_lcm_init(&_dimos_lcm_ref());
}

/**
 * Subscribe to messages on a channel with automatic decoding.
 *
 * The handler receives (channel_name, decoded_msg_ptr, user_data).
 * The decoded_msg_ptr is valid only for the duration of the callback.
 *
 * @param channel    Channel name (e.g. "/twist")
 * @param type       Type descriptor (e.g. &dimos_msg__Twist__type)
 * @param handler    Callback: void handler(const char *ch, const void *msg, void *ud)
 * @param user_data  Opaque pointer forwarded to handler
 * @return           Subscription index (>=0) on success, -1 on error
 *
 * Example:
 *   void on_twist(const char *ch, const void *msg, void *ud) {
 *       const dimos_msg__Twist *t = (const dimos_msg__Twist *)msg;
 *       // use t->linear.x ...
 *   }
 *   dimos_subscribe("/twist", &dimos_msg__Twist__type, on_twist, NULL);
 */
static inline int dimos_subscribe(const char *channel,
                                   const dimos_lcm_type_t *type,
                                   dimos_lcm_handler_t handler,
                                   void *user_data)
{
    return dimos_lcm_subscribe(&_dimos_lcm_ref(), channel, type, handler, user_data);
}

/**
 * Publish a message on a channel.
 *
 * @param channel       Channel name
 * @param type          Type descriptor
 * @param encoded_data  Pre-encoded message bytes (from _encode())
 * @param data_len      Length of encoded data
 * @return              0 on success, -1 on error
 */
static inline int dimos_publish(const char *channel,
                                 const dimos_lcm_type_t *type,
                                 const uint8_t *encoded_data,
                                 uint16_t data_len)
{
    return dimos_lcm_publish(&_dimos_lcm_ref(), channel, type, encoded_data, data_len);
}

/**
 * Poll serial, dispatch inbound messages, and send outbound messages.
 *
 * This is the main event loop function, matching lcm.handleTimeout(ms).
 * Call this in loop() to process messages.
 *
 * @param timeout_ms  Not used on Arduino (non-blocking poll), but kept
 *                    for API compatibility with C++ LCM.
 * @return            Number of messages dispatched.
 */
#ifndef DSP_CHECK_MAX_BYTES
#define DSP_CHECK_MAX_BYTES 256
#endif

static inline int dimos_handle(int timeout_ms)
{
    (void)timeout_ms;  /* Arduino is always non-blocking poll */

    dimos_lcm_t *lcm = &_dimos_lcm_ref();
    struct dsp_parser &parser = _dsp_state_ref();
    int dispatched = 0;

    /* --- Inbound: serial → DSP parser → LCM pubsub dispatch --- */
    uint16_t bytes_processed = 0;
    while (_dsp_usart_available() && bytes_processed < DSP_CHECK_MAX_BYTES) {
        uint8_t b = _dsp_usart_read();
        bytes_processed++;

        enum dsp_parse_event ev = dsp_feed_byte(&parser, b);
        if (ev == DSP_PARSE_MESSAGE) {
            /* Look up the channel name for this topic ID */
            const char *channel = _dimos_channel_for_topic(parser.rx_topic);
            if (channel != (const char *)0) {
                /* Payload contains [8B fingerprint][encoded data] */
                int n = dimos_lcm_dispatch(lcm, channel, parser.rx_buf, parser.rx_len);
                if (n > 0) dispatched += n;
            }
        }
        /* CRC_FAIL / OVERFLOW: parser resets itself, keep reading */
    }

    /* --- Outbound: LCM pubsub outbox → DSP frames → serial --- */
    while (dimos_lcm_has_outbound(lcm)) {
        const char *channel = (const char *)0;
        uint8_t wire_buf[DIMOS_LCM_FINGERPRINT_SIZE + DIMOS_LCM_MAX_MSG_SIZE];
        uint16_t wire_len = dimos_lcm_pop_outbound(lcm, &channel, wire_buf, sizeof(wire_buf));
        if (wire_len == 0 || channel == (const char *)0) break;

        uint16_t topic_id = _dimos_topic_for_channel(channel);
        if (topic_id != 0xFFFF) {
            dimos_send(topic_id, wire_buf, wire_len);
        }
    }

    return dispatched;
}

#ifdef __cplusplus
}
#endif

#endif /* DIMOS_LCM_SERIAL_H */
