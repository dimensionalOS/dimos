/*
 * Multi-type Echo — DimOS Arduino hardware test sketch.
 *
 * Echoes Bool, Int32, Vector3, and Quaternion messages back to the host.
 * Used to validate round-trip serialization correctness across multiple
 * message types, including float64→float32 precision on AVR.
 */

#include "dimos_arduino.h"
#include <util/delay.h>

/* --- Bool echo --- */
void on_bool(const char *ch, const void *msg, void *ctx) {
    (void)ch; (void)ctx;
    const dimos_msg__Bool *m = (const dimos_msg__Bool *)msg;
    uint8_t buf[1];
    int n = dimos_msg__Bool__encode(buf, 0, sizeof(buf), m);
    if (n > 0) dimos_publish(DIMOS_CHANNEL__BOOL_OUT, &dimos_msg__Bool__type, buf, n);
}

/* --- Int32 echo --- */
void on_int32(const char *ch, const void *msg, void *ctx) {
    (void)ch; (void)ctx;
    const dimos_msg__Int32 *m = (const dimos_msg__Int32 *)msg;
    uint8_t buf[4];
    int n = dimos_msg__Int32__encode(buf, 0, sizeof(buf), m);
    if (n > 0) dimos_publish(DIMOS_CHANNEL__INT32_OUT, &dimos_msg__Int32__type, buf, n);
}

/* --- Vector3 echo --- */
void on_vec3(const char *ch, const void *msg, void *ctx) {
    (void)ch; (void)ctx;
    const dimos_msg__Vector3 *m = (const dimos_msg__Vector3 *)msg;
    uint8_t buf[24];
    int n = dimos_msg__Vector3__encode(buf, 0, sizeof(buf), m);
    if (n > 0) dimos_publish(DIMOS_CHANNEL__VEC3_OUT, &dimos_msg__Vector3__type, buf, n);
}

/* --- Quaternion echo --- */
void on_quat(const char *ch, const void *msg, void *ctx) {
    (void)ch; (void)ctx;
    const dimos_msg__Quaternion *m = (const dimos_msg__Quaternion *)msg;
    uint8_t buf[32];
    int n = dimos_msg__Quaternion__encode(buf, 0, sizeof(buf), m);
    if (n > 0) dimos_publish(DIMOS_CHANNEL__QUAT_OUT, &dimos_msg__Quaternion__type, buf, n);
}

void setup() {
    dimos_init(DIMOS_BAUDRATE);
    dimos_subscribe(DIMOS_CHANNEL__BOOL_IN,  &dimos_msg__Bool__type,       on_bool,  NULL);
    dimos_subscribe(DIMOS_CHANNEL__INT32_IN, &dimos_msg__Int32__type,      on_int32, NULL);
    dimos_subscribe(DIMOS_CHANNEL__VEC3_IN,  &dimos_msg__Vector3__type,    on_vec3,  NULL);
    dimos_subscribe(DIMOS_CHANNEL__QUAT_IN,  &dimos_msg__Quaternion__type, on_quat,  NULL);
    DimosSerial.println("MultiEcho ready");
}

void loop() {
    dimos_handle(10);
    _delay_ms(1);
}
