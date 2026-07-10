/*
 * Twist Echo — Example DimOS Arduino sketch.
 *
 * Receives Twist commands from the host, echoes them back.
 * Demonstrates:
 *   - dimos_init() / dimos_subscribe() / dimos_handle()
 *   - Typed callbacks matching C++ LCM style
 *   - Using generated type descriptors for subscribe/publish
 *   - DimosSerial.println() going through the DSP debug channel
 *   - Config values available as #defines
 *
 * NOTE: We use _delay_ms() from <util/delay.h> instead of Arduino's delay()
 * because delay() relies on timer 0 interrupts which don't fire in QEMU's
 * AVR model.  _delay_ms is a pure busy loop and works in any simulator.
 */

#include "dimos_arduino.h"
#include <util/delay.h>

/* Shared state — accessible from callback */
dimos_msg__Twist last_twist;
uint32_t msg_count = 0;

void on_twist(const char *channel, const void *msg, void *ctx) {
    (void)channel;
    (void)ctx;
    const dimos_msg__Twist *twist = (const dimos_msg__Twist *)msg;

    /* Copy to shared state */
    last_twist = *twist;
    msg_count++;

    DimosSerial.print("Got twist #");
    DimosSerial.print(msg_count);
    DimosSerial.print(": linear.x=");
    DimosSerial.println(twist->linear.x);

    /* Echo it back */
    uint8_t buf[48];
    int encoded = dimos_msg__Twist__encode(buf, 0, sizeof(buf), twist);
    if (encoded > 0) {
        dimos_publish(DIMOS_CHANNEL__TWIST_ECHO_OUT,
                      &dimos_msg__Twist__type, buf, encoded);
    }
}

void setup() {
    dimos_init(DIMOS_BAUDRATE);
    dimos_subscribe(DIMOS_CHANNEL__TWIST_IN,
                    &dimos_msg__Twist__type, on_twist, NULL);
    DimosSerial.println("TwistEcho ready");
}

void loop() {
    dimos_handle(10);
    _delay_ms(1);
}
