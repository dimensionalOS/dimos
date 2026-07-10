/*
 * LED Echo — Example DimOS Arduino sketch.
 *
 * Receives Bool commands from the host to control the built-in LED.
 * Echoes the LED state back so the host can confirm it changed.
 *
 * Demonstrates the simplest possible ArduinoModule:
 *   - One input stream  (Bool → LED on/off)
 *   - One output stream  (Bool → confirm LED state)
 *   - DimosSerial debug prints
 *   - Subscribe/callback API matching C++ LCM
 */

#include "dimos_arduino.h"
#include <util/delay.h>

#define LED_PIN 13

void on_led_cmd(const char *channel, const void *msg, void *ctx) {
    (void)channel;
    (void)ctx;
    const dimos_msg__Bool *cmd = (const dimos_msg__Bool *)msg;

    /* Set the LED */
    digitalWrite(LED_PIN, cmd->data ? HIGH : LOW);
    DimosSerial.print("LED ");
    DimosSerial.println(cmd->data ? "ON" : "OFF");

    /* Echo back the state */
    uint8_t buf[1];
    int encoded = dimos_msg__Bool__encode(buf, 0, sizeof(buf), cmd);
    if (encoded > 0) {
        dimos_publish(DIMOS_CHANNEL__LED_STATE,
                      &dimos_msg__Bool__type, buf, encoded);
    }
}

void setup() {
    dimos_init(DIMOS_BAUDRATE);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    dimos_subscribe(DIMOS_CHANNEL__LED_CMD,
                    &dimos_msg__Bool__type, on_led_cmd, NULL);
    DimosSerial.println("LED Echo ready");
}

void loop() {
    dimos_handle(10);
    _delay_ms(1);
}
