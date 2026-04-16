/*
 * ESP32 SPI RX Test
 * Clocks out a dummy 0x00 every second to read a byte from the ATmega slave.
 * Expected: ATmega sends an incrementing counter (0x00, 0x01, 0x02, ...).
 *
 * Wiring (VSPI):
 *   ESP32 GPIO18 (SCK)  -> ATmega PB5 (SCK)
 *   ESP32 GPIO33 (MISO) -> ATmega PB4 (MISO)
 *   ESP32 GPIO17 (MOSI) -> ATmega PB3 (MOSI)
 *   ESP32 GPIO5  (SS)   -> ATmega PB2 (SS)
 *   GND <-> GND
 */

#include <SPI.h>

#define SCK_PIN  18
#define MISO_PIN 33
#define MOSI_PIN 17
#define SS_PIN    5

void setup() {
    // intentionally empty
}

void loop() {
    static bool initialized = false;

    if (!initialized) {
        Serial.begin(115200);
        while (!Serial) {}

        SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, SS_PIN);
        pinMode(SS_PIN, OUTPUT);
        digitalWrite(SS_PIN, HIGH);

        initialized = true;
        Serial.println("ESP32 SPI RX test ready");
    }

    uint8_t rx;

    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(SS_PIN, LOW);
    rx = SPI.transfer(0x00);   // dummy byte — just clocks out ATmega's SPDR
    digitalWrite(SS_PIN, HIGH);
    SPI.endTransaction();

    Serial.printf("Received from ATmega: 0x%02X\n", rx);

    delay(1000);
}
