/*
 * esp32_spi_master.ino
 * ====================
 * ESP32 SPI master — sends a test payload to ATmega slave every second
 * and prints each round's result over Serial.
 *
 * Wiring (VSPI):
 *   ESP32 GPIO18 (SCK)  -> ATmega PB5 (SCK)
 *   ESP32 GPIO19 (MISO) -> ATmega PB4 (MISO)
 *   ESP32 GPIO23 (MOSI) -> ATmega PB3 (MOSI)
 *   ESP32 GPIO5  (SS)   -> ATmega PB2 (SS)
 *   GND <-> GND
 *
 * Frame format:
 *   CS_low -> [LEN] [DATA...] -> CS_high
 */

#include <Arduino.h>
#include <SPI.h>

#define PIN_SCK  18
#define PIN_MISO 19
#define PIN_MOSI 23
#define PIN_SS   5

#define SPI_FREQ      500000
#define TEST_DELAY_MS 1000

static const uint8_t TEST_MSG[] = {0xA1, 0xB2, 0xC3, 0xD4};
static const uint8_t TEST_LEN   = sizeof(TEST_MSG);

void setup() {}

void loop() {
    static bool     initialized = false;
    static uint32_t round       = 0;

    if (!initialized) {
        Serial.begin(115200);
        while (!Serial)
            delay(10);

        SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_SS);
        pinMode(PIN_SS, OUTPUT);
        digitalWrite(PIN_SS, HIGH);

        Serial.println("[ESP32] SPI master ready");
        initialized = true;
    }

    delay(TEST_DELAY_MS);
    round++;

    SPI.beginTransaction(SPISettings(SPI_FREQ, MSBFIRST, SPI_MODE0));
    digitalWrite(PIN_SS, LOW);
    SPI.transfer(TEST_LEN);
    for (uint8_t i = 0; i < TEST_LEN; i++)
        SPI.transfer(TEST_MSG[i]);
    digitalWrite(PIN_SS, HIGH);
    SPI.endTransaction();

    Serial.printf("[ESP32] Round %lu — sent %u bytes:", round, TEST_LEN);
    for (uint8_t i = 0; i < TEST_LEN; i++)
        Serial.printf(" 0x%02X", TEST_MSG[i]);
    Serial.println();
}
