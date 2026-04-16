/*
 * spi-test-esp32.ino
 * ==================
 * SPI MASTER side of the ATmega <-> ESP32 loopback test.
 *
 * Sends [LEN][DATA...] to the ATmega slave, waits a guard delay,
 * then clocks back the ATmega's echo and prints PASS/FAIL.
 *
 * Wiring (VSPI):
 *   ESP32 GPIO18 (SCK)  -> ATmega PB5 (SCK)
 *   ESP32 GPIO33 (MISO) -> ATmega PB4 (MISO)
 *   ESP32 GPIO17 (MOSI) -> ATmega PB3 (MOSI)
 *   ESP32 GPIO5  (SS)   -> ATmega PB2 (SS)
 *   GND <-> GND
 *
 * Frame format (matches spi_comm.c):
 *   TX transaction:  CS_low -> [LEN][DATA...] -> CS_high
 *   <300 µs guard>
 *   RX transaction:  CS_low -> [LEN][DATA...] (clocked in) -> CS_high
 *
 * NOTE: All init is in loop() behind a static-bool guard.
 *       setup() is intentionally left empty.
 */

#include <Arduino.h>
#include <SPI.h>

#define PIN_SCK  18
#define PIN_MISO 33
#define PIN_MOSI 17
#define PIN_SS   5

#define SPI_FREQ      1000000
#define GUARD_US      300
#define TEST_DELAY_MS 1000
#define SPI_MAX_PAYLOAD 64

static const uint8_t TEST_MSG[] = {0xA1, 0xB2, 0xC3, 0xD4};
static const uint8_t TEST_LEN   = sizeof(TEST_MSG);

void setup() {}

void loop() {
    static bool initialized = false;
    static uint32_t round = 0;

    if (!initialized) {
        Serial.begin(115200);
        while (!Serial)
            delay(10);

        SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_SS);
        pinMode(PIN_SS, OUTPUT);
        digitalWrite(PIN_SS, HIGH);

        Serial.println("[ESP32 SPI MASTER] Ready");
        initialized = true;
    }

    delay(TEST_DELAY_MS);
    round++;

    /* ── TX transaction: send [LEN][DATA...] ── */
    SPI.beginTransaction(SPISettings(SPI_FREQ, MSBFIRST, SPI_MODE0));
    digitalWrite(PIN_SS, LOW);
    SPI.transfer(TEST_LEN);
    for (uint8_t i = 0; i < TEST_LEN; i++)
        SPI.transfer(TEST_MSG[i]);
    digitalWrite(PIN_SS, HIGH);
    SPI.endTransaction();

    /* ── Guard delay: ATmega prepares its echo ── */
    delayMicroseconds(GUARD_US);

    /* ── RX transaction: clock out ATmega's echo ── */
    uint8_t rx_buf[SPI_MAX_PAYLOAD];
    memset(rx_buf, 0, sizeof(rx_buf));
    SPI.beginTransaction(SPISettings(SPI_FREQ, MSBFIRST, SPI_MODE0));
    digitalWrite(PIN_SS, LOW);
    uint8_t resp_len = SPI.transfer(0x00);
    if (resp_len <= SPI_MAX_PAYLOAD) {
        for (uint8_t i = 0; i < resp_len; i++)
            rx_buf[i] = SPI.transfer(0x00);
    }
    digitalWrite(PIN_SS, HIGH);
    SPI.endTransaction();

    /* ── Verify echo ── */
    Serial.printf("[ESP32] Round %lu — sent %u bytes:", round, TEST_LEN);
    for (uint8_t i = 0; i < TEST_LEN; i++)
        Serial.printf(" 0x%02X", TEST_MSG[i]);
    Serial.println();

    if (resp_len != TEST_LEN) {
        Serial.printf("[ESP32] FAIL — bad resp_len=%u (expected %u)\n",
                      resp_len, TEST_LEN);
        return;
    }

    bool ok = true;
    for (uint8_t i = 0; i < resp_len; i++) {
        if (rx_buf[i] != TEST_MSG[i]) {
            ok = false;
            Serial.printf("[ESP32] FAIL — byte[%u]: got 0x%02X expected 0x%02X\n",
                          i, rx_buf[i], TEST_MSG[i]);
        }
    }
    if (ok)
        Serial.printf("[ESP32] PASS — echo matched (%u bytes)\n", resp_len);
}
