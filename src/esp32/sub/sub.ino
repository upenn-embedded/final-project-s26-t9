/*
 * sub_esp32.ino
 * =============
 * Role: Sub ESP32 — ESP-NOW endpoint from Main ESP32, SPI MASTER to Sub ATmega
 *
 * Flow:
 *   1. Wait to receive a PKT_CMD packet via ESP-NOW.
 *   2. SPI Transaction 1: send [1][CMD_START_COLL] to Sub ATmega (as master).
 *   3. SPI Transaction 2: clock out Sub ATmega's response [1][ROBOT_ADDRESS].
 *   4. Stagger: delay ROBOT_ADDRESS * TIME_PER_ESP ms to avoid ESP-NOW collisions.
 *   5. Broadcast PKT_ADDR with ROBOT_ADDRESS via ESP-NOW.
 *   6. Repeat.
 *
 * NOTE: All initialization is inside loop() behind a static-bool guard.
 *       setup() is intentionally empty — it does not work reliably in this
 *       project's build environment.
 *
 * SPI pins (VSPI, master to Sub ATmega):
 *   SCK  = GPIO18   MISO = GPIO19   MOSI = GPIO23   SS = GPIO5
 *
 * The Sub ATmega uses two separate NSS transactions (see spi_comm.c):
 *   TX transaction: [LEN][CMD_START_COLL]   (master drives)
 *   <300 µs guard: ATmega pre-loads ROBOT_ADDRESS into SPDR>
 *   RX transaction: [LEN][ROBOT_ADDRESS]    (slave drives)
 */

#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <esp_now.h>

/* ── Protocol constants (must match main ESP32 and ATmega) ───────────── */
#define CMD_START_COLL 0xA1
#define PKT_CMD        0xA1
#define PKT_ADDR       0xA2

/* ── Timing ──────────────────────────────────────────────────────────── */
#define TIME_PER_ESP 50 /* ms stagger per robot index */

/* ── SPI pin assignments ─────────────────────────────────────────────── */
#define PIN_SCK  18
#define PIN_MISO 33
#define PIN_MOSI 17
#define PIN_SS   5

/* ── ESP-NOW packet ──────────────────────────────────────────────────── */
struct EspPacket {
    uint8_t type;       /* PKT_CMD or PKT_ADDR */
    uint8_t robot_addr; /* valid for PKT_ADDR only */
    uint8_t pad[2];     /* align to 4 bytes */
};

/* ── Flag set by ESP-NOW callback, consumed in loop() ────────────────── */
volatile bool cmd_received = false;

/* ── ESP-NOW receive callback ────────────────────────────────────────── */
void
onEspNowReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    if (len < (int) sizeof(EspPacket))
        return;
    const EspPacket *pkt = (const EspPacket *) data;
    if (pkt->type == PKT_CMD)
        cmd_received = true;
}

/* ── setup() intentionally empty ────────────────────────────────────── */
void
setup() {}

/* ── Main loop ───────────────────────────────────────────────────────── */
void
loop() {
    /* ── One-time initialization ── */
    static bool initialized = false;
    static const uint8_t broadcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

    if (!initialized) {
        Serial.begin(115200);

        /* WiFi in station mode (required for ESP-NOW) */
        WiFi.mode(WIFI_STA);
        WiFi.disconnect();

        if (esp_now_init() != ESP_OK) {
            Serial.println("[SUB ESP32] ERROR: esp_now_init failed");
            while (true)
                delay(1000);
        }
        esp_now_register_recv_cb(onEspNowReceive);

        /* Register broadcast peer for sending responses */
        esp_now_peer_info_t peer = {};
        memcpy(peer.peer_addr, broadcast, 6);
        peer.channel = 0;
        peer.encrypt = false;
        if (esp_now_add_peer(&peer) != ESP_OK) {
            Serial.println("[SUB ESP32] ERROR: esp_now_add_peer failed");
            while (true)
                delay(1000);
        }

        /* SPI master to Sub ATmega */
        SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_SS);
        pinMode(PIN_SS, OUTPUT);
        digitalWrite(PIN_SS, HIGH); /* SS idle high */

        initialized = true;
        Serial.println("[SUB ESP32] Ready. MAC: " + WiFi.macAddress());
    }

    /* ── Spin until CMD received via ESP-NOW ── */
    if (!cmd_received)
        return;
    cmd_received = false;

    /*
     * ── SPI Transaction 1: send [1][CMD_START_COLL] to Sub ATmega ──
     *
     * Sub ATmega spi_slave_receive() is waiting for this.
     * Frame: [length=1][CMD_START_COLL]
     */
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(PIN_SS, LOW);
    SPI.transfer(1);              /* length prefix */
    SPI.transfer(CMD_START_COLL); /* command byte  */
    digitalWrite(PIN_SS, HIGH);
    SPI.endTransaction();

    /*
     * Guard delay: Sub ATmega needs time after its spi_slave_receive()
     * completes to call spi_slave_send() and pre-load SPDR with the
     * length byte.  ATmega's own guard is 200 µs; 300 µs gives margin.
     */
    delayMicroseconds(300);

    /*
     * ── SPI Transaction 2: clock out [len][ROBOT_ADDRESS] from Sub ATmega ──
     *
     * Sub ATmega spi_slave_send() has pre-loaded SPDR = length (1),
     * then shifts out ROBOT_ADDRESS on the following byte.
     */
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(PIN_SS, LOW);
    uint8_t resp_len = SPI.transfer(0x00);   /* ATmega sends length byte (=1) */
    uint8_t robot_addr = SPI.transfer(0x00); /* ATmega sends ROBOT_ADDRESS    */
    digitalWrite(PIN_SS, HIGH);
    SPI.endTransaction();

    if (resp_len != 1) {
        Serial.printf("[SUB ESP32] Bad resp_len=%u — skipping\n", resp_len);
        return;
    }

    Serial.printf("[SUB ESP32] Got address=%u, staggering %u ms\n", robot_addr, (unsigned) (robot_addr * TIME_PER_ESP));

    /* ── Staggered delay: robot N waits N*TIME_PER_ESP ms ── */
    delay((uint32_t) robot_addr * TIME_PER_ESP);

    /* ── Broadcast address back via ESP-NOW ── */
    EspPacket resp;
    resp.type = PKT_ADDR;
    resp.robot_addr = robot_addr;
    resp.pad[0] = 0;
    resp.pad[1] = 0;
    esp_now_send(broadcast, (uint8_t *) &resp, sizeof(resp));
}
