#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <esp_now.h>

#define CMD_START_COLL 0xA1
#define CMD_MOVE_DATA  0xB1
#define PKT_CMD        0xA1
#define PKT_ADDR       0xA2
#define PKT_MOVE       0xB1

#define NUM_ROBOTS       4
#define NUM_PHT          6
#define DATA_LEN         (NUM_ROBOTS * NUM_PHT * 2) /* 24 bytes */
#define MOVE_PAYLOAD_LEN (NUM_ROBOTS * 4)           /* 8 bytes */

#ifndef ROBOT_ADDRESS
#define ROBOT_ADDRESS 1
#endif

#define TIME_PER_ESP 50
#define SS_PIN       5
#define SPI_FREQ     1000000
#define SPI_GUARD_MS 50

static const SPISettings spi_cfg(SPI_FREQ, MSBFIRST, SPI_MODE0);

struct EspDataPacket {
    uint8_t type;
    uint8_t robot_addr;
    uint8_t pad[2];
    uint8_t data[DATA_LEN];
};

struct EspMovePacket {
    uint8_t type;
    uint8_t pad[3];
    uint8_t data[MOVE_PAYLOAD_LEN];
};

/* ── Flags and buffers set by ESP-NOW callback ───────────────────────── */
volatile bool cmd_received = false;
volatile bool move_received = false;
static uint8_t pending_move[MOVE_PAYLOAD_LEN];

void
onEspNowReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    if (len < 4)
        return;
    uint8_t type = data[0];

    if (type == PKT_CMD) {
        cmd_received = true;
        return;
    }

    if (type == PKT_MOVE && len >= (int) sizeof(EspMovePacket)) {
        const EspMovePacket *pkt = (const EspMovePacket *) data;
        memcpy((void *) pending_move, pkt->data, MOVE_PAYLOAD_LEN);
        move_received = true;
    }
}

void
setup() {
    static const uint8_t broadcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    Serial.begin(115200);

    SPI.begin();
    pinMode(SS_PIN, OUTPUT);
    digitalWrite(SS_PIN, HIGH);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    if (esp_now_init() != ESP_OK) {
        Serial.println("[SUB ESP32] ERROR: esp_now_init failed");
        while (true)
            delay(1000);
    }
    esp_now_register_recv_cb(onEspNowReceive);

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, broadcast, 6);
    peer.channel = 0;
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) != ESP_OK) {
        Serial.println("[SUB ESP32] ERROR: esp_now_add_peer failed");
        while (true)
            delay(1000);
    }

    delay(100);
    Serial.println("[SUB ESP32] Ready. MAC: " + WiFi.macAddress());
}

void
loop() {
    static const uint8_t broadcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

    /* ── Handle CMD_START_COLL ── */
    if (cmd_received) {
        cmd_received = false;
        delay(5);

        /* TX: send [LEN=1][CMD_START_COLL] to sub ATmega */
        portDISABLE_INTERRUPTS();
        SPI.beginTransaction(spi_cfg);
        digitalWrite(SS_PIN, LOW);
        SPI.transfer(1);
        SPI.transfer(CMD_START_COLL);
        digitalWrite(SS_PIN, HIGH);
        SPI.endTransaction();
        portENABLE_INTERRUPTS();

        delay(SPI_GUARD_MS);

        /* RX: clock in [LEN][DATA_LEN bytes] from sub ATmega */
        uint8_t rx_data[DATA_LEN];
        portDISABLE_INTERRUPTS();
        SPI.beginTransaction(spi_cfg);
        digitalWrite(SS_PIN, LOW);
        uint8_t len = SPI.transfer(0x00);
        for (uint8_t i = 0; i < DATA_LEN; i++)
            rx_data[i] = SPI.transfer(0x00);
        digitalWrite(SS_PIN, HIGH);
        SPI.endTransaction();
        portENABLE_INTERRUPTS();

        cmd_received = false;

        if (len != DATA_LEN) {
            Serial.printf("[SUB ESP32] Bad resp_len=%u (expected %u) — skipping\n", len, (unsigned) DATA_LEN);
            return;
        }

        Serial.printf("[SUB ESP32] Got %u bytes, staggering %u ms\n", len, (unsigned) (ROBOT_ADDRESS * TIME_PER_ESP));

        delay((uint32_t) ROBOT_ADDRESS * TIME_PER_ESP);

        EspDataPacket resp;
        resp.type = PKT_ADDR;
        resp.robot_addr = ROBOT_ADDRESS;
        resp.pad[0] = 0;
        resp.pad[1] = 0;
        memcpy(resp.data, rx_data, DATA_LEN);
        esp_now_send(broadcast, (uint8_t *) &resp, sizeof(resp));
        return;
    }

    /* ── Handle CMD_MOVE_DATA ── */
    if (move_received) {
        move_received = false;

        uint8_t move_snap[MOVE_PAYLOAD_LEN];
        memcpy(move_snap, (const void *) pending_move, MOVE_PAYLOAD_LEN);

        /*
         * Frame must match spi_slave_receive() expectation: [LEN][DATA...]
         * LEN = 1 (cmd byte) + MOVE_PAYLOAD_LEN (8) = 9
         * spi_slave_receive strips the length prefix, leaving:
         *   rx_buf[0]   = CMD_MOVE_DATA
         *   rx_buf[1..] = move payload bytes
         */
        uint8_t spi_buf[1 + 1 + MOVE_PAYLOAD_LEN]; /* [LEN][CMD][data...] */
        spi_buf[0] = 1 + MOVE_PAYLOAD_LEN;         /* length prefix       */
        spi_buf[1] = CMD_MOVE_DATA;
        memcpy(spi_buf + 2, move_snap, MOVE_PAYLOAD_LEN);

        portDISABLE_INTERRUPTS();
        SPI.beginTransaction(spi_cfg);
        digitalWrite(SS_PIN, LOW);
        for (uint8_t i = 0; i < sizeof(spi_buf); i++)
            SPI.transfer(spi_buf[i]);
        digitalWrite(SS_PIN, HIGH);
        SPI.endTransaction();
        portENABLE_INTERRUPTS();

        Serial.printf("[SUB ESP32] MOVE_DATA sent to ATmega\n");
    }
}