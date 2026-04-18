/*
 * main_esp32.ino
 * ==============
 * Role: Main ESP32 — SPI SLAVE to Main ATmega328PB, ESP-NOW hub to Sub ESP32s
 *
 * Flow (sequential, single-cycle):
 *   1. Queue trans1: wait to receive [LEN][CMD_START_COLL] from ATmega.
 *   2. Validate received command.
 *   3. Broadcast CMD_START_COLL via ESP-NOW to all sub ESP32s.
 *   4. Wait COLLECT_TIMEOUT ms for sub ESP32 replies.
 *   5. Print collected robot addresses.
 *   6. Queue trans2: send [LEN][ADDR0][ADDR1...] back to ATmega.
 *   7. Repeat.
 *
 * NOTE: All initialization is inside loop() behind a static-bool guard.
 *       setup() is intentionally empty.
 *
 * SPI pins (VSPI / SPI3, slave to Main ATmega):
 *   SCK  = GPIO18   MISO = GPIO33   MOSI = GPIO17   SS = GPIO5
 */

#include "driver/spi_slave.h"
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

/* ── Protocol constants (must match sub firmware and ATmega) ─────────── */
#define CMD_START_COLL 0xA1
#define PKT_CMD        0xA1
#define PKT_ADDR       0xA2

/* ── Timing ──────────────────────────────────────────────────────────── */
#define MAX_ROBOTS      2
#define COLLECT_TIMEOUT 150 /* ms: wait window for sub ESP32 replies */

/* ── SPI buffer size ─────────────────────────────────────────────────── */
/* 1 length byte + up to MAX_ROBOTS payload bytes + spare */
#define SPI_BUF_SIZE 66

/* ── ESP-NOW packet ──────────────────────────────────────────────────── */
struct EspPacket {
    uint8_t type;       /* PKT_CMD or PKT_ADDR */
    uint8_t robot_addr; /* valid for PKT_ADDR only */
    uint8_t pad[2];     /* align to 4 bytes */
};

/* ── Globals shared between ESP-NOW callback and loop() ──────────────── */
volatile uint8_t collected[MAX_ROBOTS];
volatile uint8_t collected_count = 0;

/* ── ESP-NOW receive callback (runs on WiFi task) ────────────────────── */
void onEspNowReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    if (len < (int)sizeof(EspPacket)) return;
    const EspPacket *pkt = (const EspPacket *)data;
    if (pkt->type != PKT_ADDR) return;
    if (collected_count < MAX_ROBOTS)
        collected[collected_count++] = pkt->robot_addr;
}

/* ── setup() intentionally empty ────────────────────────────────────── */
void setup() {}

/* ── Main loop ───────────────────────────────────────────────────────── */
void loop() {

    /* ── One-time initialization ── */
    static bool     initialized = false;
    static uint8_t *rx1         = nullptr;
    static uint8_t *tx1         = nullptr;
    static uint8_t *rx2         = nullptr;
    static uint8_t *tx2         = nullptr;
    static const uint8_t broadcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

    if (!initialized) {
        Serial.begin(115200);

        /* WiFi in station mode (required for ESP-NOW) */
        WiFi.mode(WIFI_STA);
        WiFi.disconnect();

        if (esp_now_init() != ESP_OK) {
            Serial.println("[MAIN ESP32] ERROR: esp_now_init failed");
            while (true) delay(1000);
        }
        esp_now_register_recv_cb(onEspNowReceive);

        /* Register broadcast peer */
        esp_now_peer_info_t peer = {};
        memcpy(peer.peer_addr, broadcast, 6);
        peer.channel = 0;
        peer.encrypt = false;
        if (esp_now_add_peer(&peer) != ESP_OK) {
            Serial.println("[MAIN ESP32] ERROR: esp_now_add_peer failed");
            while (true) delay(1000);
        }

        /* Allocate DMA-capable SPI buffers (ESP-IDF requirement) */
        rx1 = (uint8_t *)heap_caps_malloc(SPI_BUF_SIZE, MALLOC_CAP_DMA);
        tx1 = (uint8_t *)heap_caps_malloc(SPI_BUF_SIZE, MALLOC_CAP_DMA);
        rx2 = (uint8_t *)heap_caps_malloc(SPI_BUF_SIZE, MALLOC_CAP_DMA);
        tx2 = (uint8_t *)heap_caps_malloc(SPI_BUF_SIZE, MALLOC_CAP_DMA);
        if (!rx1 || !tx1 || !rx2 || !tx2) {
            Serial.println("[MAIN ESP32] ERROR: DMA malloc failed");
            while (true) delay(1000);
        }

        /* Initialize SPI3 (VSPI) as slave */
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num   = 17;
        buscfg.miso_io_num   = 33;
        buscfg.sclk_io_num   = 18;
        buscfg.quadwp_io_num = -1;
        buscfg.quadhd_io_num = -1;

        spi_slave_interface_config_t slvcfg = {};
        slvcfg.mode         = 0;   /* SPI Mode 0 — matches ATmega */
        slvcfg.spics_io_num = 5;   /* SS = GPIO5 */
        slvcfg.queue_size   = 2;
        slvcfg.flags        = 0;

        if (spi_slave_initialize(SPI3_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO) != ESP_OK) {
            Serial.println("[MAIN ESP32] ERROR: spi_slave_initialize failed");
            while (true) delay(1000);
        }

        initialized = true;
        Serial.println("[MAIN ESP32] Ready. MAC: " + WiFi.macAddress());
    }

    /* ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
     * STEP 1 — Receive CMD_START_COLL from ATmega via SPI
     * Protocol: ATmega sends [0x01][0xA1] (length=1, cmd=CMD_START_COLL)
     * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ */
    memset(tx1, 0xFF, SPI_BUF_SIZE); /* MISO don't-care while receiving */
    memset(rx1, 0x00, SPI_BUF_SIZE);

    spi_slave_transaction_t trans1 = {};
    trans1.length    = SPI_BUF_SIZE * 8;
    trans1.tx_buffer = tx1;
    trans1.rx_buffer = rx1;

    spi_slave_queue_trans(SPI3_HOST, &trans1, portMAX_DELAY);

    spi_slave_transaction_t *ret1;
    spi_slave_get_trans_result(SPI3_HOST, &ret1, portMAX_DELAY);

    /* Validate: rx1[0] = length byte (expect 1), rx1[1] = command byte */
    if (rx1[0] != 1 || rx1[1] != CMD_START_COLL) {
        Serial.printf("[MAIN ESP32] Unexpected packet: len=0x%02X cmd=0x%02X — ignoring\n",
                      rx1[0], rx1[1]);
        return;
    }
    Serial.println("[MAIN ESP32] CMD_START_COLL received from ATmega");

    /* ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
     * STEP 2 — Broadcast CMD_START_COLL to all sub ESP32s via ESP-NOW
     * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ */
    collected_count = 0;
    memset((void *)collected, 0, sizeof(collected));

    EspPacket cmd_pkt;
    cmd_pkt.type       = PKT_CMD;
    cmd_pkt.robot_addr = 0;
    cmd_pkt.pad[0]     = 0;
    cmd_pkt.pad[1]     = 0;
    esp_now_send(broadcast, (uint8_t *)&cmd_pkt, sizeof(cmd_pkt));
    Serial.println("[MAIN ESP32] CMD_START_COLL broadcast via ESP-NOW");

    /* ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
     * STEP 3 — Wait for sub ESP32 replies
     * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ */
    delay(COLLECT_TIMEOUT);

    /* Snapshot volatile data once */
    uint8_t snap_count = collected_count;
    uint8_t snap_addrs[MAX_ROBOTS];
    memcpy(snap_addrs, (const void *)collected, snap_count);

    /* ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
     * STEP 4 — Print collected robot addresses
     * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ */
    Serial.printf("[MAIN ESP32] Collected %u robot(s):", snap_count);
    for (uint8_t i = 0; i < snap_count; i++) {
        Serial.printf("  [%u] = 0x%02X", i, snap_addrs[i]);
    }
    Serial.println();

    /* ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
     * STEP 5 — Send collected addresses back to ATmega via SPI
     * Protocol: [LEN][ADDR0][ADDR1...] — ATmega's spi_receive_response()
     *           reads the length byte first, then that many address bytes.
     * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ */
    tx2[0] = snap_count;
    memcpy(tx2 + 1, snap_addrs, snap_count);
    memset(tx2 + 1 + snap_count, 0xFF, SPI_BUF_SIZE - 1 - snap_count); /* pad remainder */
    memset(rx2, 0x00, SPI_BUF_SIZE);

    spi_slave_transaction_t trans2 = {};
    trans2.length    = SPI_BUF_SIZE * 8;
    trans2.tx_buffer = tx2;
    trans2.rx_buffer = rx2;

    spi_slave_queue_trans(SPI3_HOST, &trans2, portMAX_DELAY);

    spi_slave_transaction_t *ret2;
    spi_slave_get_trans_result(SPI3_HOST, &ret2, portMAX_DELAY);

    Serial.println("[MAIN ESP32] Response sent to ATmega — cycle complete\n");
}