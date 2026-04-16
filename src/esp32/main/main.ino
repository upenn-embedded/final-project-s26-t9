/*
 * main_esp32.ino
 * ==============
 * Role: Main ESP32 — SPI SLAVE to Main ATmega328PB, ESP-NOW hub to Sub ESP32s
 *
 * Flow (one-cycle lag design):
 *   1. Pre-queue two SPI slave transactions:
 *        trans1: receive [1][CMD_START_COLL] from ATmega (tx don't-care)
 *        trans2: send [prev_len][prev_addr0][prev_addr1...] to ATmega
 *      Both are queued into the hardware before NSS asserts, so the 200 µs
 *      ATmega guard window is handled entirely by hardware with no CPU delay.
 *   2. Wait for both transactions to complete.
 *   3. Broadcast CMD_START_COLL via ESP-NOW to all sub ESP32s.
 *   4. Wait COLLECT_TIMEOUT ms for sub ESP32s to reply.
 *   5. Save collected addresses as "prev" for the next cycle's trans2.
 *   6. Repeat.
 *
 * NOTE: All initialization is inside loop() behind a static-bool guard.
 *       setup() is intentionally empty — it does not work reliably in this
 *       project's build environment.
 *
 * SPI pins (VSPI / SPI3, slave to Main ATmega):
 *   SCK  = GPIO18   MISO = GPIO19   MOSI = GPIO23   SS = GPIO5
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include "driver/spi_slave.h"

/* ── Protocol constants (must match sub firmware and ATmega) ─────────── */
#define CMD_START_COLL  0xA1
#define PKT_CMD         0xA1
#define PKT_ADDR        0xA2

/* ── Timing ──────────────────────────────────────────────────────────── */
#define MAX_ROBOTS      2
#define TIME_PER_ESP    50    /* ms stagger per robot index */
#define COLLECT_TIMEOUT 150   /* ms: (MAX_ROBOTS-1)*TIME_PER_ESP + 100 margin */

/* ── SPI buffer size ─────────────────────────────────────────────────── */
/* 1 length byte + up to 64 payload bytes + 1 spare */
#define SPI_BUF_SIZE    66

/* ── ESP-NOW packet ──────────────────────────────────────────────────── */
struct EspPacket {
    uint8_t type;        /* PKT_CMD or PKT_ADDR */
    uint8_t robot_addr;  /* valid for PKT_ADDR only */
    uint8_t pad[2];      /* align to 4 bytes */
};

/* ── Globals shared between ESP-NOW callback and loop() ──────────────── */
volatile uint8_t collected[MAX_ROBOTS];
volatile uint8_t collected_count = 0;

/* ── ESP-NOW receive callback (runs on WiFi task) ────────────────────── */
void onEspNowReceive(const uint8_t *mac, const uint8_t *data, int len)
{
    if (len < (int)sizeof(EspPacket)) return;
    const EspPacket *pkt = (const EspPacket *)data;
    if (pkt->type != PKT_ADDR) return;
    if (collected_count < MAX_ROBOTS)
        collected[collected_count++] = pkt->robot_addr;
}

/* ── setup() intentionally empty ────────────────────────────────────── */
void setup() {}

/* ── Main loop ───────────────────────────────────────────────────────── */
void loop()
{
    /* ── One-time initialization ── */
    static bool      initialized = false;
    static uint8_t  *rx1         = nullptr;
    static uint8_t  *tx1         = nullptr;
    static uint8_t  *rx2         = nullptr;
    static uint8_t  *tx2         = nullptr;
    static uint8_t   prev_response[MAX_ROBOTS];
    static uint8_t   prev_len    = 0;
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
        memset(tx1, 0xFF, SPI_BUF_SIZE);
        memset(tx2, 0x00, SPI_BUF_SIZE);
        memset(prev_response, 0x00, sizeof(prev_response));

        /* Initialize SPI3 (VSPI) as slave */
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num   = 23;
        buscfg.miso_io_num   = 19;
        buscfg.sclk_io_num   = 18;
        buscfg.quadwp_io_num = -1;
        buscfg.quadhd_io_num = -1;

        spi_slave_interface_config_t slvcfg = {};
        slvcfg.mode        = 0;      /* SPI Mode 0 — matches ATmega */
        slvcfg.spics_io_num = 5;     /* SS = GPIO5 */
        slvcfg.queue_size  = 2;      /* hold both transactions at once */
        slvcfg.flags       = 0;

        if (spi_slave_initialize(SPI3_HOST, &buscfg, &slvcfg,
                                 SPI_DMA_CH_AUTO) != ESP_OK) {
            Serial.println("[MAIN ESP32] ERROR: spi_slave_initialize failed");
            while (true) delay(1000);
        }

        initialized = true;
        Serial.println("[MAIN ESP32] Ready. MAC: " + WiFi.macAddress());
    }

    /* ── Build trans1: receive CMD from ATmega ── */
    memset(tx1, 0xFF, SPI_BUF_SIZE);  /* don't-care MISO during CMD receive */
    memset(rx1, 0x00, SPI_BUF_SIZE);

    spi_slave_transaction_t trans1 = {};
    trans1.length    = SPI_BUF_SIZE * 8;  /* max; actual length set by NSS */
    trans1.tx_buffer = tx1;
    trans1.rx_buffer = rx1;

    /* ── Build trans2: send previous cycle's addresses to ATmega ── */
    /*
     * Protocol: ATmega's spi_receive_response() reads [LEN][DATA...].
     * tx2[0] = length, tx2[1..prev_len] = robot addresses.
     */
    tx2[0] = prev_len;
    memcpy(tx2 + 1, prev_response, prev_len);
    memset(tx2 + 1 + prev_len, 0xFF, SPI_BUF_SIZE - 1 - prev_len);
    memset(rx2, 0x00, SPI_BUF_SIZE);

    spi_slave_transaction_t trans2 = {};
    trans2.length    = SPI_BUF_SIZE * 8;
    trans2.tx_buffer = tx2;
    trans2.rx_buffer = rx2;

    /*
     * Pre-queue both transactions. The hardware fires trans1 on the first
     * NSS assertion, then trans2 on the second NSS assertion (after the
     * 200 µs ATmega guard delay), with no CPU involvement between them.
     */
    spi_slave_queue_trans(SPI3_HOST, &trans1, portMAX_DELAY);
    spi_slave_queue_trans(SPI3_HOST, &trans2, portMAX_DELAY);

    spi_slave_transaction_t *ret;
    spi_slave_get_trans_result(SPI3_HOST, &ret, portMAX_DELAY); /* trans1 */
    spi_slave_get_trans_result(SPI3_HOST, &ret, portMAX_DELAY); /* trans2 */

    /* ── Validate received command ── */
    /* rx1[0] = length byte (should be 1), rx1[1] = command byte */
    if (rx1[0] != 1 || rx1[1] != CMD_START_COLL) {
        Serial.printf("[MAIN ESP32] Unexpected: len=0x%02X cmd=0x%02X\n",
                      rx1[0], rx1[1]);
        return;
    }

    /* ── Broadcast CMD via ESP-NOW to all sub ESP32s ── */
    collected_count = 0;
    memset((void *)collected, 0, sizeof(collected));

    EspPacket cmd_pkt;
    cmd_pkt.type       = PKT_CMD;
    cmd_pkt.robot_addr = 0;
    cmd_pkt.pad[0]     = 0;
    cmd_pkt.pad[1]     = 0;
    esp_now_send(broadcast, (uint8_t *)&cmd_pkt, sizeof(cmd_pkt));

    /* ── Wait for sub ESP32s to reply ── */
    delay(COLLECT_TIMEOUT);

    /* ── Store results for next cycle's trans2 ── */
    prev_len = collected_count;
    memcpy(prev_response, (const void *)collected, collected_count);

    Serial.printf("[MAIN ESP32] Cycle complete — collected %u robot(s)\n",
                  collected_count);
}
