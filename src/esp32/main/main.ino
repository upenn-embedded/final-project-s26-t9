#include "driver/spi_slave.h"
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

#define CMD_START_COLL 0xA1
#define PKT_CMD        0xA1
#define PKT_ADDR       0xA2

#define MAX_ROBOTS      2
#define COLLECT_TIMEOUT 150 

#define SPI_BUF_SIZE 66

struct EspPacket {
    uint8_t type;
    uint8_t robot_addr;
    uint8_t pad[2]; 
};

volatile uint8_t collected[MAX_ROBOTS];
volatile uint8_t collected_count = 0;

void onEspNowReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    if (len < (int)sizeof(EspPacket)) return;
    const EspPacket *pkt = (const EspPacket *)data;
    if (pkt->type != PKT_ADDR) return;
    if (collected_count < MAX_ROBOTS)
        collected[collected_count++] = pkt->robot_addr;
}

void setup() {}

void loop() {

    static bool     initialized = false;
    static uint8_t *rx1         = nullptr;
    static uint8_t *tx1         = nullptr;
    static uint8_t *rx2         = nullptr;
    static uint8_t *tx2         = nullptr;
    static const uint8_t broadcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

    if (!initialized) {
        Serial.begin(115200);

        WiFi.mode(WIFI_STA);
        WiFi.disconnect();

        if (esp_now_init() != ESP_OK) {
            Serial.println("[MAIN ESP32] ERROR: esp_now_init failed");
            while (true) delay(1000);
        }
        esp_now_register_recv_cb(onEspNowReceive);

        esp_now_peer_info_t peer = {};
        memcpy(peer.peer_addr, broadcast, 6);
        peer.channel = 0;
        peer.encrypt = false;
        if (esp_now_add_peer(&peer) != ESP_OK) {
            Serial.println("[MAIN ESP32] ERROR: esp_now_add_peer failed");
            while (true) delay(1000);
        }

        rx1 = (uint8_t *)heap_caps_malloc(SPI_BUF_SIZE, MALLOC_CAP_DMA);
        tx1 = (uint8_t *)heap_caps_malloc(SPI_BUF_SIZE, MALLOC_CAP_DMA);
        rx2 = (uint8_t *)heap_caps_malloc(SPI_BUF_SIZE, MALLOC_CAP_DMA);
        tx2 = (uint8_t *)heap_caps_malloc(SPI_BUF_SIZE, MALLOC_CAP_DMA);
        if (!rx1 || !tx1 || !rx2 || !tx2) {
            Serial.println("[MAIN ESP32] ERROR: DMA malloc failed");
            while (true) delay(1000);
        }

        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num   = 17;
        buscfg.miso_io_num   = 33;
        buscfg.sclk_io_num   = 18;
        buscfg.quadwp_io_num = -1;
        buscfg.quadhd_io_num = -1;

        spi_slave_interface_config_t slvcfg = {};
        slvcfg.mode         = 0;
        slvcfg.spics_io_num = 5;
        slvcfg.queue_size   = 2;
        slvcfg.flags        = 0;

        if (spi_slave_initialize(SPI3_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO) != ESP_OK) {
            Serial.println("[MAIN ESP32] ERROR: spi_slave_initialize failed");
            while (true) delay(1000);
        }

        initialized = true;
        delay(1000);
        Serial.println("[MAIN ESP32] Ready. MAC: " + WiFi.macAddress());
    }

    memset(tx1, 0xFF, SPI_BUF_SIZE); 
    memset(rx1, 0x00, SPI_BUF_SIZE);

    spi_slave_transaction_t trans1 = {};
    trans1.length    = SPI_BUF_SIZE * 8;
    trans1.tx_buffer = tx1;
    trans1.rx_buffer = rx1;

    spi_slave_queue_trans(SPI3_HOST, &trans1, portMAX_DELAY);

    spi_slave_transaction_t *ret1;
    spi_slave_get_trans_result(SPI3_HOST, &ret1, portMAX_DELAY);

    if (rx1[0] != 1 || rx1[1] != CMD_START_COLL) {
        Serial.printf("[MAIN ESP32] Unexpected packet: len=0x%02X cmd=0x%02X — ignoring\n",
                      rx1[0], rx1[1]);
        return;
    }
    Serial.println("[MAIN ESP32] CMD_START_COLL received from ATmega");

    collected_count = 0;
    memset((void *)collected, 0, sizeof(collected));

    EspPacket cmd_pkt;
    cmd_pkt.type       = PKT_CMD;
    cmd_pkt.robot_addr = 0;
    cmd_pkt.pad[0]     = 0;
    cmd_pkt.pad[1]     = 0;
    esp_now_send(broadcast, (uint8_t *)&cmd_pkt, sizeof(cmd_pkt));
    Serial.println("[MAIN ESP32] CMD_START_COLL broadcast via ESP-NOW");

    delay(COLLECT_TIMEOUT);

    uint8_t snap_count = collected_count;
    uint8_t snap_addrs[MAX_ROBOTS];
    memcpy(snap_addrs, (const void *)collected, snap_count);

    Serial.printf("[MAIN ESP32] Collected %u robot(s):", snap_count);
    for (uint8_t i = 0; i < snap_count; i++) {
        Serial.printf("  [%u] = 0x%02X", i, snap_addrs[i]);
    }
    Serial.println();

    tx2[0] = snap_count;
    memcpy(tx2 + 1, snap_addrs, snap_count);
    memset(tx2 + 1 + snap_count, 0xFF, SPI_BUF_SIZE - 1 - snap_count);
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