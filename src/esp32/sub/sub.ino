#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <esp_now.h>

#define CMD_START_COLL 0xA1
#define PKT_CMD        0xA1
#define PKT_ADDR       0xA2

#define NUM_ROBOTS   2
#define NUM_PHT      6
#define DATA_LEN     (NUM_ROBOTS * NUM_PHT * 2)  /* 24 bytes */

#ifndef ROBOT_ADDRESS
#  define ROBOT_ADDRESS 0
#endif

#define TIME_PER_ESP 50   /* ms stagger per robot index */

#define SS_PIN       5
#define SPI_FREQ     1000000
static const SPISettings spi_cfg(SPI_FREQ, MSBFIRST, SPI_MODE0);

/*
 * Guard delay between sending CMD and reading response.
 *
 * ATmega collection loop = NUM_ROBOTS × MS_BETWEEN_COLLECTS
 *                        = 2 × 10 ms = 20 ms minimum.
 * Add margin for ADC sweeps (6 × ~104 µs ≈ 0.6 ms) and SPI overhead.
 * 30 ms is safe; do NOT use delayMicroseconds(1000) = 1 ms here.
 */
#define SPI_GUARD_MS 30

struct EspDataPacket {
    uint8_t type;
    uint8_t robot_addr;
    uint8_t pad[2];
    uint8_t data[DATA_LEN];
};

volatile bool cmd_received = false;

void onEspNowReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    if (len < (int)sizeof(EspDataPacket)) return;
    const EspDataPacket *pkt = (const EspDataPacket *)data;
    if (pkt->type == PKT_CMD)
        cmd_received = true;
}

void setup() {
    static const uint8_t broadcast[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    Serial.begin(115200);

    SPI.begin();
    pinMode(SS_PIN, OUTPUT);
    digitalWrite(SS_PIN, HIGH);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    if (esp_now_init() != ESP_OK) {
        Serial.println("[SUB ESP32] ERROR: esp_now_init failed");
        while (true) delay(1000);
    }
    esp_now_register_recv_cb(onEspNowReceive);

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, broadcast, 6);
    peer.channel = 0;
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) != ESP_OK) {
        Serial.println("[SUB ESP32] ERROR: esp_now_add_peer failed");
        while (true) delay(1000);
    }

    delay(100);
    Serial.println("[SUB ESP32] Ready. MAC: " + WiFi.macAddress());
}

void loop() {
  static const uint8_t broadcast[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

  if (!cmd_received) return;
  cmd_received = false;

  delay(5); /* debounce */

  /* ── TX: send [LEN=1][CMD_START_COLL] to sub ATmega ── */
  portDISABLE_INTERRUPTS();
  SPI.beginTransaction(spi_cfg);
  digitalWrite(SS_PIN, LOW);
  SPI.transfer(1);
  SPI.transfer(CMD_START_COLL);
  digitalWrite(SS_PIN, HIGH);
  SPI.endTransaction();
  portENABLE_INTERRUPTS();

  /*
    * Guard delay OUTSIDE the interrupt lock so FreeRTOS tick can run
    * and delay() actually counts real milliseconds.
    * ATmega collection = NUM_ROBOTS × MS_BETWEEN_COLLECTS = 20 ms.
    * 30 ms gives comfortable margin.
    */
  delay(SPI_GUARD_MS);

  /* ── RX: clock in [LEN][DATA_LEN bytes] from sub ATmega ── */
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

  Serial.printf("[SUB ESP32] raw bytes: len=%u data[0]=%u data[1]=%u\n", 
                len, rx_data[0], rx_data[1]);
  if (len != DATA_LEN) {
      Serial.printf("[SUB ESP32] Bad resp_len=%u (expected %u) — skipping\n",
                    len, (unsigned)DATA_LEN);
      return;
  }

  Serial.printf("[SUB ESP32] Got %u bytes from ATmega, staggering %u ms\n",
                len, (unsigned)(ROBOT_ADDRESS * TIME_PER_ESP));

  delay((uint32_t)ROBOT_ADDRESS * TIME_PER_ESP);

  EspDataPacket resp;
  resp.type       = PKT_ADDR;
  resp.robot_addr = ROBOT_ADDRESS;
  resp.pad[0]     = 0;
  resp.pad[1]     = 0;
  memcpy(resp.data, rx_data, DATA_LEN);
  esp_now_send(broadcast, (uint8_t *)&resp, sizeof(resp));
}