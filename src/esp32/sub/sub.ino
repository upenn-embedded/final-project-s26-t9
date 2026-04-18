#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <esp_now.h>

#define CMD_START_COLL 0xA1
#define PKT_CMD        0xA1
#define PKT_ADDR       0xA2

#define TIME_PER_ESP 50  // ms stagger per robot index

#define SS_PIN       5
#define SPI_FREQ     1000000  // 1 MHz — must match ATmega SPR0=1 @ 16 MHz
#define SPI_GUARD_US 1000     // microseconds between TX and RX transactions

static const SPISettings spi_cfg(SPI_FREQ, MSBFIRST, SPI_MODE0);

struct EspPacket {
    uint8_t type;
    uint8_t robot_addr;
    uint8_t pad[2];
};

volatile bool cmd_received = false;

// Callback
void onEspNowReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    if (len < (int)sizeof(EspPacket)) return;
    const EspPacket *pkt = (const EspPacket *)data;
    if (pkt->type == PKT_CMD)
        cmd_received = true;
}

void setup() {
  static const uint8_t broadcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  Serial.begin(115200);

  SPI.begin();
  pinMode(SS_PIN, OUTPUT);
  digitalWrite(SS_PIN, HIGH);

  // WiFi in station mode (required for ESP-NOW)
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
  static const uint8_t broadcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    if (!cmd_received) return;
    cmd_received = false;
    delay(5);

    // Transaction 1: send [LEN=1][CMD_START_COLL] to Sub ATmega
    portDISABLE_INTERRUPTS();

    digitalWrite(SS_PIN, LOW);
    SPI.transfer(1);
    SPI.transfer(CMD_START_COLL);
    digitalWrite(SS_PIN, HIGH);

    // guard delay
    delayMicroseconds(1000);

    // Transaction 2: clock in [LEN][ROBOT_ADDRESS] from Sub ATmega
    digitalWrite(SS_PIN, LOW);
    uint8_t len  = SPI.transfer(0x00);
    uint8_t addr = SPI.transfer(0x00);
    digitalWrite(SS_PIN, HIGH);

    portENABLE_INTERRUPTS();

    Serial.printf("[SUB ESP32] raw len=0x%02X addr=0x%02X\n", len, addr);
    if (len != 1) {
        Serial.printf("[SUB ESP32] Bad resp_len=%u — skipping\n", len);
        return;
    }

    Serial.printf("[SUB ESP32] Got address=0x%02X, staggering %u ms\n",
                  addr, (unsigned)(addr * TIME_PER_ESP));

    // Staggered delay then broadcast
    delay((uint32_t)addr * TIME_PER_ESP);

    EspPacket resp;
    resp.type       = PKT_ADDR;
    resp.robot_addr = addr;
    resp.pad[0]     = 0;
    resp.pad[1]     = 0;
    esp_now_send(broadcast, (uint8_t *)&resp, sizeof(resp));
}