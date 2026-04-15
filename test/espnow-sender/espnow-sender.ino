#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// Broadcast address — all ESP32s in range receive this
const uint8_t BROADCAST_ADDR[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Must match the struct in espnow-receiver.ino
struct Packet {
    uint32_t seq;
    char msg[32];
};

uint32_t seqNum = 0;

void
onSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
    bool success = (status == ESP_NOW_SEND_SUCCESS);
    Serial.printf("Broadcast #%u %s\n", seqNum - 1, success ? "delivered" : "FAILED");
}

void
setup() {}

void
loop() {
    Serial.begin(115200);

    // Station mode — no AP needed, just enables the radio
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    if (esp_now_init() != ESP_OK) {
        Serial.println("ERROR: esp_now_init() failed. Halting.");
        while (true)
            ;
    }

    esp_now_register_send_cb(onSent);

    // Register the broadcast address as the only peer
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, BROADCAST_ADDR, sizeof(BROADCAST_ADDR));
    peer.channel = 0;   // 0 = use current channel
    peer.encrypt = false;

    if (esp_now_add_peer(&peer) != ESP_OK) {
        Serial.println("ERROR: esp_now_add_peer() failed. Halting.");
        while (true)
            ;
    }

    Serial.println("Sender ready. Broadcasting every 1s...");
    Serial.println("This device's MAC: " + WiFi.macAddress());

    while (1) {
        Packet pkt;
        pkt.seq = seqNum++;
        snprintf(pkt.msg, sizeof(pkt.msg), "hello #%u", pkt.seq);

        esp_now_send(BROADCAST_ADDR, reinterpret_cast<const uint8_t *>(&pkt), sizeof(pkt));

        delay(1000);
    }
}
