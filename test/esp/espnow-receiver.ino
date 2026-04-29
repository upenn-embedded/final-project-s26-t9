#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// Must match the struct in espnow-sender.ino
struct Packet {
    uint32_t seq;
    char msg[32];
};

String
macToString(const uint8_t *mac) {
    char buf[18];
    snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return String(buf);
}

void
onReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    if (len < static_cast<int>(sizeof(Packet))) {
        Serial.println("Got a short/malformed packet, skipping.");
        return;
    }

    Packet pkt;
    memcpy(&pkt, data, sizeof(pkt));

    Serial.printf("Received from %s — seq: %u, message: \"%s\"\n", macToString(info->src_addr).c_str(), pkt.seq, pkt.msg);
}

void
setup() {}

void
loop() {
    Serial.begin(115200);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    if (esp_now_init() != ESP_OK) {
        Serial.println("ERROR: esp_now_init() failed. Halting.");
        while (true)
            ;
    }

    esp_now_register_recv_cb(onReceive);

    Serial.println("Receiver ready. Waiting for broadcasts...");
    Serial.println("This device's MAC: " + WiFi.macAddress());

    while (1) {
        delay(100);
    }
}
