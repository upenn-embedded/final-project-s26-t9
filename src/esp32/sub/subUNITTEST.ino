#include <SPI.h>

#define SS_PIN    5     // whichever GPIO you've wired to ATmega's SS
#define CMD_START_COLL  0xA1

void setup() {
    Serial.begin(115200);
    SPI.begin();
    pinMode(SS_PIN, OUTPUT);
    digitalWrite(SS_PIN, HIGH);
    delay(1000);  // give ATmega time to boot
}

void loop() {
    // ── 1. Send the command ──
    digitalWrite(SS_PIN, LOW);
    SPI.transfer(1);           // LEN byte  = 1 data byte follows
    SPI.transfer(CMD_START_COLL);  // DATA byte = 0xA1
    digitalWrite(SS_PIN, HIGH);

    // ── 2. Guard delay — ATmega needs time to prepare response ──
    delayMicroseconds(1000);    // must match what ATmega expects

    // ── 3. Read the response ──
    digitalWrite(SS_PIN, LOW);
    uint8_t len  = SPI.transfer(0x00);  // dummy to clock in LEN byte
    uint8_t addr = SPI.transfer(0x00);  // dummy to clock in address byte
    digitalWrite(SS_PIN, HIGH);

    // ── 4. Print result ──
    Serial.print("Got robot address: ");
    Serial.println(addr);

    delay(1000);
}