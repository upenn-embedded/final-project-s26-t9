/*
 * ATmega328P SPI TX Test
 * Sends an incrementing counter to the ESP32 master on each SPI transaction.
 * The ESP32 initiates each transfer with a dummy 0x00; ATmega replies with
 * the current counter value, then increments it for next time.
 *
 * Expected output on ESP32: 0x00, 0x01, 0x02, 0x03, ...
 *
 * Wiring (SPI slave):
 *   PB2 (SS)   <- ESP32 GPIO5
 *   PB3 (MOSI) <- ESP32 GPIO17
 *   PB4 (MISO) -> ESP32 GPIO33
 *   PB5 (SCK)  <- ESP32 GPIO18
 *   GND <-> GND
 *
 * Build: MPLAB X IDE, target atmega328p, F_CPU = 16000000UL
 */

#include "uart.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdio.h>

static volatile uint8_t counter = 0;

/* ---------- SPI slave ---------- */

static void
spi_slave_init(void) {
    DDRB |=  (1 << PB4);
    DDRB &= ~((1 << PB5) | (1 << PB3) | (1 << PB2));

    SPCR0 = (1 << 6) | (1 << 7);   /* SPE=bit6, SPIE=bit7 */

    SPDR0 = counter;   /* pre-load first value */
}

/* ---------- SPI interrupt ---------- */

ISR(SPI0_STC_vect) {
    /* SPDR0 just shifted out counter; load the next value */
    counter++;
    SPDR0 = counter;

    printf("Sent: 0x%02X\r\n", counter - 1);
}

/* ---------- main ---------- */

int
main(void) {
    UART_init(103);
    spi_slave_init();
    sei();

    printf("ATmega SPI TX test ready\r\n");

    while (1) {}

    return 0;
}
