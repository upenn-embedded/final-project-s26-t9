/*
 * ATmega328P SPI Slave Test
 * Receives a byte from ESP32 master and echoes back (received + 1).
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

/* ---------- SPI slave ---------- */

static void
spi_slave_init(void) {
    /* MISO is output; SCK, MOSI, SS are inputs */
    DDRB |= (1 << PB4);
    DDRB &= ~((1 << PB5) | (1 << PB3) | (1 << PB2));

    /*
     * SPCR: SPE=1 (enable), SPIE=1 (interrupt), MSTR=0 (slave),
     *       CPOL=0, CPHA=0 → SPI mode 0, MSB first
     */
    SPCR0 = (1 << 6) | (1 << 7);   /* SPE=bit6, SPIE=bit7 */

    /* Pre-load first response byte so it is ready before first transaction */
    SPDR0 = 0x00;
}

/* ---------- SPI interrupt ---------- */

ISR(SPI0_STC_vect) {
    uint8_t rx = SPDR0; /* reading SPDR0 clears the interrupt flag */
    SPDR0 = rx + 1;     /* load response for next transfer */

    printf("Recv: 0x%02X\r\n", rx);
}

/* ---------- main ---------- */

int
main(void) {
    UART_init(103);
    spi_slave_init();
    sei();

    printf("ATmega SPI slave ready\r\n");

    while (1) {
        /* all work is done in ISR */
    }

    return 0;
}
