/*
 * ATmega328PB SPI Slave Test
 * Echoes back received byte + 1.
 * Send 0xAB from ESP32, expect 0xAC back.
 *
 * Wiring:
 *   PB2 (SS)   <- ESP32 GPIO5
 *   PB3 (MOSI) <- ESP32 GPIO17
 *   PB4 (MISO) -> ESP32 GPIO33
 *   PB5 (SCK)  <- ESP32 GPIO18
 */

#include "uart.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdio.h>

static volatile uint8_t rx_byte  = 0;
static volatile uint8_t rx_ready = 0;

static void spi_slave_init(void) {
    DDRB |=  (1 << PB4);                              // MISO = output
    DDRB &= ~((1 << PB5) | (1 << PB3) | (1 << PB2)); // SCK, MOSI, SS = inputs
    SPCR0 = (1 << 6) | (1 << 7);                      // SPE=bit6, SPIE=bit7, MSTR=0 (slave)
    SPDR0 = 0x00;                                      // preload response
}

ISR(SPI0_STC_vect) {
    rx_byte  = SPDR0;        // read clears interrupt flag
    SPDR0    = rx_byte + 1;  // preload response for next transfer
    rx_ready = 1;
}

int main(void) {
    UART_init(103);
    spi_slave_init();
    sei();

    printf("ATmega SPI slave ready\r\n");

    while (1) {
        if (rx_ready) {
            rx_ready = 0;
            printf("Recv: 0x%02X  Sending: 0x%02X\r\n", rx_byte, rx_byte + 1);
        }
    }
}
