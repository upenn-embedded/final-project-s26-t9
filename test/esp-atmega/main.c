#define F_CPU 16000000UL

#include "uart.h"
#include <avr/io.h>
#include <stdint.h>

#define SPI_MAX_PAYLOAD 64

static void
spi_slave_init(void) {
    /* MISO = output, SS/MOSI/SCK = inputs */
    DDRB = (1 << DDB4);
    /* Enable SPI0 in slave mode, MSB first, Mode 0 (SPE=bit6) */
    SPCR0 = (1 << 6);
}

static uint8_t
spi_transfer_byte(void) {
    while (!(SPSR0 & (1 << 7)))  /* SPIF = bit 7 */
        ;
    return SPDR0;
}

int
main(void) {
    uart_init();
    spi_slave_init();

    printf("ATmega SPI slave ready\r\n");

    uint8_t buf[SPI_MAX_PAYLOAD];
    unsigned round = 0;

    while (1) {
        uint8_t len = spi_transfer_byte();

        if (len == 0 || len > SPI_MAX_PAYLOAD) {
            printf("[ATMEGA] bad len=%u, skipping\r\n", len);
            continue;
        }

        for (uint8_t i = 0; i < len; i++)
            buf[i] = spi_transfer_byte();

        round++;
        printf("[ATMEGA] Round %u — recv %u bytes:", round, len);
        for (uint8_t i = 0; i < len; i++)
            printf(" 0x%02X", buf[i]);
        printf("\r\n");
    }
}
