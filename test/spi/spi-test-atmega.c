/*
 * spi-test-atmega.c
 * =================
 * SPI SLAVE side of the ATmega <-> ESP32 loopback test.
 *
 * Waits for the ESP32 master to send [LEN][DATA...], then echoes
 * the exact same payload back during the ESP32's RX transaction.
 * Prints each round's result over UART at 115200 baud.
 *
 * Wiring:
 *   ATmega PB2 (SS)   -> ESP32 GPIO5  (SS)
 *   ATmega PB3 (MOSI) -> ESP32 GPIO23 (MOSI)
 *   ATmega PB4 (MISO) -> ESP32 GPIO19 (MISO)
 *   ATmega PB5 (SCK)  -> ESP32 GPIO18 (SCK)
 *   GND <-> GND
 *
 * Build:
 *   avr-gcc -mmcu=atmega328pb -DF_CPU=16000000UL -O2 -I../src/atmega/common \
 *       spi-test-atmega.c \
 *       ../src/atmega/common/spi_comm.c \
 *       ../src/atmega/common/uart.c \
 *       -o spi-test-atmega.elf
 */

#define F_CPU 16000000UL

#include <avr/io.h>
#include <stdint.h>
#include "../src/atmega/common/spi_comm.h"
#include "../src/atmega/common/uart.h"

int main(void)
{
    uart_init();
    spi_slave_init();

    printf("[ATMEGA SPI SLAVE] Ready\r\n");

    uint8_t rx_buf[SPI_MAX_PAYLOAD];
    uint8_t rx_len = 0;
    unsigned round = 0;

    while (1)
    {
        /* ── 1. Wait for ESP32's TX transaction ── */
        if (spi_slave_receive(rx_buf, &rx_len) != 0)
        {
            printf("[ATMEGA] ERR: bad receive length\r\n");
            continue;
        }

        round++;

        /*
         * ── 2. Echo the payload back ──
         *
         * Do NOT printf between receive and send — UART at 115200 baud
         * takes ~4 ms per line, blowing past the ESP32's 300 µs guard
         * delay and causing it to read a stale SPDR.  Print AFTER send.
         */
        int8_t rc = spi_slave_send(rx_buf, rx_len);

        if (rc != 0)
        {
            printf("[ATMEGA] Round %u ERR: send failed\r\n", round);
        }
        else
        {
            printf("[ATMEGA] Round %u OK: echoed %u bytes:", round, rx_len);
            for (uint8_t i = 0; i < rx_len; i++)
                printf(" 0x%02X", rx_buf[i]);
            printf("\r\n");
        }
    }
}
