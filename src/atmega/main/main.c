/*
 * main_atmega328pb.c
 * ==================
 * Role : Main ATmega328PB — SPI MASTER to Main ESP32
 *
 * Flow:
 *   1. Send [1][CMD_START_COLL] to Main ESP32 via spi_send_message()
 *   2. spi_receive_response() applies the guard delay then clocks in
 *      the ESP32's compiled reply (one address byte per sub-robot)
 *   3. Print the received bytes over UART with uart_printf()
 *   4. Repeat
 */

#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include "spi_comm.h"
#include "uart.h"               /* provides uart_printf() — same syntax as printf */

/* ── Protocol (must match ESP32 firmware) ─────────────────────────── */
#define CMD_START_COLL  0xA1

/* Pause between complete START_COLL cycles.
 * Must exceed the Main ESP32's COLLECT_TIMEOUT (150 ms) so the ESP32
 * finishes collecting sub-robot addresses before the next CMD arrives. */
#define CYCLE_DELAY_MS  200

/* ── Main ─────────────────────────────────────────────────────────── */
int main(void)
{
    uart_init();
    spi_master_init();

    printf("[MAIN ATMEGA] Ready\r\n");

    uint8_t tx_buf[]             = { CMD_START_COLL };
    uint8_t rx_buf[SPI_MAX_PAYLOAD];
    uint8_t rx_len               = 0;

    while (1)
    {
        /*
         * spi_exchange():
         *   TX transaction -> guard delay -> RX transaction
         *
         * The ESP32 uses the guard delay to:
         *   - receive all sub-ESP32 replies over ESP-NOW
         *   - pack them into its SPI TX buffer
         *
         * If the ESP-NOW round-trip exceeds SPI_GUARD_US, increase
         * SPI_GUARD_US in atmega328pb_spi.c accordingly.
         */
        int8_t result = spi_exchange(tx_buf, sizeof(tx_buf), rx_buf, &rx_len);

        if (result != 0)
        {
            printf("[MAIN ATMEGA] ERR: spi_exchange failed\r\n");
        }
        else
        {
            printf("[MAIN ATMEGA] RX %u byte(s):", rx_len);
            for (uint8_t i = 0; i < rx_len; i++)
                printf(" 0x%02X", rx_buf[i]);
            printf("\r\n");
        }

        _delay_ms(CYCLE_DELAY_MS);
    }
}