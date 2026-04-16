/*
 * sub_atmega328pb.c
 * =================
 * Role : Sub ATmega328PB — SPI SLAVE to its paired Sub ESP32
 *
 * Flow:
 *   1. spi_slave_receive() blocks until the Sub ESP32 sends [LEN][CMD]
 *   2. If CMD == CMD_START_COLL, call spi_slave_send() to return
 *      [1][ROBOT_ADDRESS] during the ESP32's RX transaction
 *   3. Repeat
 *
 * Set ROBOT_ADDRESS at compile time:
 *   avr-gcc ... -DROBOT_ADDRESS=3 sub_atmega328pb.c
 */

#define F_CPU 16000000UL

#include <avr/io.h>
#include <stdint.h>
#include "spi_comm.h"
#include "uart.h"               /* provides uart_printf() — same syntax as printf */

/* ── Configuration ────────────────────────────────────────────────── */

#ifndef ROBOT_ADDRESS
#  define ROBOT_ADDRESS  0      /* override at compile time: -DROBOT_ADDRESS=N */
#endif

/* ── Protocol (must match ESP32 firmware) ─────────────────────────── */
#define CMD_START_COLL  0xA1

/* ── Main ─────────────────────────────────────────────────────────── */
int main(void)
{
    spi_slave_init();

    printf("[SUB ATMEGA %u] Ready\r\n", (unsigned)ROBOT_ADDRESS);

    uint8_t rx_buf[SPI_MAX_PAYLOAD];
    uint8_t rx_len = 0;

    /* Response is always just our one-byte address */
    uint8_t tx_buf[] = { (uint8_t)ROBOT_ADDRESS };

    while (1)
    {
        /*
         * ── 1. Wait for START_COLL from the Sub ESP32 ──
         *
         * spi_slave_receive() blocks until /SS is asserted and the full
         * [LEN][DATA...] frame has been clocked in.
         */
        if (spi_slave_receive(rx_buf, &rx_len) != 0)
        {
            printf("[SUB ATMEGA %u] ERR: bad receive length\r\n",
                        (unsigned)ROBOT_ADDRESS);
            continue;
        }

        /* Expect exactly one command byte */
        if (rx_len != 1 || rx_buf[0] != CMD_START_COLL)
        {
            printf("[SUB ATMEGA %u] ERR: unexpected cmd 0x%02X (len %u)\r\n",
                        (unsigned)ROBOT_ADDRESS, rx_buf[0], rx_len);
            continue;
        }

        printf("[SUB ATMEGA %u] Got START_COLL, sending address\r\n",
                    (unsigned)ROBOT_ADDRESS);

        /*
         * ── 2. Send our address back ──
         *
         * spi_slave_send() pre-loads SPDR with the length byte immediately
         * so it is ready before the ESP32 starts its RX transaction after
         * the guard delay.  It then blocks until the full [LEN][DATA...]
         * frame has been clocked out on MISO.
         */
        if (spi_slave_send(tx_buf, sizeof(tx_buf)) != 0)
        {
            printf("[SUB ATMEGA %u] ERR: send failed\r\n",
                        (unsigned)ROBOT_ADDRESS);
        }
    }
}