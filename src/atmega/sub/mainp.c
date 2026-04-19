#define F_CPU 8000000UL

#include <avr/io.h>
#include <stdint.h>
#include "spi_comm.h"
#include "uart.h"

#ifndef ROBOT_ADDRESS
#  define ROBOT_ADDRESS  1
#endif

#define CMD_START_COLL  0xA1

int main(void)
{
    uart_init();
    spi_slave_init();

    printf("[SUB ATMEGA %u] Ready\r\n", (unsigned)ROBOT_ADDRESS);

    uint8_t rx_buf[SPI_MAX_PAYLOAD];
    uint8_t rx_len = 0;

    uint8_t tx_buf[] = { (uint8_t)ROBOT_ADDRESS };

    while (1)
    {
        if (spi_slave_receive(rx_buf, &rx_len) != 0)
        {
            //printf("[SUB ATMEGA %u] ERR: bad receive length\r\n",
            //          (unsigned)ROBOT_ADDRESS);
            continue;
        }

        if (rx_len != 1 || rx_buf[0] != CMD_START_COLL)
        {
            //printf("[SUB ATMEGA %u] ERR: unexpected cmd 0x%02X (len %u)\r\n",
            //          (unsigned)ROBOT_ADDRESS, rx_buf[0], rx_len);
            continue;
        }

        if (spi_slave_send(tx_buf, sizeof(tx_buf)) != 0)
        {
            //printf("[SUB ATMEGA %u] ERR: send failed\r\n",
            //         (unsigned)ROBOT_ADDRESS);
        }
        else
        {
            //printf("[SUB ATMEGA %u] Sent address %u\r\n",
            //         (unsigned)ROBOT_ADDRESS, (unsigned)ROBOT_ADDRESS);
        }
    }
}