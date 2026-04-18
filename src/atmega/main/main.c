#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include "spi_comm.h"
#include "uart.h"               

#define CMD_START_COLL  0xA1

#define CYCLE_DELAY_MS  200

int main(void)
{
    uart_init();
    spi_master_init();

    printf("[MAIN ATMEGA] Ready\r\n");

    uint8_t tx_buf[] = { CMD_START_COLL };
    uint8_t rx_buf[SPI_MAX_PAYLOAD];
    uint8_t rx_len = 0;

    while (1)
    {
        int8_t result = spi_exchange(tx_buf, sizeof(tx_buf), rx_buf, &rx_len);

        if (result != 0)
        {
            printf("[MAIN ATMEGA] ERR: spi_exchange failed\r\n");
        }
        else
        {
            uint8_t robot_count = rx_buf[0];
            printf("[MAIN ATMEGA] Received %u robot(s):", robot_count);
            for (uint8_t i = 0; i < robot_count; i++)
                printf(" 0x%02X", rx_buf[1 + i]);
            printf("\r\n");
        }

        _delay_ms(CYCLE_DELAY_MS);
    }
}