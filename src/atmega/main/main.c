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
        int8_t result = spi_send_message(tx_buf, sizeof(tx_buf));

        if (result != 0)
        {
            printf("[MAIN ATMEGA] ERR: spi send failed\r\n");
            continue;
        }
        
        _delay_ms(1000);
        result = spi_receive_response(rx_buf, &rx_len);
        if (result != 0) {
            printf("[MAIN ATMEGA] ERR: spi receive failed\r\n");
            continue;
        }

        #define NUM_ROBOTS_RECV 2
        #define NUM_PHT         6
        #define DATA_LEN        (NUM_ROBOTS_RECV * NUM_PHT * 2)

        uint8_t  robot_count = rx_buf[0];
        uint8_t *p           = rx_buf + 1;

        for (uint8_t r = 0; r < robot_count; r++)
        {
            uint8_t addr = *p++;
            printf("ROBOT=0x%02X EMITTER=0 ", addr);

            for (uint8_t emitter = 0; emitter < NUM_ROBOTS_RECV; emitter++)
            {
                printf("ROBOT=0x%02X EMITTER=%u", addr, emitter);

                for (uint8_t pht = 0; pht < NUM_PHT; pht++)
                {
                    uint16_t adc = (uint16_t)p[0] | ((uint16_t)p[1] << 8);
                    p += 2;
                    printf(" PHT%u=%u", pht + 1, adc);
                }

                printf("\r\n");
            }
        }
        printf("---\r\n"); 
    }
}