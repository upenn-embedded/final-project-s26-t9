/*
 * sub_atmega328pb.c
 * =================
 * Role : Sub ATmega328PB ? SPI SLAVE to its paired Sub ESP32
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
#include <avr/interrupt.h>
#include <stdint.h>
#include "spi_comm.h"
#include "uart.h"               /* provides uart_printf() ? same syntax as printf */

/* ?? Configuration ?????????????????????????????????????????????????? */

#ifndef ROBOT_ADDRESS
#  define ROBOT_ADDRESS  1      /* override at compile time: -DROBOT_ADDRESS=N */
#endif

/* ?? Protocol (must match ESP32 firmware) ??????????????????????????? */
#define SLAVE_ADDR 0x12
#define CMD_START_COLL 0xA1

volatile uint8_t cmd = 0;
volatile uint8_t response[2];
volatile uint8_t response_len = 0;

/* ?? Main ??????????????????????????????????????????????????????????? */

void i2c_slave_init(void)
{
    TWAR0 = (SLAVE_ADDR << 1);  // set address
    TWCR0 = (1 << TWEN) | (1 << TWEA) | (1 << TWIE) | (1 << TWINT);

    sei();
}

ISR(TWI0_vect)
{
    switch (TWSR0 & 0xF8)
    {
        // SLA+W received (master writing)
        case 0x60:
        case 0x68:
            TWCR0 |= (1 << TWINT) | (1 << TWEA);
            break;

        // data received
        case 0x80:
        {
            cmd = TWDR0;

            if (cmd == CMD_START_COLL) {
                // prepare response immediately
                response[0] = 1;
                response[1] = (uint8_t)ROBOT_ADDRESS;
                response_len = 2;
            }

            TWCR0 |= (1 << TWINT) | (1 << TWEA);
            break;
        }

        // SLA+R received (master reading)
        case 0xA8:
        {
            TWDR0 = response[0];
            response_len = 1;

            TWCR0 |= (1 << TWINT) | (1 << TWEA);
            break;
        }

        // data transmitted
        case 0xB8:
        {
            if (response_len == 1) {
                TWDR0 = response[1];
                response_len = 0;
            }

            TWCR0 |= (1 << TWINT) | (1 << TWEA);
            break;
        }

        // stop or error
        case 0xA0:
        default:
            TWCR0 |= (1 << TWINT) | (1 << TWEA);
            break;
    }
}

int main(void)
{
    uart_init();
//    spi_slave_init();
//
//    printf("[SUB ATMEGA %u] Ready\r\n", (unsigned)ROBOT_ADDRESS);
//
//    uint8_t rx_buf[SPI_MAX_PAYLOAD];
//    uint8_t rx_len = 0;
//
//    /* Response is always just our one-byte address */
//    uint8_t tx_buf[] = { (uint8_t)ROBOT_ADDRESS };
    
    i2c_slave_init();

    while (1)
    {
//        if (spi_slave_receive(rx_buf, &rx_len) != 0)
//        {
//             printf("[SUB ATMEGA %u] ERR: bad receive length\r\n",
//                        (unsigned)ROBOT_ADDRESS);
//            continue;
//        }
//
//        if (rx_len != 1 || rx_buf[0] != CMD_START_COLL)
//        {
//             printf("[SUB ATMEGA %u] ERR: unexpected cmd 0x%02X (len %u)\r\n",
//                        (unsigned)ROBOT_ADDRESS, rx_buf[0], rx_len);
//            continue;
//        }
//
//        if (spi_slave_send(tx_buf, sizeof(tx_buf)) != 0)
//        {
//             printf("[SUB ATMEGA %u] ERR: send failed\r\n",
//                        (unsigned)ROBOT_ADDRESS);
//        }
//        else
//        {
//             printf("[SUB ATMEGA %u] Sent address %u\r\n",
//                        (unsigned)ROBOT_ADDRESS, (unsigned)ROBOT_ADDRESS);
//        }
    }
}