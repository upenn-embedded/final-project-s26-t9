#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include "spi_comm.h"
#include "uart.h"

#ifndef ROBOT_ADDRESS
#  define ROBOT_ADDRESS 0
#endif

#define CMD_START_COLL      0xA1
#define MS_BETWEEN_COLLECTS 10
#define NUM_ROBOTS          2
#define NUM_PHT             6 

#define DDREM  DDRD
#define DDEM   DD6
#define PORTEM PORTD
#define PIN_EM PD6

static inline void emitter_on(void)  { PORTEM |=  (1 << PIN_EM); }
static inline void emitter_off(void) { PORTEM &= ~(1 << PIN_EM); }

static void adc_init(void)
{
    DDRC  &= ~0x3F;
    PORTC &= ~0x3F;

    PRR0  &= ~(1 << PRADC); 

    ADMUX  =  (1 << REFS0);

    ADCSRA =  (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2);

    DIDR0  =  0x3F;

    ADCSRA |= (1 << ADEN); 
}

static uint16_t adc_read(uint8_t ch)
{
    ADMUX = (ADMUX & 0xF0) | (ch & 0x0F);

    _delay_us(10);

    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    return ADC;
}

static void Initialize(void)
{
    DDREM  |=  (1 << DDEM);
    PORTEM &= ~(1 << PIN_EM);

    adc_init();
}

int main(void)
{
    Initialize();
    uart_init();
    spi_slave_init();

    printf("[SUB ATMEGA %u] Ready\r\n", (unsigned)ROBOT_ADDRESS);

    uint8_t  rx_buf[SPI_MAX_PAYLOAD];
    uint8_t  rx_len = 0;

    uint8_t tx_buf[NUM_ROBOTS * NUM_PHT * 2];

    while (1)
    {
        if (spi_slave_receive(rx_buf, &rx_len) != 0)
            continue;
        if (rx_len != 1 || rx_buf[0] != CMD_START_COLL)
            continue;

        for (uint8_t curr_addr = 0; curr_addr < NUM_ROBOTS; curr_addr++)
        {
            uint8_t start = curr_addr * NUM_PHT * 2;

            if (curr_addr == ROBOT_ADDRESS)
            {
                emitter_on();
                _delay_ms(MS_BETWEEN_COLLECTS);
                emitter_off();

                for (uint8_t p = 0; p < NUM_PHT; p++) {
                    tx_buf[start + p*2 + 0] = 0x00;
                    tx_buf[start + p*2 + 1] = 0x00;
                }
            }
            else
            {
                _delay_ms(MS_BETWEEN_COLLECTS / 2);

                for (uint8_t p = 0; p < NUM_PHT; p++) {
                    uint16_t val = adc_read(p);
                    tx_buf[start + p*2 + 0] = (uint8_t)(val & 0xFF);
                    tx_buf[start + p*2 + 1] = (uint8_t)(val >> 8);
                }

                _delay_ms(MS_BETWEEN_COLLECTS / 2);
            }
        }

        int size = (unsigned)sizeof(tx_buf);
        spi_slave_send(tx_buf, sizeof(tx_buf));
        printf("[SUB ATMEGA%d] Collection done, sent %u bytes\r\n", ROBOT_ADDRESS, size);
    }
}