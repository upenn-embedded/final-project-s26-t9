#define F_CPU 8000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include "spi_comm.h"
#include "uart.h"

#ifndef ROBOT_ADDRESS
#  define ROBOT_ADDRESS 1
#endif

#define CMD_START_COLL      0xA1
#define MS_BETWEEN_COLLECTS 10
#define NUM_ROBOTS          2
#define NUM_PHT             6   /* ADC0?ADC5 = PC0?PC5 */

/* ?? Emitter ??????????????????????????????????????????????????????????? */
#define DDREM  DDRD
#define DDEM   DD6
#define PORTEM PORTD
#define PIN_EM PD6

static inline void emitter_on(void)  { PORTEM |=  (1 << PIN_EM); }
static inline void emitter_off(void) { PORTEM &= ~(1 << PIN_EM); }

/* ?? ADC ??????????????????????????????????????????????????????????????? */
/*
 * Single-conversion mode ? free-running (ADATE) cannot switch channels
 * cleanly because the next conversion starts before ADMUX settles.
 * We start one conversion per channel and poll ADIF.
 */
static void adc_init(void)
{
    /* PC0?PC5 as inputs (ADC0?ADC5) */
    DDRC  &= ~0x3F;
    PORTC &= ~0x3F; /* no pull-ups on analog pins */

    PRR     &= ~(1 << PRADC);  /* power on ADC (PRR0 is just PRR on 328P) */

    /* AVcc reference */
    ADMUX  =  (1 << REFS0);

    /* Prescaler = 64 -> 125 kHz ADC clock @ 8 MHz (must be 50?200 kHz) */
    /* ADPS2 and ADPS1 set to 1, ADPS0 to 0 */
    ADCSRA =  (1 << ADPS1) | (1 << ADPS2);

    /* Disable digital input buffers on ADC0?ADC5 to reduce noise */
    DIDR0  =  0x3F;

    ADCSRA |= (1 << ADEN);   /* enable ADC ? no ADSC yet, single-shot */
}

/*
 * adc_read(ch) ? single conversion on channel ch (0?5).
 *
 * Switches ADMUX, starts one conversion, waits for ADIF, returns ADC.
 * At 125 kHz ADC clock one conversion = 13 cycles = ~104 µs.
 */
static uint16_t adc_read(uint8_t ch)
{
    /* Select channel ? mask MUX3:0 then set */
    ADMUX = (ADMUX & 0xF0) | (ch & 0x0F);

    /* Short settle after MUX switch before starting conversion */
    _delay_us(10);

    ADCSRA |= (1 << ADSC);              /* start single conversion */
    while (ADCSRA & (1 << ADSC));       /* wait for ADSC to clear */
    return ADC;
}

/* ?? Peripheral init ??????????????????????????????????????????????????? */
static void Initialize(void)
{
    /* Emitter pin ? output, starts OFF */
    DDREM  |=  (1 << DDEM);
    PORTEM &= ~(1 << PIN_EM);

    adc_init();
}

/* ?? Main ?????????????????????????????????????????????????????????????? */
int main(void)
{
    Initialize();
    uart_init();
    spi_slave_init();

    printf("[SUB ATMEGA %u] Ready\r\n", (unsigned)ROBOT_ADDRESS);

    uint8_t  rx_buf[SPI_MAX_PAYLOAD];
    uint8_t  rx_len = 0;

    /*
     * tx_buf layout: NUM_ROBOTS × NUM_PHT × 2 bytes (little-endian uint16_t)
     *
     * tx_buf[curr_addr * NUM_PHT*2 + pht*2 + 0] = ADC low  byte
     * tx_buf[curr_addr * NUM_PHT*2 + pht*2 + 1] = ADC high byte
     *
     * When curr_addr == ROBOT_ADDRESS all 6 slots for that row = 0x0000
     * (self-emission, cannot receive own IR).
     *
     * Total = 2 robots × 6 PHTs × 2 bytes = 24 bytes < SPI_MAX_PAYLOAD.
     */
    uint8_t tx_buf[NUM_ROBOTS * NUM_PHT * 2];

    while (1)
    {
        /* ?? Wait for CMD_START_COLL from Sub ESP32 ?? */
        if (spi_slave_receive(rx_buf, &rx_len) != 0)
        {
            // printf("[SUB ATMEGA %u] ERR: bad receive length\r\n", (unsigned)ROBOT_ADDRESS);
            continue;
        }

        if (rx_len != 1 || rx_buf[0] != CMD_START_COLL)
        {
            // printf("[SUB ATMEGA %u] ERR: unexpected cmd 0x%02X (len %u)\r\n", (unsigned)ROBOT_ADDRESS, rx_buf[0], rx_len);
            continue;
        }

        /* ?? Collection loop ?? */
        for (uint8_t curr_addr = 0; curr_addr < NUM_ROBOTS; curr_addr++)
        {
            uint8_t start = curr_addr * NUM_PHT * 2;

            if (curr_addr == ROBOT_ADDRESS)
            {
                /*
                 * Our turn to emit ? fire emitter, zero out our row.
                 * We cannot read our own IR return.
                 */
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
                /*
                 * Another robot is emitting ? sample all 6 PHT channels.
                 *
                 * Wait half the slot for the emitter to ramp up and the
                 * IR to stabilise, then do one ADC sweep across all 6
                 * channels (6 × ~104 µs ? 624 µs total), then idle out
                 * the remainder of the slot.
                 */
                _delay_ms(MS_BETWEEN_COLLECTS / 2);

                for (uint8_t p = 0; p < NUM_PHT; p++) {
                    uint16_t val = adc_read(p); /* ADC0=PHT1 ? ADC5=PHT6 */
                    tx_buf[start + p*2 + 0] = (uint8_t)(val & 0xFF);
                    tx_buf[start + p*2 + 1] = (uint8_t)(val >> 8);
                }

                _delay_ms(MS_BETWEEN_COLLECTS / 2);
            }
        }

        /*
         * ?? Send PHT data to Sub ESP32 ??
         *
         * No printf before send ? at 115200 baud printf ? 4 ms which
         * blows past the ESP32's guard delay and corrupts SPDR.
         */
        if (spi_slave_send(tx_buf, sizeof(tx_buf)) != 0)
        {
             // printf("[SUB ATMEGA %u] ERR: send failed\r\n", (unsigned)ROBOT_ADDRESS);
        }
    }
}