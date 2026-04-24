#define F_CPU 16000000UL
#include "movement.h"
#include "spi_comm.h"
#include "uart.h"
#include <avr/io.h>
#include <stdint.h>
#include <util/delay.h>

#ifndef ROBOT_ADDRESS
#define ROBOT_ADDRESS 0
#endif

#define CMD_START_COLL      0xA1
#define MS_BETWEEN_COLLECTS 10
#define NUM_ROBOTS          2
#define NUM_PHT             6 /* ADC0?ADC5 = PC0?PC5 */

/* ?? Emitter ??????????????????????????????????????????????????????????? */
#define DDREM            DDRD
#define DDEM             DD0
#define PORTEM           PORTD
#define PIN_EM           PD0
#define CMD_MOVE_DATA    0xB1
#define MOVE_PAYLOAD_LEN (NUM_ROBOTS * 4) /* dir+dist per robot, little-endian */
#define MOVE_TIME        250

static void
delay_ms_var(uint16_t ms) {
    while (ms--)
        _delay_us(1000);
}

static inline void
emitter_on(void) {
    PORTEM |= (1 << PIN_EM);
}
static inline void
emitter_off(void) {
    PORTEM &= ~(1 << PIN_EM);
}

/* ?? ADC ??????????????????????????????????????????????????????????????? */
/*
 * Single-conversion mode ? free-running (ADATE) cannot switch channels
 * cleanly because the next conversion starts before ADMUX settles.
 * We start one conversion per channel and poll ADIF.
 */
static void
adc_init(void) {
    /* PC0?PC5 as inputs (ADC0?ADC5) */
    DDRC &= ~0x3F;
    PORTC &= ~0x3F; /* no pull-ups on analog pins */

    PRR0 &= ~(1 << PRADC); /* power on ADC */

    /* AVcc reference */
    ADMUX = (1 << REFS0);

    /* Prescaler = 128 ? 125 kHz ADC clock @ 16 MHz (must be 50?200 kHz) */
    ADCSRA = (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2);

    /* Disable digital input buffers on ADC0?ADC5 to reduce noise */
    DIDR0 = 0x3F;

    ADCSRA |= (1 << ADEN); /* enable ADC ? no ADSC yet, single-shot */
}

/*
 * adc_read(ch) ? single conversion on channel ch (0?5).
 *
 * Switches ADMUX, starts one conversion, waits for ADIF, returns ADC.
 * At 125 kHz ADC clock one conversion = 13 cycles = ~104 µs.
 */
static uint16_t
adc_read(uint8_t ch) {
    /* Select channel ? mask MUX3:0 then set */
    ADMUX = (ADMUX & 0xF0) | (ch & 0x0F);

    /* Short settle after MUX switch before starting conversion */
    _delay_us(10);

    ADCSRA |= (1 << ADSC); /* start single conversion */
    while (ADCSRA & (1 << ADSC))
        ; /* wait for ADSC to clear */
    return ADC;
}

/* ?? Peripheral init ??????????????????????????????????????????????????? */
static void
Initialize(void) {
    /* Emitter pin ? output, starts OFF */
    DDREM |= (1 << DDEM);
    PORTEM &= ~(1 << PIN_EM);

    adc_init();
}

/* ?? Main ?????????????????????????????????????????????????????????????? */
int
main(void) {
    Initialize();
    // uart_init();
    spi_slave_init();

    // printf("[SUB ATMEGA %u] Ready\r\n", (unsigned) ROBOT_ADDRESS);

    uint8_t rx_buf[SPI_MAX_PAYLOAD];
    uint8_t rx_len = 0;

    /*
     * tx_buf layout: NUM_ROBOTS × NUM_PHT × 2 bytes (little-endian uint16_t)
     *
     *   tx_buf[curr_addr * NUM_PHT*2 + pht*2 + 0] = ADC low  byte
     *   tx_buf[curr_addr * NUM_PHT*2 + pht*2 + 1] = ADC high byte
     *
     * When curr_addr == ROBOT_ADDRESS all 6 slots for that row = 0x0000
     * (self-emission, cannot receive own IR).
     *
     * Total = 2 robots × 6 PHTs × 2 bytes = 24 bytes < SPI_MAX_PAYLOAD.
     */
    uint8_t tx_buf[NUM_ROBOTS * NUM_PHT * 2];

    /* In main(), replace the existing receive+check block with: */
    while (1) {
        if (spi_slave_receive(rx_buf, &rx_len) != 0)
            continue;

        /* ?? CMD_START_COLL ? existing collection logic, unchanged ?? */
        if (rx_len >= 1 && rx_buf[0] == CMD_START_COLL) {
            for (uint8_t curr_addr = 0; curr_addr < NUM_ROBOTS; curr_addr++) {
                uint8_t start = curr_addr * NUM_PHT * 2;

                if (curr_addr == ROBOT_ADDRESS) {
                    emitter_on();
                    _delay_ms(MS_BETWEEN_COLLECTS);
                    emitter_off();

                    for (uint8_t p = 0; p < NUM_PHT; p++) {
                        tx_buf[start + p * 2 + 0] = 0x00;
                        tx_buf[start + p * 2 + 1] = 0x00;
                    }
                } else {
                    _delay_ms(MS_BETWEEN_COLLECTS / 2);

                    for (uint8_t p = 0; p < NUM_PHT; p++) {
                        uint16_t val = adc_read(p);
                        tx_buf[start + p * 2 + 0] = (uint8_t) (val & 0xFF);
                        tx_buf[start + p * 2 + 1] = (uint8_t) (val >> 8);
                    }

                    _delay_ms(MS_BETWEEN_COLLECTS / 2);
                }
            }

            int size = (unsigned) sizeof(tx_buf);
            spi_slave_send(tx_buf, sizeof(tx_buf));
            // printf("[SUB ATMEGA%d] Collection done, sent %u bytes\r\n", ROBOT_ADDRESS, size);
            continue;
        }

        /* ?? CMD_MOVE_DATA ? extract and print this robot's movement data ?? */
        if (rx_len >= 1 && rx_buf[0] == CMD_MOVE_DATA) {
            /*
             * Move payload layout (rx_buf[1..] after the command byte):
             *   [dir0_lo][dir0_hi][dist0_lo][dist0_hi]   <- robot 0
             *   [dir1_lo][dir1_hi][dist1_lo][dist1_hi]   <- robot 1
             *   ...
             *
             * This robot's slot is at byte offset ROBOT_ADDRESS * 4
             * within the payload (rx_buf[1] onward).
             */
            uint8_t expected_len = 1 + MOVE_PAYLOAD_LEN; /* cmd + data */
            if (rx_len < expected_len) {
                // printf("[SUB ATMEGA%d] ERR: move payload too short (%u < %u)\r\n", ROBOT_ADDRESS, rx_len, expected_len);
                continue;
            }

            uint8_t offset = 1 + (ROBOT_ADDRESS * 4); /* skip cmd byte, seek to our slot */
            int dir = ((uint16_t) rx_buf[offset + 0] | ((uint16_t) rx_buf[offset + 1] << 8));
            int dist = ((uint16_t) rx_buf[offset + 2] | ((uint16_t) rx_buf[offset + 3] << 8));

            // printf("[SUB ATMEGA%d] MOVE dir=%u dist=%u\r\n", ROBOT_ADDRESS, dir, dist);

            int turn_ms = 166;
            if (dir > 0) {
                if (dist > 0)
                    turn_ms = dist;
                if (dir < 3) {
                    turn_cw();
                } else {
                    turn_ccw();
                }
                delay_ms_var(turn_ms);
                stop_movement();
                continue;
            }

            if (dist != 0) {
                /* dist is in units of 10 ms; use a loop since _delay_ms needs a constant */
                if (dist > 0) {
                    move_forward();
                } else {
                    move_backward();
                }
                _delay_ms(166);
                stop_movement();
            }
            continue;
        }

        /* Unknown command */
        // printf("[SUB ATMEGA%d] Unknown cmd 0x%02X len=%u\r\n", ROBOT_ADDRESS, rx_buf[0], rx_len);
    }
}
