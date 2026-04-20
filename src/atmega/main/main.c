#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <string.h>
#include "spi_comm.h"
#include "uart.h"

#define CMD_START_COLL    0xA1
#define CMD_MOVE_DATA     0xB1
#define CYCLE_DELAY_MS    200
#define ADC_DIF_THRESHOLD 100
#define NUM_ROBOTS_RECV   2
#define NUM_PHT           6
#define DATA_LEN          (NUM_ROBOTS_RECV * NUM_PHT * 2)

/*
 * Movement instruction for one robot.
 * dir  = PHT index (0–5) of the strongest return — points toward target
 * dist = ADC value representing distance to target
 */
typedef struct {
    uint16_t dir;
    uint16_t dist;
} MoveCmd;

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
        /* ── Send START_COLL ── */
        if (spi_send_message(tx_buf, sizeof(tx_buf)) != 0) {
            printf("[MAIN ATMEGA] ERR: spi send failed\r\n");
            continue;
        }

        _delay_ms(1000);

        if (spi_receive_response(rx_buf, &rx_len) != 0) {
            printf("[MAIN ATMEGA] ERR: spi receive failed\r\n");
            continue;
        }

        uint8_t  robot_count = rx_buf[0];
        uint8_t *p           = rx_buf + 1;

        /*
         * adj_matrix[r][emitter][0] = best ADC distance
         * adj_matrix[r][emitter][1] = PHT index of that reading
         * Indexed by robot slot r (0..robot_count-1), not raw address byte.
         */
        uint16_t adj_matrix[NUM_ROBOTS_RECV][NUM_ROBOTS_RECV][2];
        memset(adj_matrix, 0, sizeof(adj_matrix));

        /* ── Parse and print in one pass ── */
        uint8_t parsed_addr[NUM_ROBOTS_RECV];

        for (uint8_t r = 0; r < robot_count; r++)
        {
            parsed_addr[r] = *p++;

            for (uint8_t emitter = 0; emitter < NUM_ROBOTS_RECV; emitter++)
            {
                printf("ROBOT=0x%02X EMITTER=%u", parsed_addr[r], emitter);

                int32_t max_val  = -1;
                int32_t max_pht  = -1;
                int32_t avg_dist = 0;

                for (uint8_t pht = 0; pht < NUM_PHT; pht++)
                {
                    uint16_t adc = (uint16_t)p[0] | ((uint16_t)p[1] << 8);
                    p += 2;
                    printf(" PHT%u=%u", pht + 1, adc);
                    if ((int32_t)adc > max_val) { max_val = adc; max_pht = pht; }
                    avg_dist += adc;
                }
                avg_dist /= NUM_PHT;
                printf("\r\n");

                /*
                 * Only record edge if one PHT is clearly dominant.
                 * Index by robot slot r, not raw address, to avoid
                 * out-of-bounds writes if address bytes are non-sequential.
                 */
                if (max_val - avg_dist > ADC_DIF_THRESHOLD) {
                    /* Average with reverse edge if already recorded */
                    if (adj_matrix[emitter][r][0] != 0)
                        max_val = ((int32_t)adj_matrix[emitter][r][0] + max_val) / 2;

                    adj_matrix[r][emitter][0] = (uint16_t)max_val;
                    adj_matrix[r][emitter][1] = (uint16_t)max_pht;
                }
            }
        }
        printf("---\r\n");

        /* ── Build movement commands ── */
        /*
         * Robot 1 is stationary.
         * Robot 0 must turn to face robot 1 and drive dist_to_move.
         *
         * adj_matrix[0][1] = what robot 0 saw when robot 1 was emitting
         *   [0] = ADC strength (proxy for distance)
         *   [1] = PHT index    (proxy for direction)
         */
        MoveCmd moves[NUM_ROBOTS_RECV];
        memset(moves, 0, sizeof(moves));

        if (robot_count == 2 && adj_matrix[0][1][0] != 0)
        {
            moves[0].dir  = adj_matrix[0][1][1]; /* PHT toward robot 1 */
            moves[0].dist = adj_matrix[0][1][0]; /* ADC distance       */
            moves[1].dir  = 0;                   /* robot 1 stationary */
            moves[1].dist = 0;

            printf("Robot0: turn=%u dist=%u\r\n", moves[0].dir, moves[0].dist);
        }
        else
        {
            printf("No valid edge — skipping move\r\n");
        }
        printf("---\r\n");
        
        // Move Data
        uint8_t move_buf[1 + NUM_ROBOTS_RECV * 4];
        move_buf[0] = CMD_MOVE_DATA;
        for (uint8_t i = 0; i < NUM_ROBOTS_RECV; i++) {
            move_buf[1 + i*4 + 0] = (uint8_t)(moves[i].dir  & 0xFF);
            move_buf[1 + i*4 + 1] = (uint8_t)(moves[i].dir  >> 8);
            move_buf[1 + i*4 + 2] = (uint8_t)(moves[i].dist & 0xFF);
            move_buf[1 + i*4 + 3] = (uint8_t)(moves[i].dist >> 8);
        }

        if (spi_send_message(move_buf, sizeof(move_buf)) != 0)
            printf("[MAIN ATMEGA] ERR: move send failed\r\n");

        _delay_ms(CYCLE_DELAY_MS);
    }
}