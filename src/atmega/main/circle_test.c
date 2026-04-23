#define F_CPU 16000000UL
#include "spi_comm.h"
#include "uart.h"
#include <avr/io.h>
#include <stdint.h>
#include <string.h>
#include <util/delay.h>

#define CMD_START_COLL    0xA1
#define CMD_MOVE_DATA     0xB1
#define CYCLE_DELAY_MS    200
#define ADC_DIF_THRESHOLD 0
#define NUM_ROBOTS_RECV   2
#define NUM_PHT           6

#define MOVING_ROBOT  1
#define CENTRAL_ROBOT 0

#define MIN_DIST_CIRCLE_THRESHOLD 30
#define MAX_DIST_CIRCLE_THRESHOLD 60

/*
 * TARGET_SIDE: PHT index on the central robot to stop at.
 * adj_matrix[CENTRAL_ROBOT][MOVING_ROBOT][1] == TARGET_SIDE means
 * the moving robot is in front of that face of the central robot.
 */
#define TARGET_SIDE 1

typedef struct {
    uint16_t dir;
    uint16_t dist;
} MoveCmd;

typedef enum { ORBIT_START, ORBIT_TURN, ORBIT_MOVE } OrbitPhase;

int
main(void) {
    uart_init();
    spi_master_init();

    printf("[CIRCLE] Ready. Orbiting robot %d around robot %d. Target side=%d\r\n", MOVING_ROBOT, CENTRAL_ROBOT, TARGET_SIDE);

    uint8_t tx_buf[] = {CMD_START_COLL};
    uint8_t rx_buf[SPI_MAX_PAYLOAD];
    uint8_t rx_len = 0;

    OrbitPhase orbit_phase = ORBIT_START;
    int done = 0;
    int time_since_turn = 0;

    while (1) {
        if (spi_send_message(tx_buf, sizeof(tx_buf)) != 0) {
            printf("[CIRCLE] ERR: spi send failed\r\n");
            continue;
        }

        _delay_ms(1000);

        if (spi_receive_response(rx_buf, &rx_len) != 0) {
            printf("[CIRCLE] ERR: spi receive failed\r\n");
            continue;
        }

        uint8_t robot_count = rx_buf[0];
        uint8_t *p = rx_buf + 1;

        if (robot_count < NUM_ROBOTS_RECV) {
            printf("[CIRCLE] Only saw %d robots. Repolling.\r\n", robot_count);
            continue;
        }

        /*
         * adj_matrix[r][emitter][0] = peak ADC on robot r when emitter fires
         * adj_matrix[r][emitter][1] = PHT index on robot r of that peak
         *
         * adj_matrix[CENTRAL_ROBOT][MOVING_ROBOT][1] = which face of the central
         * robot is currently closest to the moving robot — used for stop condition.
         */
        uint16_t adj_matrix[NUM_ROBOTS_RECV][NUM_ROBOTS_RECV][2];
        memset(adj_matrix, 0, sizeof(adj_matrix));

        for (uint8_t r = 0; r < robot_count; r++) {
            uint8_t addr = *p++;
            (void) addr;

            for (uint8_t emitter = 0; emitter < NUM_ROBOTS_RECV; emitter++) {
                int32_t max_val = -1;
                int32_t max_pht = -1;
                int32_t avg_dist = 0;

                for (uint8_t pht = 0; pht < NUM_PHT; pht++) {
                    uint16_t adc = (uint16_t) p[0] | ((uint16_t) p[1] << 8);
                    p += 2;
                    printf(" PHT%u=%u", pht + 1, adc);
                    if ((int32_t) adc > max_val) {
                        max_val = adc;
                        max_pht = pht;
                    }
                    avg_dist += adc;
                }
                avg_dist /= NUM_PHT;
                printf("\r\n");

                if (max_val - avg_dist > ADC_DIF_THRESHOLD) {
                    if (adj_matrix[emitter][r][0] != 0)
                        max_val = ((int32_t) adj_matrix[emitter][r][0] + max_val) / 2;

                    adj_matrix[r][emitter][0] = (uint16_t) max_val;
                    adj_matrix[r][emitter][1] = (uint16_t) max_pht;
                }
            }
        }
        printf("---\r\n");

        uint16_t central_pht = adj_matrix[CENTRAL_ROBOT][MOVING_ROBOT][1];
        uint16_t central_dist = adj_matrix[CENTRAL_ROBOT][MOVING_ROBOT][0];
        uint16_t moving_pht = adj_matrix[MOVING_ROBOT][CENTRAL_ROBOT][1];

        printf("[CIRCLE] central_pht=%u central_dist=%u phase=%s\r\n", central_pht, central_dist, (orbit_phase == ORBIT_TURN) ? "TURN" : "MOVE");

        MoveCmd moves[NUM_ROBOTS_RECV];
        memset(moves, 0, sizeof(moves));

        if (done) {
            /* Stay still — no-op every cycle */
        } else if (central_dist != 0 && central_pht == TARGET_SIDE) {
            printf("[CIRCLE] Reached target side %d! Stopping.\r\n", TARGET_SIDE);
            done = 1;
        } else {
            if (orbit_phase == ORBIT_START) {
                if (moving_pht != 2) {
                    moves[MOVING_ROBOT].dir = 5;
                    moves[MOVING_ROBOT].dist = 0;
                } else {
                    orbit_phase = ORBIT_MOVE;
                }
            } else if (time_since_turn <= 0 && central_dist > MAX_DIST_CIRCLE_THRESHOLD) {
                moves[MOVING_ROBOT].dir = 5;
                moves[MOVING_ROBOT].dist = 0;
                time_since_turn = 2;
            } else if (time_since_turn <= 0 && central_dist < MIN_DIST_CIRCLE_THRESHOLD) {
                moves[MOVING_ROBOT].dir = 2;
                moves[MOVING_ROBOT].dist = 0;
                time_since_turn = 2;
            } else {
                moves[MOVING_ROBOT].dir = -1;
                moves[MOVING_ROBOT].dist = 1;
                time_since_turn--;
            }
            // if (orbit_phase == ORBIT_TURN) {
            //     /* Turn CW 60° (1 PHT step). dir=2 → dir < 3 → turn_cw() in sub */
            //     moves[MOVING_ROBOT].dir = 2;
            //     moves[MOVING_ROBOT].dist = 0;
            //     orbit_phase = ORBIT_MOVE;
            // } else if (orbit_phase == ORBIT_MOVE) {
            //     /* Move forward to arc along orbit path */
            //     moves[MOVING_ROBOT].dir = 0;
            //     moves[MOVING_ROBOT].dist = 1;
            //     orbit_phase = ORBIT_TURN;
            // }
        }

        /* moves[CENTRAL_ROBOT] stays zero — central robot does not move */

        uint8_t move_buf[1 + NUM_ROBOTS_RECV * 4];
        move_buf[0] = CMD_MOVE_DATA;
        for (uint8_t i = 0; i < NUM_ROBOTS_RECV; i++) {
            move_buf[1 + i * 4 + 0] = (uint8_t) (moves[i].dir & 0xFF);
            move_buf[1 + i * 4 + 1] = (uint8_t) (moves[i].dir >> 8);
            move_buf[1 + i * 4 + 2] = (uint8_t) (moves[i].dist & 0xFF);
            move_buf[1 + i * 4 + 3] = (uint8_t) (moves[i].dist >> 8);
        }

        if (spi_send_message(move_buf, sizeof(move_buf)) != 0)
            printf("[CIRCLE] ERR: move send failed\r\n");

        _delay_ms(CYCLE_DELAY_MS);
    }
}