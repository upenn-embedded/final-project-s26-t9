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
#define ADC_DIF_THRESHOLD 20
#define NUM_ROBOTS_RECV   2
#define NUM_PHT           6
#define DATA_LEN          (NUM_ROBOTS_RECV * NUM_PHT * 2)

#define MOVING_ROBOT  1
#define CENTRAL_ROBOT 0
#define TARGET_PHT    3 /* back PHT — we want this to face the other robot */

/*
 * Tuning: 250 ms ≈ 90°, so 1 PHT sector (60°) ≈ 167 ms.
 * dist units = 10 ms, so MS_PER_PHT / 10 = dist units per PHT step.
 */
#define MS_PER_PHT 167
#define DIST_SCALE 10

typedef struct {
    uint16_t dir;
    uint16_t dist;
} MoveCmd;

int
main(void) {
    uart_init();
    spi_master_init();

    printf("[ALIGN] Ready. Aligning robot %d back edge toward robot %d.\r\n", MOVING_ROBOT, CENTRAL_ROBOT);

    uint8_t tx_buf[] = {CMD_START_COLL};
    uint8_t rx_buf[SPI_MAX_PAYLOAD];
    uint8_t rx_len = 0;

    while (1) {
        if (spi_send_message(tx_buf, sizeof(tx_buf)) != 0) {
            printf("[ALIGN] ERR: spi send failed\r\n");
            continue;
        }

        _delay_ms(1000);

        if (spi_receive_response(rx_buf, &rx_len) != 0) {
            printf("[ALIGN] ERR: spi receive failed\r\n");
            continue;
        }

        uint8_t robot_count = rx_buf[0];
        uint8_t *p = rx_buf + 1;

        if (robot_count < NUM_ROBOTS_RECV) {
            printf("[ALIGN] Only saw %d robots. Repolling.\r\n", robot_count);
            continue;
        }

        /*
         * adj_matrix[r][emitter][0] = max ADC across all 6 PHTs
         * adj_matrix[r][emitter][1] = PHT index of that max (direction toward emitter)
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

        /* Which PHT on the moving robot is currently pointing toward the central robot? */
        uint16_t dominant_pht = adj_matrix[MOVING_ROBOT][CENTRAL_ROBOT][1];
        uint16_t adc_dist = adj_matrix[MOVING_ROBOT][CENTRAL_ROBOT][0];

        printf("[ALIGN] dominant_pht=%u adc=%u target_pht=%u\r\n", dominant_pht, adc_dist, TARGET_PHT);

        MoveCmd moves[NUM_ROBOTS_RECV];
        memset(moves, 0, sizeof(moves));

        if (adc_dist == 0) {
            printf("[ALIGN] No signal from central robot. Repolling.\r\n");
            _delay_ms(CYCLE_DELAY_MS);
            continue;
        }

        /*
         * How far does robot 1 need to turn so its PHT 3 (back) faces robot 0?
         *
         * diff = steps needed clockwise from dominant_pht to TARGET_PHT.
         * Normalized to -2..3 so we always take the shorter arc.
         *   diff > 0 → turn CW (dir=2)
         *   diff < 0 → turn CCW (dir=3)
         *   diff == 0 → already aligned
         */
        int8_t diff = (int8_t) TARGET_PHT - (int8_t) dominant_pht;
        if (diff > 3)
            diff -= 6;
        if (diff <= -3)
            diff += 6;

        if (diff == 0) {
            printf("[ALIGN] Back edge aligned! PHT 3 facing central robot.\r\n");
            /* Send no-op so sub doesn't move this cycle */
        } else {
            uint16_t steps = (uint16_t) (diff < 0 ? -diff : diff);
            uint16_t turn_ms = steps * MS_PER_PHT;
            uint16_t dist_val = (turn_ms + DIST_SCALE - 1) / DIST_SCALE; /* round up */

            moves[MOVING_ROBOT].dir = (diff > 0) ? 2 : 3; /* 2=CW, 3=CCW */
            moves[MOVING_ROBOT].dist = dist_val;

            printf("[ALIGN] Turning %s %u step(s) = %u ms (dist_val=%u)\r\n", (diff > 0) ? "CW" : "CCW", steps, turn_ms, dist_val);
        }

        /* Robot 0 stays still */
        moves[CENTRAL_ROBOT].dir = 0;
        moves[CENTRAL_ROBOT].dist = 0;

        uint8_t move_buf[1 + NUM_ROBOTS_RECV * 4];
        move_buf[0] = CMD_MOVE_DATA;
        for (uint8_t i = 0; i < NUM_ROBOTS_RECV; i++) {
            move_buf[1 + i * 4 + 0] = (uint8_t) (moves[i].dir & 0xFF);
            move_buf[1 + i * 4 + 1] = (uint8_t) (moves[i].dir >> 8);
            move_buf[1 + i * 4 + 2] = (uint8_t) (moves[i].dist & 0xFF);
            move_buf[1 + i * 4 + 3] = (uint8_t) (moves[i].dist >> 8);
        }

        if (spi_send_message(move_buf, sizeof(move_buf)) != 0)
            printf("[ALIGN] ERR: move send failed\r\n");

        _delay_ms(CYCLE_DELAY_MS);
    }
}
