#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <string.h>
#include "spi_comm.h"
#include "uart.h"

#define CMD_START_COLL        0xA1
#define CMD_MOVE_DATA         0xB1
#define CYCLE_DELAY_MS        200
#define ADC_DIF_THRESHOLD     20
#define NUM_ROBOTS_RECV       2
#define NUM_PHT               6
#define DATA_LEN              (NUM_ROBOTS_RECV * NUM_PHT * 2)

#define CLOSE_ENOUGH_THRESHOLD 900
#define MOVING_ROBOT           1   /* robot 1 moves toward robot 0 */
#define CENTRAL_ROBOT          0   /* robot 0 stays still */

typedef struct {
    uint16_t dir;
    uint16_t dist;   /* 0 = stay, nonzero = move forward */
} MoveCmd;

int main(void)
{
    uart_init();
    spi_master_init();

    printf("[APPROACH] Ready\r\n");

    uint8_t tx_buf[] = { CMD_START_COLL };
    uint8_t rx_buf[SPI_MAX_PAYLOAD];
    uint8_t rx_len = 0;

    int docked = 0;

    while (1)
    {
        if (spi_send_message(tx_buf, sizeof(tx_buf)) != 0) {
            printf("[APPROACH] ERR: spi send failed\r\n");
            continue;
        }

        _delay_ms(1000);

        if (spi_receive_response(rx_buf, &rx_len) != 0) {
            printf("[APPROACH] ERR: spi receive failed\r\n");
            continue;
        }

        uint8_t  robot_count = rx_buf[0];
        uint8_t *p           = rx_buf + 1;

        if (robot_count < NUM_ROBOTS_RECV) {
            printf("[APPROACH] Only saw %d robots. Repolling.\r\n", robot_count);
            continue;
        }

        /*
         * adj_matrix[r][emitter][0] = max ADC across all 6 PHTs (distance proxy)
         * adj_matrix[r][emitter][1] = PHT index of that max reading
         * Higher ADC = more IR light received = closer to emitter.
         */
        uint16_t adj_matrix[NUM_ROBOTS_RECV][NUM_ROBOTS_RECV][2];
        memset(adj_matrix, 0, sizeof(adj_matrix));

        for (uint8_t r = 0; r < robot_count; r++)
        {
            uint8_t addr = *p++;

            for (uint8_t emitter = 0; emitter < NUM_ROBOTS_RECV; emitter++)
            {
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

                if (max_val - avg_dist > ADC_DIF_THRESHOLD) {
                    if (adj_matrix[emitter][r][0] != 0)
                        max_val = ((int32_t)adj_matrix[emitter][r][0] + max_val) / 2;

                    adj_matrix[r][emitter][0] = (uint16_t)max_val;
                    adj_matrix[r][emitter][1] = (uint16_t)max_pht;
                }

                (void)addr;
            }
        }
        printf("---\r\n");

        /* Distance robot 1 reads from robot 0's emitter — our approach metric */
        uint16_t dist_to_target = adj_matrix[MOVING_ROBOT][CENTRAL_ROBOT][0];
        printf("[APPROACH] Robot %d dist=%u\r\n", MOVING_ROBOT, dist_to_target);

        MoveCmd moves[NUM_ROBOTS_RECV];
        memset(moves, 0, sizeof(moves));

        if (!docked && dist_to_target > CLOSE_ENOUGH_THRESHOLD) {
            printf("[APPROACH] Docked!\r\n");
            docked = 1;
        }

        if (!docked && dist_to_target > 0) {
            /* Robot 1 moves forward; robot 0 stays (dist=0) */
            moves[MOVING_ROBOT].dir  = 1;   /* positive dir → move_forward() in sub */
            moves[MOVING_ROBOT].dist = 1;
        }
        /* moves[CENTRAL_ROBOT] stays zero — robot 0 does not move */

        uint8_t move_buf[1 + NUM_ROBOTS_RECV * 4];
        move_buf[0] = CMD_MOVE_DATA;
        for (uint8_t i = 0; i < NUM_ROBOTS_RECV; i++) {
            move_buf[1 + i*4 + 0] = (uint8_t)(moves[i].dir  & 0xFF);
            move_buf[1 + i*4 + 1] = (uint8_t)(moves[i].dir  >> 8);
            move_buf[1 + i*4 + 2] = (uint8_t)(moves[i].dist & 0xFF);
            move_buf[1 + i*4 + 3] = (uint8_t)(moves[i].dist >> 8);
        }

        if (spi_send_message(move_buf, sizeof(move_buf)) != 0)
            printf("[APPROACH] ERR: move send failed\r\n");

        _delay_ms(CYCLE_DELAY_MS);
    }
}
