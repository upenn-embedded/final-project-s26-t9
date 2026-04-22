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
#define ADC_DIF_THRESHOLD 20
#define NUM_ROBOTS_RECV   2
#define NUM_PHT           6
#define DATA_LEN          (NUM_ROBOTS_RECV * NUM_PHT * 2)
#define NUM_ROBOTS        2

#define MIN_ADC_CIRCLE_THRESHOLD 200
#define MAX_ADC_CIRCLE_THRESHOLD 300

#define STARTING_STATE 0
#define CIRCLING_CW_STATE 1
#define CIRCLING_CCW_STATE 2
#define FINALIZING_STATE 3

#define BACK_PHT 3

/*
 * Movement instruction for one robot.
 * dir  = PHT index (0?5) of the strongest return ? points toward target
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

    int central_robot = -1;
    int current_moving_robot = -1;
    int curr_goal_side = -1;
    int done_robots[NUM_ROBOTS];
    for (int i = 0; i < NUM_ROBOTS; i++) {
        done_robots[i] = 0;
    }
    int curr_move_state = STARTING_STATE;
    
    while (1)
    {
        /* ?? Send START_COLL ?? */
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
        
        if (robot_count < NUM_ROBOTS) {
            printf("Only saw %d robots. Repolling.\n", robot_count);
            continue;
        }

        /*
         * adj_matrix[r][emitter][0] = best ADC distance
         * adj_matrix[r][emitter][1] = PHT index of that reading
         * Indexed by robot slot r (0..robot_count-1), not raw address byte.
         */
        uint16_t adj_matrix[NUM_ROBOTS_RECV][NUM_ROBOTS_RECV][2];
        memset(adj_matrix, 0, sizeof(adj_matrix));

        /* ?? Parse and print in one pass ?? */
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

        /* ?? Build movement commands ?? */
        /*
         * Robot 1 is stationary.
         * Robot 0 must turnvg to face robot 1 and drive dist_to_move.
         *
         * adj_matrix[0][1] = what robot 0 saw when robot 1 was emitting
         *   [0] = ADC strength (proxy for distance)
         *   [1] = PHT index    (proxy for direction)
         */
        MoveCmd moves[NUM_ROBOTS_RECV];
        memset(moves, 0, sizeof(moves));
        
        printf("---\r\n");

        move_start:      
        // Get most central robot if none
        if (central_robot == -1) {
            int max_num_neighbors = -1;
            int max_total_dist = 0;
            int max_robot_num = -1;
            for (int i = 0; i < robot_count; i++) {
                int curr_num_neighbors = 0;
                int curr_total_dist = 0;
                for (int j = 0; j < robot_count; j++) {
                    if (adj_matrix[i][j][0] != 0) {
                        curr_num_neighbors++;
                        curr_total_dist += adj_matrix[i][j][0];
                    }
                }
                
                if (curr_num_neighbors > max_num_neighbors || (
                        curr_num_neighbors == max_num_neighbors && max_total_dist < curr_total_dist)) {
                    max_num_neighbors = curr_num_neighbors;
                    max_total_dist = curr_total_dist;
                    max_robot_num = i;
                } 
            }
            
            if (max_robot_num == -1) {
                printf("[MAIN ATMEGA] All robots isolated. Cannot find central robot. Repolling.\n");
                continue;
            }
            
            central_robot = max_robot_num;
            done_robots[central_robot] = 1;
            printf("[MAIN ATMEGA] Central robot set to %d\n", central_robot);
        }
        
        // Get next robot to move if none
        if (current_moving_robot == -1) {
            int closest_robot = -1;
            int closest_side = 0;
            int max_dist = -1;
            for (int i = 0; i < robot_count; i++) {
                if (done_robots[i] || !adj_matrix[central_robot][i][0])  continue;
                
                // test distance to side 1
                // test distance to side 2
                // test distance to side 3
            }
            
            if (closest_robot == -1) {
                printf("[MAIN ATMEGA] Failed to get next moving robot. Repolling.\n");
                continue;
            }
            
            done_robots[central_robot] = 1;
            current_moving_robot = closest_robot;
            curr_goal_side = closest_side;
            curr_move_state = STARTING_STATE;
        }
        
        if (curr_move_state == STARTING_STATE) {
            // determine cw or ccw based on PHT
            // turn towards the corresponding adc
            // if too close, move a bit out as well. othwerise, move a bit in
            curr_move_state = CIRCLING_CW_STATE;
        } else if (curr_move_state == CIRCLING_CW_STATE) {
            int adc_dist = adj_matrix[central_robot][current_moving_robot][0];
            int adc_pht = adj_matrix[central_robot][current_moving_robot][0];
            
            // If close enough, turn and move backwards
            if (adc_pht == curr_goal_side) {
                curr_move_state = FINALIZING_STATE;
                moves[current_moving_robot].dir = BACK_PHT;
                moves[current_moving_robot].dist = (uint16_t) (-adc_dist);
            } else {
                if (adc_dist < MIN_ADC_CIRCLE_THRESHOLD) {
                    // turn slightly ccw
                } else if (adc_dist > MAX_ADC_CIRCLE_THRESHOLD) {
                    // turn slightly cw
                }
                
                moves[current_moving_robot].dist = 1;
            }
        } else if (curr_move_state == CIRCLING_CCW_STATE) {
            int adc_dist = adj_matrix[central_robot][current_moving_robot][0];
            int adc_pht = adj_matrix[central_robot][current_moving_robot][0];
            
            // If close enough, turn and move backwards
            if (adc_pht == curr_goal_side) {
                curr_move_state = FINALIZING_STATE;
                moves[current_moving_robot].dir = BACK_PHT;
                moves[current_moving_robot].dist = (uint16_t) (-adc_dist);
            } else {
                if (adc_dist < MIN_ADC_CIRCLE_THRESHOLD) {
                    // turn slightly cw
                } else if (adc_dist > MAX_ADC_CIRCLE_THRESHOLD) {
                    // turn slightly ccw
                }
                
                moves[current_moving_robot].dist = 1;
            }
        } else if (curr_move_state == FINALIZING_STATE) {
            // Move backwards, and turn back towards right direction if necessary
            int adc_dist = adj_matrix[current_moving_robot][central_robot][0];
            int adc_pht = adj_matrix[current_moving_robot][central_robot][0];
            
            if (adc_dist > 900) {
                printf("[MAIN ATMEGA] Finished robot!\n");
                current_moving_robot = -1;
                // update this side to be marked as done
                goto move_start;
            }
            
            moves[current_moving_robot].dist = -adc_dist;
            if (adc_pht != curr_goal_side) {
                moves[current_moving_robot].dir = BACK_PHT;
            } else {
                moves[current_moving_robot].dir = (uint16_t) (-1);
            }
        }
        
        // Determine how to move the curr robot
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