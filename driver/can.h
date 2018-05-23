#ifndef _CAN_H_
#define _CAN_H_

#include <stdint.h>

int can_init(void);

void can_reset(void);

extern uint8_t can_tx_buffer[16];
extern volatile int can_tx_count;

int can_transmit_commit_buffer(void);

// CAN_RX_QUEUE_LENGTH must be 2^n-1 !!!
#define CAN_RX_QUEUE_LENGTH 15
extern uint8_t *can_rx_buffer;

int can_receive_shift_buffer(void);

#define FIFO_0 0
#define FIFO_1 1
#define FILTER_ID_N 14

typedef struct {
    int active;
    uint32_t filter;
    uint32_t mask;
    int fifo;
} can_filter_t;

int can_filter_set(uint32_t filter_id, const can_filter_t *finfo);

int can_filter_get(uint32_t filter_id, can_filter_t *finfo);

#endif
