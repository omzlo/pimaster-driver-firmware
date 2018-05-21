#ifndef _NOCAN_H_
#define _NOCAN_H_

#include <stdint.h>

void nocan_init(void);

void nocan_power_on(void);

void nocan_power_off(void);


#define NOCAN_STATUS_SET(status)    do { \
    __disable_irq(); \
    NOCAN_REGS.STATUS |= (status); \
    __enable_irq(); \
} while (0)

#define NOCAN_STATUS_CLEAR(status)  do { \
    __disable_irq(); \
    NOCAN_REGS.STATUS &= ~(status); \
    __enable_irq(); \
} while (0)


#define NOCAN_STATUS_RX_PENDING     0x01       // 0x01
#define NOCAN_STATUS_TX_PENDING     0x02       // 0x02
#define NOCAN_STATUS_UNDEF_0        0x04
#define NOCAN_STATUS_UNDEF_1        0x08
#define NOCAN_STATUS_ERROR          0x10
#define NOCAN_STATUS_FAULT          0x20
#define NOCAN_STATUS_POWERED        0x40
#define NOCAN_STATUS_CAN_RES        0x80

typedef struct __attribute__((packed)) {
    uint8_t SIGNATURE[4];       // 0
    uint8_t VERSION_MAJOR;      // 4
    uint8_t VERSION_MINOR;      // 5
    uint8_t CHIP_UDID[12];      // 6
    volatile uint8_t STATUS;    // 18, status register
    uint8_t RESERVED;           // 19
    uint16_t LEVELS[6];         // 20, Voltage(0), Current(1), VRef(2),  VRefCal(3), Watchdog limit(4), FaultLevel(5).
    uint16_t GUARD;             // 32
    uint32_t COUNTER;           // 34
                                // --
                                // 36
} nocan_registers_t;

extern nocan_registers_t NOCAN_REGS;

#endif
