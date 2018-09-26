#include <stm32f0xx.h>
#include <stdint.h>
#include "nocan.h"
#include "gpio.h"
#include "can.h"
#include "spi_slave.h"
#include "version.h"

nocan_registers_t NOCAN_REGS;

const uint8_t *CHIP_UDID = (const uint8_t *)0x1FFFF7AC;

void nocan_init(void) 
{
    int i;

    // reset registers
    for (i=0; i<sizeof(NOCAN_REGS);i++) ((uint8_t *)&NOCAN_REGS)[i]=0;

    NOCAN_REGS.SIGNATURE[0] = 'C';
    NOCAN_REGS.SIGNATURE[1] = 'A';
    NOCAN_REGS.SIGNATURE[2] = 'N';
    NOCAN_REGS.SIGNATURE[3] = '0';
    NOCAN_REGS.VERSION_MAJOR = PIMASTER_VERSION_MAJOR;
    NOCAN_REGS.VERSION_MINOR = PIMASTER_VERSION_MINOR;
    for (i=0;i<12;i++) NOCAN_REGS.CHIP_UDID[i]=CHIP_UDID[i];
    NOCAN_REGS.GUARD = 0xBEEF;

    NOCAN_REGS.LEVELS[3] = *((uint16_t *)(0x1FFFF7BA));   // VREFINT_CAL for STM32F072
    NOCAN_REGS.LEVELS[4] = 3096;                          // limit for trigger
}


void nocan_power_on(void)
{
    NOCAN_STATUS_CLEAR(NOCAN_STATUS_FAULT);
    NOCAN_STATUS_SET(NOCAN_STATUS_POWERED);
    gpio_set_yellow_led();
    gpio_set_pwr_den();
    gpio_set_pwr_in();
    ADC1->CFGR1 |= ADC_CFGR1_AWDEN;     // enable analog watchdog
}

void nocan_power_off(void)
{
    gpio_clear_pwr_in();
    ADC1->CFGR1 &= ~ADC_CFGR1_AWDEN;    // disable analog watchdog
    gpio_clear_pwr_den();
    gpio_clear_yellow_led();
    NOCAN_STATUS_CLEAR(NOCAN_STATUS_POWERED);
}


#if 0
int nocan_status_set(uint8_t status)
{
    __disable_irq();
    NOCAN->STATUS |= (1<<status);
    __enable_irq();

    switch (status) {
        case NOCAN_STATUS_RX_PENDING:       /* 0 */
            gpio_set_RX_INT(LOW);
            return 0;
        case NOCAN_STATUS_TX_PENDING:       /* 1 */
            gpio_set_TX_INT(LOW);
            return 0;
        case NOCAN_STATUS_UNDEF_0:          /* 2 */
        case NOCAN_STATUS_UNDEF_1:          /* 3 */
            return -1;
        case NOCAN_STATUS_ERROR:            /* 4 */
        case NOCAN_STATUS_FAULT:            /* 5 */
            gpio_set_LED_RED(HIGH);
            return 0;
        case NOCAN_STATUS_POWERED:          /* 6 */
            gpio_set_LED_YEL(HIGH);
            gpio_set_PWR_DEN(HIGH);
            gpio_set_PWR_IN(HIGH);
            ADC1->CFGR1 |= ADC_CFGR1_AWDEN;     // enable analog watchdog
            return 0;
        case NOCAN_STATUS_CAN_RES:          /* 7 */
            gpio_set_CAN_RES(LOW); // enable is LOW (p-ch mosfet)
            return 0;
    }
    return -1;
}

int nocan_status_clear(uint8_t status)
{
    __disable_irq();
    NOCAN->STATUS &= ~(1<<status);
    __enable_irq();

    switch (status) {
        case NOCAN_STATUS_RX_PENDING:       /* 0 */
            gpio_set_RX_INT(HIGH);
            return 0;
        case NOCAN_STATUS_TX_PENDING:       /* 1 */
            gpio_set_TX_INT(HIGH);
            return 0;
        case NOCAN_STATUS_UNDEF_0:          /* 2 */
        case NOCAN_STATUS_UNDEF_1:          /* 3 */
            return -1;
        case NOCAN_STATUS_ERROR:            /* 4 */
        case NOCAN_STATUS_FAULT:            /* 5 */
            if ((NOCAN->STATUS & (NOCAN_STATUS_FAULT|NOCAN_STATUS_ERROR))==0)
                gpio_set_LED_RED(LOW);
            return 0;
        case NOCAN_STATUS_POWERED:          /* 6 */
            ADC1->CFGR1 &= ~ADC_CFGR1_AWDEN;    // disable analog watchdog
            gpio_set_LED_YEL(LOW);
            gpio_set_PWR_DEN(LOW);
            gpio_set_PWR_IN(LOW);
            return 0;
        case NOCAN_STATUS_CAN_RES:          /* 7 */
            gpio_set_CAN_RES(HIGH); // disable is HIGH (p-ch mosfet)
            return 0;
    }
    return -1;
}

#endif
