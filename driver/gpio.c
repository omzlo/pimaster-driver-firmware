#include <stm32f0xx.h>
#include <stm32f0_helpers.h>

/*
 * RX_INT: PB10
 * TX_INT: PB11
 * CAN_RES: PB12
 * LED_YEL: PB3 
 * LED_RED: PB2
 * PWR_IN: PB0
 * PWR_DEN: PB1
 */

#define PWR_IN_Pin      0
#define PWR_DEN_Pin     1
#define LED_RED_Pin     2
#define LED_YEL_Pin     3
#define RX_INT_Pin      10
#define TX_INT_Pin      11
#define CAN_RES_Pin     12


void gpio_init(void)
{
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    GPIO_CONFIGURE_OUTPUT(GPIOB, PWR_IN_Pin, GPIO_SPEED_LOW, GPIO_PUSH_PULL); 
    GPIO_CONFIGURE_OUTPUT(GPIOB, PWR_DEN_Pin, GPIO_SPEED_LOW, GPIO_PUSH_PULL); 
    GPIO_CONFIGURE_OUTPUT(GPIOB, LED_RED_Pin, GPIO_SPEED_LOW, GPIO_PUSH_PULL); 
    GPIO_CONFIGURE_OUTPUT(GPIOB, LED_YEL_Pin, GPIO_SPEED_LOW, GPIO_PUSH_PULL); 
    GPIO_CONFIGURE_OUTPUT(GPIOB, RX_INT_Pin, GPIO_SPEED_LOW, GPIO_PUSH_PULL); 
    GPIO_CONFIGURE_OUTPUT(GPIOB, TX_INT_Pin, GPIO_SPEED_LOW, GPIO_PUSH_PULL); 
    GPIO_CONFIGURE_OUTPUT(GPIOB, CAN_RES_Pin, GPIO_SPEED_LOW, GPIO_PUSH_PULL); 
}
/*
#define gpio_set_B(pin, val) do { if (val) GPIOB->BSRR = 1<<(pin); else GPIOB->BRR = 1<<(pin); } while(0)

void gpio_set_LED_YEL(int val)
{
    gpio_set_B(LED_YEL_Pin, val);
}

void gpio_set_LED_RED(int val)
{
    gpio_set_B(LED_RED_Pin, val);
}

void gpio_set_RX_INT(int val)
{
    gpio_set_B(RX_INT_Pin,val);
}

void gpio_set_TX_INT(int val)
{
    gpio_set_B(TX_INT_Pin,val);
}

void gpio_set_PWR_IN(int val)
{
    gpio_set_B(PWR_IN_Pin,val);
}

void gpio_set_PWR_DEN(int val)
{
    gpio_set_B(PWR_DEN_Pin,val);
}

void gpio_set_CAN_RES(int val)
{
    gpio_set_B(CAN_RES_Pin,val);
}
*/
