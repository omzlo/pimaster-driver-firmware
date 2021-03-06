#ifndef _GPIO_H_
#define _GPIO_H_

#define HIGH 1
#define LOW  0

void gpio_init(void);

#define PWR_IN_Pin      0
#define PWR_DEN_Pin     1
#define LED_RED_Pin     2
#define LED_YEL_Pin     3
#define RX_INT_Pin      10
#define TX_INT_Pin      11
#define CAN_RES_Pin     12

#define GPIOB_SET(pin)      (GPIOB->BSRR = 1<<(pin))
#define GPIOB_CLEAR(pin)    (GPIOB->BRR = 1<<(pin))

#define gpio_set_yellow_led() GPIOB_SET(LED_YEL_Pin)
#define gpio_clear_yellow_led() GPIOB_CLEAR(LED_YEL_Pin)

//void gpio_set_LED_YEL(int val);

#define gpio_set_red_led() GPIOB_SET(LED_RED_Pin)
#define gpio_clear_red_led() GPIOB_CLEAR(LED_RED_Pin)

//void gpio_set_LED_RED(int val);

#define gpio_set_rx_int() GPIOB_SET(RX_INT_Pin)
#define gpio_clear_rx_int() GPIOB_CLEAR(RX_INT_Pin)

//void gpio_set_RX_INT(int val);

#define gpio_set_tx_int() GPIOB_SET(TX_INT_Pin)
#define gpio_clear_tx_int() GPIOB_CLEAR(TX_INT_Pin)

//void gpio_set_TX_INT(int val);

#define gpio_set_pwr_in() GPIOB_SET(PWR_IN_Pin)
#define gpio_clear_pwr_in() GPIOB_CLEAR(PWR_IN_Pin)

//void gpio_set_PWR_IN(int val);

#define gpio_set_pwr_den() GPIOB_SET(PWR_DEN_Pin)
#define gpio_clear_pwr_den() GPIOB_CLEAR(PWR_DEN_Pin)

//void gpio_set_PWR_DEN(int val);

#define gpio_set_can_res() GPIOB_SET(CAN_RES_Pin)
#define gpio_clear_can_res() GPIOB_CLEAR(CAN_RES_Pin)

//void gpio_set_CAN_RES(int val);

#endif
