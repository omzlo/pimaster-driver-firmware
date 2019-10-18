#include <stm32f0xx.h>
#include "spi_slave.h"
#include "usart.h"
#include "can.h"
#include "nocan.h"
#include "gpio.h"
#include "adc.h"

#define SPI_WRITE(b) (*(uint8_t *)&(SPI1->DR) = (uint8_t)(b))

#define SPI_READ() ((uint8_t)(SPI1->DR))


/*****/


void spi_slave_configure_EXTI(void)
{
  /* (1) PA4 as source input */
  /* (2) unmask port 0 */
  /* (3) Rising edge */
  /* (4) Set priority */
  /* (5) Enable EXTI0_1_IRQn */

  SYSCFG->EXTICR[1] = (SYSCFG->EXTICR[1] & ~SYSCFG_EXTICR2_EXTI4) | SYSCFG_EXTICR2_EXTI4_PA; /* (1) */ 
  //  SYSCFG->EXTICR[0] => PA0, PB0, PC0, ... PF0 as specified in bits [0:3] of  SYSCFG_EXTICR1 
  //                    => PA1, PB1, PC1, ... PF1 as specified in bits [4:7]
  //                    ...
  //                    => PA3, PB3, PC3, ... PF3 as specified in bits [12:15]
  //
  //  SYSCFG->EXTICR[1] => PA4, PB4, PC4, ... PF4 as specified in bits [0:3] of  SYSCFG_EXTICR2 
  //  
    
  
  
  EXTI->IMR |= EXTI_IMR_MR4; /* (2) */
  // Interrupt request form line _0_ is unmasked (1)
  // SYSCFG->EXTICR selects the letter PA, PB, ... PF and here we select one or more pins 
  // for the letter  (incorrect)

  EXTI->RTSR |= EXTI_RTSR_TR4; /* (3) */ 
  // Rising edge on line _0_
  EXTI->FTSR |= EXTI_FTSR_TR4;
  // EXTI->FTSR for falling edge

  NVIC_SetPriority(EXTI4_15_IRQn, 0); /* (4) */ 
  // EXTI0_1 covers interrupts on pins Px0 and Px1
  // EXTI2_3 covers interrupts on pins Px2 and Px3
  // EXTI4_15 coverts interrupts on pins Px4, Px5, Px6, ..., Px15
  // Priority 0 is the highest (as here), priority 3 is the lowest 
  //=// NVIC_EnableIRQ(EXTI0_1_IRQn); /* (5) */ 
  NVIC_EnableIRQ(EXTI4_15_IRQn); /* (5) */ 
}

#define SPI_OK_BYTE 0x80
#define SPI_MORE_BYTE 0xA0
#define SPI_ERR_BYTE 0xFF

static uint8_t opcode;
static int offset;

void EXTI4_15_IRQHandler(void)
{
    EXTI->PR |= EXTI_PR_PR4;
    if ((GPIOA->IDR & GPIO_IDR_4)==0)
    {
        SPI1->CR1 &= ((uint16_t)0xFEFF);
        offset = -1;
    }
    else 
    { 
        while ((SPI1->SR & SPI_SR_RXNE) != 0) opcode = SPI_READ();
        SPI1->CR1 |= SPI_CR1_SSI;
    }
}

void SPI1_IRQHandler(void)
{
    uint8_t param;
    static void *jump_table[16] = {
        &&op_null,              // 0
        &&op_reset,             // 1
        &&op_device_info,       // 2
        &&op_power_level,       // 3
        &&op_set_power,         // 4
        &&op_set_can_res,       // 5
        &&op_get_status,        // 6
        &&op_store_data,        // 7
        &&op_send_req,          // 8
        &&op_fetch_data,        // 9 
        &&op_recv_ack,          // 10
        &&op_set_current_limit, // 11
        &&op_bootloader_mode,   // 12
        &&op_null,              // 13
        &&op_null,              // 14
        &&op_null,              // 15
    };
    static void *start_jump_table[16] = {
        &&op_start_null,                // 0
        &&op_start_reset,               // 1
        &&op_start_device_info,         // 2
        &&op_start_power_level,         // 3
        &&op_start_set_power,           // 4
        &&op_start_set_can_res,         // 5
        &&op_start_get_status,          // 6
        &&op_start_store_data,          // 7
        &&op_start_send_req,            // 8
        &&op_start_fetch_data,          // 9 
        &&op_start_recv_ack,            // 10
        &&op_start_set_current_limit,   // 11
        &&op_start_bootloader_mode,     // 12
        &&op_start_null,                // 13
        &&op_start_null,                // 14
        &&op_start_null,                // 15
    };



    //if ((SPI1->SR & SPI_SR_RXNE) == SPI_SR_RXNE)
    //{
        if (offset < 0)
        {
            opcode = SPI_READ() & 15;
            offset = 0;
            goto *start_jump_table[opcode];
        }
        else 
        {
            param = SPI_READ();
            offset++; 
            goto *jump_table[opcode];
        }
    //}
    //return;

    /********** 0 **********/
op_start_null:      
op_null:            
    SPI_WRITE(SPI_ERR_BYTE);
    return;

    /********** 1 **********/
op_start_reset:   
    SPI_WRITE(SPI_MORE_BYTE);
    return;
 
op_reset:          
    if (offset==1) {
        if (param == 1)
        {
            SPI_WRITE(SPI_OK_BYTE);
            can_reset();
            nocan_init();
            return;
        } 
        if (param == 2)
        {
            SPI_WRITE(SPI_OK_BYTE);
            NVIC_SystemReset();
            return;
        }
    }
    SPI_WRITE(SPI_ERR_BYTE);
    return;

    /********** 2 **********/
op_start_device_info:
op_device_info: 
    // 4 bytes of signature, 2 bytes of version, 12 bytes of chip id = 18 bytes
    if (offset<sizeof(nocan_registers_t))
    {
        SPI_WRITE(((uint8_t*)&NOCAN_REGS)[offset]);
        return;
    }
    SPI_WRITE(SPI_ERR_BYTE);
    return;

    /********** 3 **********/
op_start_power_level:   
op_power_level:   
    if (offset<14) 
    {
        SPI_WRITE(((uint8_t*)&NOCAN_REGS.STATUS)[offset]);
        return;
    }
    SPI_WRITE(SPI_ERR_BYTE);
    return;

    /********** 4 **********/
op_start_set_power:
    SPI_WRITE(SPI_MORE_BYTE);
    return;

op_set_power:
    if (offset==1) 
    {
        SPI_WRITE(SPI_OK_BYTE);
        if (param==0)
            nocan_power_off();
        else
            nocan_power_on();
        return;
    }
    SPI_WRITE(SPI_ERR_BYTE);
    return;

    /********** 5 **********/
op_start_set_can_res:
op_set_can_res: 
    // TODO:
    SPI_WRITE(SPI_ERR_BYTE);
    return;

    /********** 6 **********/
op_start_get_status:
op_get_status:   
    SPI_WRITE(NOCAN_REGS.STATUS);
    return;

    /********** 7 **********/
op_start_store_data:    
    //if (can_tx_current_buffer == 0)
    //{
    //    SPI_WRITE(SPI_ERR_BYTE);
    //    opcode = 0;
    //}
    //else 
    //{
        SPI_WRITE(SPI_MORE_BYTE);
    //}
    return;

op_store_data: 
    //if (can_tx_current_buffer!=0 && offset<16) 
    if (offset<16)
    {
        //SPI_WRITE(SPI_OK_BYTE);
        can_tx_buffer[offset-1] = param;
        return;
    } 
    SPI_WRITE(SPI_ERR_BYTE);
    return;

    /********** 8 **********/
op_start_send_req:
    SPI_WRITE(SPI_OK_BYTE);
    if (can_tx_count>=2)
        gpio_clear_tx_int();    // preemptive
    //can_transmit_push_buffer();
    can_transmit_commit_buffer();
    return;

op_send_req:
    SPI_WRITE(SPI_ERR_BYTE);
    return;

    /********** 9 **********/
op_start_fetch_data:
op_fetch_data:
    if (offset<16) 
    {
        SPI_WRITE(can_rx_buffer[offset]);
        return;
    }
    SPI_WRITE(SPI_ERR_BYTE);
    return;

    /********** 10 **********/
op_start_recv_ack:
    SPI_WRITE(SPI_OK_BYTE);
    gpio_set_rx_int();    // preemptive
    can_receive_shift_buffer();
    return;

 op_recv_ack:     
    SPI_WRITE(SPI_ERR_BYTE);
    return;

    /********** 11 ***********/
op_start_set_current_limit:
    SPI_WRITE(SPI_MORE_BYTE);
    return;

op_set_current_limit:
    if (offset==1) {
        SPI_WRITE(SPI_MORE_BYTE);
        NOCAN_REGS.LEVELS[4] = (uint16_t)param<<8;
        return;
    }
    if (offset==2) {
        SPI_WRITE(SPI_OK_BYTE);
        NOCAN_REGS.LEVELS[4] |= (uint16_t)param;
        adc_watchdog_update();
        return;
    }
    SPI_WRITE(SPI_ERR_BYTE);
    return;
    
    /********** 12 ***********/
op_start_bootloader_mode:
    SPI_WRITE(SPI_MORE_BYTE);
    return;

op_bootloader_mode:
    if (offset==1) {
        if (param == 0x17)
        {
            SPI_WRITE(SPI_OK_BYTE);
            rtc_backup_write(0, 0xdeadbeef);
            NVIC_SystemReset();
            return;
        }
    }
    SPI_WRITE(SPI_ERR_BYTE);
    return; 
}


int spi_slave_init()
{

    /* Enable the peripheral clock of GPIOA */
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    /* Select AF mode (10) on PA4, PA5, PA6, PA7 */
    GPIOA->MODER = (GPIOA->MODER 
            & ~(GPIO_MODER_MODER5 | GPIO_MODER_MODER6 | GPIO_MODER_MODER7))
        | (GPIO_MODER_MODER5_1| GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1);

    /* AF0 for SPI1 signals */
    GPIOA->AFR[1] = (GPIOA->AFR[1] &
            ~(GPIO_AFRL_AFRL5 | GPIO_AFRL_AFRL6 | GPIO_AFRL_AFRL7)); 


    /* Enable input for GPIO PA4 */
    // Nothing to do since default state
    GPIOA->MODER &=  ~(GPIO_MODER_MODER4); 


    /* Enable the peripheral clock SPI1 */
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    /* Configure SPI1 in slave */
    /* nSS hard, slave, CPOL and CPHA at zero (rising first edge) */
    /* (1) RXNE IT, 8-bit Rx fifo */
    /* (2) Enable SPI1 */
    SPI1->CR2 = SPI_CR2_RXNEIE                          // Enable RX buffer not empty interrupt
        | SPI_CR2_FRXTH                                 // RXNE event generated if FIFO level = 8 bits
        | SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0    // DataSize=8 bits
        ; /* (1) */
    SPI1->CR1 |= (SPI_CR1_SPE                            // SPI enable
        | SPI_CR1_SSM)                                  // Software Slave Select
        ; /* (2) */


    /* Configure IT */
    /* (3) Set priority for SPI1_IRQn */
    /* (4) Enable SPI1_IRQn */
    NVIC_SetPriority(SPI1_IRQn, 0); /* (3) */
    NVIC_EnableIRQ(SPI1_IRQn); /* (4) */

    //spi_slave_configure_DMA();
    spi_slave_configure_EXTI();
    return 0;
}

