#include <stm32f0xx.h>
#include "spi_slave.h"
#include "gpio.h"

#define SPI_WRITE(b) (*(uint8_t *)&(SPI1->DR) = (uint8_t)(b))

#define SPI_READ() ((uint8_t)(SPI1->DR))


/*****/

static uint8_t *spi_buf;
static unsigned spi_buf_len;
unsigned spi_write_count;

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

static int write_mode;
static int offset;
static uint8_t opcode;

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
        while ((SPI1->SR & SPI_SR_FRLVL) != 0) opcode = SPI_READ();
        SPI1->CR1 |= SPI_CR1_SSI;
        if (write_mode)
          spi_write_count++;
    }
}

void SPI1_IRQHandler(void)
{
    uint8_t param = SPI_READ();

    if (offset < 0)
    {
        offset = (param & 0x7F) % spi_buf_len;
        write_mode = (param>>7);
        if (write_mode) 
        {
            SPI_WRITE(SPI_MORE_BYTE);
        }
        else
        {
            SPI_WRITE(spi_buf[offset]);
            offset = (offset+1) % spi_buf_len;
        }
        return;
    }

    if (write_mode)
    {
        spi_buf[offset]=param;
        SPI_WRITE(SPI_MORE_BYTE);
    }
    else
    {
        SPI_WRITE(spi_buf[offset]);
    }
    offset = (offset+1) % spi_buf_len;
}


int spi_slave_init(void *buf, unsigned buf_len)
{
    spi_buf = buf;
    spi_buf_len = buf_len;
    spi_write_count = 0;

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

