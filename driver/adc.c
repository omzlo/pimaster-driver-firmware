#include "adc.h"
#include "stm32f0xx.h"
#include "gpio.h"
#include "nocan.h"


/*
 * Analog SENSE is on PA3/ADC_IN3
 * Analog VIN/11 is on PA2/ADC_IN2
 * Temp sensor is on ADC_IN16
 * VREF sensor is on ADC_IN17
 */

#define NUMBER_OF_ADC_CHANNEL 3

void adc_watchdog_update(void)
{
    ADC1->TR = (NOCAN_REGS.LEVELS[4]<<16) // Analog watchdog high limit
        | 0x0000; 
}

void adc_init(void)
{
    /*** SET CLOCK FOR ADC ***/
    /* (1) Enable the peripheral clock of the ADC */
    /* (2) Start HSI14 RC oscillator */ 
    /* (3) Wait HSI14 is ready */
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; /* (1) */
    RCC->CR2 |= RCC_CR2_HSI14ON; /* (2) */
    while ((RCC->CR2 & RCC_CR2_HSI14RDY) == 0) /* (3) */
    {
        /* For robust implementation, add here time-out management */
    }  

    /*** CALIBRATE ADC ***/
    /* (1) Ensure that ADEN = 0 */
    /* (2) Clear ADEN */ 
    /* (3) Launch the calibration by setting ADCAL */
    /* (4) Wait until ADCAL=0 */
    if ((ADC1->CR & ADC_CR_ADEN) != 0) /* (1) */
    {
        ADC1->CR &= (uint32_t)(~ADC_CR_ADEN);  /* (2) */  
    }
    ADC1->CR |= ADC_CR_ADCAL; /* (3) */
    while ((ADC1->CR & ADC_CR_ADCAL) != 0) /* (4) */
    {
        /* For robust implementation, add here time-out management */
    }  


    /*** CONFIGURE GPIO PINS FOR ADC ***/
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;  // Enabe clock GPIOA
    GPIOA->MODER |= GPIO_MODER_MODER2;  // Analog mode on pin 2 (PA2)
    GPIOA->MODER |= GPIO_MODER_MODER3;  // Analog mode on pin 3 (PA3)


    /*** ENABLE ADC ***/
    do 
    {
        /* For robust implementation, add here time-out management */
		ADC1->CR |= ADC_CR_ADEN; /* (1) */
    } while ((ADC1->ISR & ADC_ISR_ADRDY) == 0) /* (2) */;

    /*** CONFIGURE CHANNELS FOR ADC ***/
    //ADC1->CFGR2 &= ~ADC_CFGR2_CKMODE;  // Select HSI14 by writing 00 in CKMODE (reset value)
    ADC1->CFGR1 |= ADC_CFGR1_EXTEN_0 // 2^0 -> on rising edge (EXTEN=01) 
        | ADC_CFGR1_EXTSEL_1         // 2^1 -> 010 
        | ADC_CFGR1_EXTSEL_0         // 2^0 -> 001 
                                     //     =  011 -> *** TRG3, source is TIM3_TRGO *** trigger on timer3
        ;                            // Setting ADC_CFGR1_SCANDIR would result in channels scanning from top to bottom
    ADC1->CHSELR = ADC_CHSELR_CHSEL2    // Select ADC_IN2
        | ADC_CHSELR_CHSEL3             // Select ADC_IN3
        | ADC_CHSELR_CHSEL17            // Select ADC_IN17
        ;
    ADC1->SMPR |= ADC_SMPR_SMP_0 | ADC_SMPR_SMP_1 | ADC_SMPR_SMP_2; // 111 is 239.5 ADC clk

    // analog watchdog
    ADC1->CFGR1 |= (3 << 26)        // Select channel 3 for analog watchdog
        | ADC_CFGR1_AWDEN           // Enable Analog Watchdog
        | ADC_CFGR1_AWDSGL          // Enable single channel mode
        ;
    ADC1->TR = (NOCAN_REGS.LEVELS[4]<<16) // Analog watchdog high limit
        | 0x0000;                   // Analog watchdog low limit 
    ADC1->IER = ADC_IER_AWDIE       // Interrupt on Analog Watchdog
        | ADC_IER_OVRIE             // Interrupt on Overflow
        ;

    ADC->CCR |= ADC_CCR_VREFEN;     // Enable internal voltage reference a.k.a. ADC_IN17

    /*** Enable ADC interrupt handler ***/
    NVIC_EnableIRQ(ADC1_COMP_IRQn); /* (7) */
    NVIC_SetPriority(ADC1_COMP_IRQn,1); /* (8) */

    /*** CONFIGURE TIM3 ***/
    /* (1) Enable the peripheral clock of the TIM3 */ 
    /* (2) Configure MMS=010 to output a rising edge at each update event */
    /* (3) Select PCLK/960 i.e. 24MHz/960=25kHz */
    /* (4) Set one update event each second */
    /* (5) Enable TIM15 */
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; /* (1) */ 
    TIM3->CR2 |= TIM_CR2_MMS_1; /* (2) */
    //TIM15->PSC = 959; /* (3) */  // The counter clock frequency is fCK_PSC/(PSC+1)
    //TIM15->ARR = (uint16_t)50000; /* (4) */
    TIM3->PSC = 47999;
    // TIM15->ARR = 1000; // every second
    TIM3->ARR = 20; // 50 times per second
    TIM3->CR1 |= TIM_CR1_CEN; /* (5) */

    /*** CONFIGURE DMA ***/
    /* (1) Enable the peripheral clock on DMA */
    /* (2) Enable DMA transfer on ADC and circular mode */ 
    /* (3) Configure the peripheral data register address */ 
    /* (4) Configure the memory address */
    /* (5) Configure the number of DMA tranfer to be performs on DMA channel 1 */
    /* (6) Configure increment, size, interrupts and circular mode */
    /* (7) Enable DMA Channel 1 */
    RCC->AHBENR |= RCC_AHBENR_DMA1EN; /* (1) */
    ADC1->CFGR1 |= ADC_CFGR1_DMAEN | ADC_CFGR1_DMACFG; /* (2) */
    DMA1_Channel1->CPAR = (uint32_t) (&(ADC1->DR)); /* (3) */
    DMA1_Channel1->CMAR = (uint32_t)(&NOCAN_REGS.LEVELS); /* (4) */
    DMA1_Channel1->CNDTR = NUMBER_OF_ADC_CHANNEL; /* (5) */
    DMA1_Channel1->CCR |= DMA_CCR_MINC | DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 \
                          | DMA_CCR_TEIE | DMA_CCR_CIRC; /* (6) */  
    DMA1_Channel1->CCR |= DMA_CCR_EN; /* (7) */

    gpio_set_pwr_den();
    
    /*** START THE PUMP ***/
    ADC1->CR |= ADC_CR_ADSTART; /* start the ADC conversions */
}

void ADC1_COMP_IRQHandler(void)
{
    if ((ADC1->ISR & ADC_ISR_AWD) != 0)
    {
        gpio_clear_pwr_in(); // redundant with nocan_power_off()
        nocan_power_off();
        NOCAN_STATUS_SET(NOCAN_STATUS_FAULT);
        gpio_set_red_led();
        ADC1->ISR |= ADC_ISR_AWD; 
        NOCAN_REGS.LEVELS[5] = NOCAN_REGS.LEVELS[1];
    }
    if ((ADC1->ISR & ADC_ISR_OVR) != 0)  /* Check OVR has triggered the IT */
    {
        ADC1->ISR |= ADC_ISR_OVR; /* Clears the pending bit */
        ADC1->CR |= ADC_CR_ADSTP; /* Stop the sequence conversion */
        //REGS.STATUS[0] = STATUS0_ADC_FAULT;
        //TODO: add info to status
    }
}

