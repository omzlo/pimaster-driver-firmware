#include <stm32f0xx.h>
#include "gpio.h"
#include "systick.h"
#include "usart.h"
#include "spi_slave.h"
#include "can.h"
#include "nocan.h"
#include "adc.h"

void dump_regs()
{
    int i, j;

    i=0;
    while (i<sizeof(nocan_registers_t))
    {
        usart_printf("%i:", i);
        for (j=0; i+j<sizeof(nocan_registers_t) && j<16; j++)
        {
            usart_printf(" %x", ((uint8_t *)(&NOCAN_REGS))[i+j]);
        }
        i += 16;
        usart_printf("\r\n");
    }
}


int main(void)
{
  char c;
  int i;

  systick_init();

  usart_init(115200);  
  usart_printf("\nStarting:\n");

  usart_printf("* Init GPIO: ");
  gpio_init();
  usart_printf("[OK]\n");

  usart_printf("* 3 second pause: ");
  for (i=0;i<6;i++)
  {
    gpio_set_red_led();
    gpio_clear_yellow_led();
    systick_delay(250);
    gpio_clear_red_led();
    gpio_set_yellow_led();
    systick_delay(250);
    usart_putc('.');
  }
  gpio_clear_yellow_led();
  usart_printf(" [OK]\n");
  
  usart_printf("* Init SPI slave: ");
  spi_slave_init();
  usart_printf("[OK]\n");

  usart_printf("* Init driver registers: ");
  nocan_init(); 
  usart_printf("[OK]\n");

  usart_printf("* Init CAN bus: ");
  can_init(); 
  usart_printf("[OK]\n");

  usart_printf("* Init ADC: ");
  adc_init();
  usart_printf("[OK]\n");

  usart_printf("* Status code: %x\n",NOCAN_REGS.STATUS);
  usart_printf("DONE\n");
  while(1)
  {
      usart_printf("\n[d,h,l,p,P,z]?");
     
      c = usart_getc();
      switch (c) {
          case 'd':
              dump_regs();
              break;
          case 'h': 
              usart_printf("\ncommands: [h] help, [d] dump registers, [l] blink led [p/P] switch power off/on [r/R] disable/enable can-bus resistor [z] reset stm32f0."); 
              break;
          case 'l':
              usart_printf("\nblink red led 2 times.");
              gpio_set_red_led();
              systick_delay(1000);
              gpio_clear_red_led();
              systick_delay(1000);
              gpio_set_red_led();
              systick_delay(1000);
              gpio_set_red_led();
              break;
          case 'p':
              usart_printf("\npower off.");
              nocan_power_off();
              break;
          case 'P':
              usart_printf("\npower on.");
              nocan_power_on();
              break; 
          case 'z':
              usart_printf("\nreseting self (stm32f0)...");
              NVIC_SystemReset();
              break;
      }
   }
}

/*
 * Debug help -- if asserts are enabled, assertion failure
 * from standard peripheral  library ends up here 
 */


#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* Infinite loop */
  /* Use GDB to find out why we're here */
  while (1)
  {
  }
}
#endif

