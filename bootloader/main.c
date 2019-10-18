#include <stm32f0xx.h>
#include "gpio.h"
#include "rtc_backup.h"
#include "flash.h"
#include "systick.h"
#include "spi_slave.h"

typedef struct {
  // @0x00
  uint32_t MARKER;  // string 'BOOT' encoded.
  uint32_t MCUID;
  uint16_t PAGE_SIZE;   // Page size in bytes
  uint16_t FLASH_SIZE;  // Flash size in 1K
  uint32_t PROG_START;  // Prog start adress
  // @0x10
  uint8_t VERSION; 
  uint8_t RESERVED;
  int8_t  ERR;       // errors are reported here
  uint8_t PROG;
  // @0x14
  uint32_t ADDR;
  // @0x18
  uint16_t DATA[32];  // 64 bytes
} regs_t; 

static regs_t REGS;

enum {
  PROG_NONE       = 0,
  PROG_ERASE_PAGE = 1,
  PROG_READ       = 2,
  PROG_WRITE      = 3,
  PROG_EXIT       = 4
};

static void regs_init(void)
{
  REGS.MARKER = 0x544f4f42;     // 'BOOT' in little endian
  REGS.MCUID = DBGMCU->IDCODE;
  REGS.PAGE_SIZE = FLASH_PAGE_SIZE;
  REGS.FLASH_SIZE = *((uint16_t*)0x1FFFF7CC);
  REGS.PROG_START = FLASH_APP_START;
  REGS.VERSION = 1;
  REGS.RESERVED = 0;
  REGS.ERR = 0;
  REGS.PROG = PROG_NONE;
  REGS.ADDR = REGS.PROG_START;
}

static int8_t validate_address(void)
{
  /* TODO: actually we should probably substract 64 to the end address. */
  if ((REGS.ADDR<FLASH_APP_START) || (REGS.ADDR>FLASH_APP_END)) {
    return -4;
  }
  return 0;
}


void update_leds()
{
  static uint32_t last = 0;
  uint32_t cur = (systick_now()/500)&1;

  if (last!=cur) {
    if (cur) 
    {
      gpio_set_red_led();
      gpio_set_yellow_led();
    } 
    else 
    {
      gpio_clear_red_led();
      gpio_clear_yellow_led();
    } 

    last = cur;
  }
}

static int test_boot_pins(void)
{
    for (int i=0; i<3; i++)
    {
        gpio_clear_swdio();
        systick_delay(10);
        if (gpio_read_swclk()!=0)
            return 0;
        gpio_set_swdio();
        systick_delay(10);
        if (gpio_read_swclk()==0)
            return 0;
    }
    return 1;
}

int main(void)
{
  int bootloader_mode = 0;
  unsigned last_spi = 0;
  unsigned next_spi;

  systick_init();
  gpio_init();
  rtc_backup_init();

  if (test_boot_pins())
  {
     bootloader_mode = 1;
  } 
  else if (rtc_backup_read(0)==0xdeadbeef)
  {
     bootloader_mode = 1;
  }
  gpio_deinit(); /* this does not affect the LEDs */
     
     
  if (bootloader_mode)
  {
    rtc_backup_write(0,0);
    flash_open();
    regs_init();
    spi_slave_init(&REGS,sizeof(REGS));

    for (;;)
    {
      next_spi = spi_write_count;
      if (next_spi != last_spi)
      {
        last_spi = next_spi;

        if (REGS.PROG != 0) {
          switch (REGS.PROG) {
            case PROG_ERASE_PAGE:
              if ((REGS.ERR = validate_address()) == 0)
              {
                REGS.ERR = flash_erase_page(REGS.ADDR);
              }
              break;
            case PROG_READ:
              if ((REGS.ERR = validate_address()) == 0)
              {
                REGS.ERR = flash_read_block(REGS.ADDR, REGS.DATA, 32);
              }
              REGS.ADDR += 64;
              break;
            case PROG_WRITE:
              if ((REGS.ERR = validate_address()) == 0)
              {
                REGS.ERR = flash_write_block(REGS.ADDR, REGS.DATA, 32);
              }
              REGS.ADDR += 64;
              break;
            case PROG_EXIT:
              NVIC_SystemReset();
              break;
            default:
              REGS.ERR = -100;
          }
          REGS.PROG = 0;
        }
      } else {
        update_leds();
      }
    }
  }
  flash_start_main_application();   
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

