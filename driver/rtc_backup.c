#include <stm32f0xx.h>
#include "rtc_backup.h"

void rtc_backup_init(void)
{
  RCC->APB1ENR |= RCC_APB1ENR_PWREN;

  // Enable access to backup domain
  // In reset state, the RTC and backup registers are protected against parasitic write access. This
  // bit must be set to enable write access to these registers.
  PWR->CR |= PWR_CR_DBP;
}

uint32_t rtc_backup_read(uint32_t addr)
{
  uint32_t *regs = (uint32_t *)(&(RTC->BKP0R));
  return regs[addr];
}

void rtc_backup_write(uint32_t addr, uint32_t val)
{
  uint32_t *regs = (uint32_t *)(&(RTC->BKP0R));
  regs[addr] = val;
}


