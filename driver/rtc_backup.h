#ifndef _RTC_H_
#define _RTC_H_

#include <stdint.h>

void rtc_backup_init(void);

uint32_t rtc_backup_read(uint32_t addr);

void rtc_backup_write(uint32_t addr, uint32_t val);

#endif
