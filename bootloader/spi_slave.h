#ifndef _SPI_SLAVE_H_
#define _SPI_SLAVE_H_
#include <stdint.h>

int spi_slave_init(void *buf, unsigned buf_len);

extern unsigned spi_write_count;

#endif
