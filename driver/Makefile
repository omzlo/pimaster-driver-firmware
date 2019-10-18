TEMPLATEROOT = ..

# compilation flags for gdb

CFLAGS  = -O3 -g -Wall
#CFLAGS += -DUSE_FASTER_CAN
ASFLAGS = -g 

# object files

OBJS=  $(STARTUP) main.o
#OBJS+= stm32f0xx_misc.o stm32f0xx_gpio.o stm32f0xx_usart.o stm32f0xx_rcc.o stm32f0xx_can.o 
#OBJS+= system.o usart.o can.o i2c_slave.o systick.o chip_options.o gpio.o nocan.o 
OBJS+= system.o systick.o gpio.o usart.o spi_slave.o can.o nocan.o adc.o rtc_backup.o 

# include common make file

include $(TEMPLATEROOT)/Makefile.common

# Override default for flash 
flash:	$(BIN)
				st-flash write $(BIN) 0x8001000
