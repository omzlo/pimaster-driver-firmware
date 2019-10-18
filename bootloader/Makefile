TEMPLATEROOT = ..

# compilation flags for gdb

CFLAGS+= -Os -Wall -nostdlib -fdata-sections -ffunction-sections 
LDFLAGS+= -Wl,--gc-sections --specs=nano.specs --specs=nosys.specs
ASFLAGS=

# object files

OBJS=  $(STARTUP) main.o
OBJS+= system.o gpio.o rtc_backup.o flash.o systick.o spi_slave.o 

# include common make file

include $(TEMPLATEROOT)/Makefile.common


