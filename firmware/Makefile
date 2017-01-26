# customize the following paths for your computer
ROOT_DIR:=$(shell dirname $(realpath $(lastword $(MAKEFILE_LIST))))
INCLUDE1 = ./usb/include
INCLUDE2 = ./usb/src
INCLUDE3 = ./spi
INCLUDE4 = ./heflash
CC = xc8

CHIP = 16F1459

CFLAGS = --chip=$(CHIP) -Q -G  --double=24 --float=24
CFLAGS += --opt=default,+asm,-asmfile,+speed,-space,-debug --addrqual=ignore
CFLAGS += --mode=pro -N64 -I. -I$(INCLUDE1) -I$(INCLUDE2) -I$(INCLUDE3) -I$(INCLUDE4) --warn=0 --asmlist
CFLAGS += --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 
CFLAGS += --runtime=default,+clear,+init,-keep,-no_startup,+osccal,-resetbits,-download,-stackcall,+clib
CFLAGS += --ROM=default,-1f80-1fff

all: Makefile
	$(CC) $(CFLAGS) -o./firmware.hex main.c spi/spi_software.c usb_descriptors.c usb/src/usb.c heflash/Flash.c heflash/HEFlash.c
	rm -f *.p1 *.d *.pre *.sym *.cmf *.cof *.hxl *.lst *.obj *.rlf *.sdb
	rm -f funclist

clean:
	rm -f firmware.hex

flash: Makefile
	$(CC) $(CFLAGS) -o./firmware.hex main.c spi/spi_software.c usb_descriptors.c usb/src/usb.c heflash/Flash.c heflash/HEFlash.c
	rm -f *.p1 *.d *.pre *.sym *.cmf *.cof *.hxl *.lst *.obj *.rlf *.sdb
	rm -f funclist
	pk2cmd -P -M -F$(ROOT_DIR)/firmware.hex
