# This is a prototype Makefile. Modify it according to your needs.
# You should at least check the settings for
# DEVICE ....... The AVR device you compile for
# CLOCK ........ Target AVR clock rate in Hertz
# OBJECTS ...... The object files created from your source files. This list is
#                usually the same as the list of source files with suffix ".o".
# PROGRAMMER ... Options to avrdude which define the hardware you use for
#                uploading to the AVR and the interface where this hardware
#                is connected. We recommend that you leave it undefined and
#                add settings like this to your ~/.avrduderc file:
#                   default_programmer = "stk500v2"
#                   default_serial = "avrdoper"
# FUSES ........ Parameters for avrdude to flash the fuses appropriately.


DEVICE     = atmega16
CLOCK      = 8000000

PROGRAMMER = arduino
PORT = /dev/ttyUSB0
BAUDRATE = 19200


TARGET = main

SRC = $(TARGET).c

OBJECTS = $(SRC:.c=.o)


LFUSE      = 0x64
HFUSE      = 0xD1
FUSES      = -U hfuse:w:$(HFUSE):m -U lfuse:w:$(LFUSE):m

# ATMega8 fuse bits used above (fuse bits for other devices are different!):
# Example for 8 MHz internal oscillator
#
# Fuse low byte:
# 0x24 = 0 0 1 0   0 1 0 0
#        ^ ^ \ /   \--+--/
#        | |  |       +------- CKSEL 3..0 (8M internal RC)
#        | |  +--------------- SUT 1..0 (slowly rising power)
#        | +------------------ BODEN (if 0, brown-out detector is enabled)
#        +-------------------- BODLEVEL (if 0: 4V, if 1: 2.7V)
#
# Fuse high byte:
# 0xd9 = 1 1 0 1   1 0 0 1 <-- BOOTRST (boot reset vector at 0x0000)
#        ^ ^ ^ ^   ^ ^ ^------ BOOTSZ0
#        | | | |   | +-------- BOOTSZ1
#        | | | |   +---------- EESAVE (set to 0 to preserve EEPROM over chip erase)
#        | | | +-------------- CKOPT (clock option, depends on oscillator type)
#        | | +---------------- SPIEN (if set to 1, serial programming is disabled)
#        | +------------------ WDTON (if set to 0, watchdog is always on)
#        +-------------------- RSTDISBL (if set to 0, RESET pin is disabled)
#
# For computing fuse byte values for other devices and options see
# the fuse bit calculator at http://www.engbedded.com/fusecalc/




# Tune the lines below only if you know what you are doing:


COMPILE = avr-gcc -std=gnu11 -Wall -Os -mmcu=$(DEVICE) -DF_CPU=$(CLOCK) -g
AVRDUDE = avrdude -c $(PROGRAMMER) -p $(DEVICE) -P $(PORT) -b $(BAUDRATE)


# symbolic targets:
all:	main.hex

flash:	all
	$(AVRDUDE) -U flash:w:main.hex:i

objdump: $(OBJECTS)
	avr-objdump -d -M intel -S $(OBJECTS) > main_dump.s

assembly:
	avr-gcc -std=gnu11 -Wall -Os -mmcu=$(DEVICE) -DF_CPU=$(CLOCK) -S $(SRC) -o main.s

build_asm:
	avr-gcc -std=gnu11 -Wall -Os -mmcu=atmega16 -DF_CPU=8000000 main.s -o main.elf
	rm -f main.hex
	avr-objcopy -j .text -j .data -O ihex main.elf main.hex
	avr-size --format=avr --mcu=atmega16 main.elf

flash_asm:	build_asm
	$(AVRDUDE) -U flash:w:main.hex:i

fuse:
	$(AVRDUDE) $(FUSES)

clean:
	rm -f main.hex main.elf main_dump.s main_gcc.s $(OBJECTS) main.s


.c.o:
	$(COMPILE) -c $< -o $@


.S.o:
	$(COMPILE) -x assembler-with-cpp -c $< -o $@
# "-x assembler-with-cpp" should not be necessary since this is the default
# file type for the .S (with capital S) extension. However, upper case
# characters are not always preserved on Windows. To ensure WinAVR
# compatibility define the file type manually.


.c.s:
	$(COMPILE) -S $< -o $@



# if you use a bootloader, change the command below appropriately:
load: all
	bootloadHID main.hex


# file targets:
main.elf: $(OBJECTS)
	$(COMPILE) $(OBJECTS) -o main.elf


main.hex: main.elf
	rm -f main.hex
	avr-objcopy -j .text -j .data -O ihex main.elf main.hex
	avr-size --format=avr --mcu=$(DEVICE) main.elf
# If you have an EEPROM section, you must also create a hex file for the
# EEPROM and add it to the "flash" target.
