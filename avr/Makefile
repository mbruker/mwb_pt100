 
PRG            = pt100_rs485
OBJ            = main.o uart.o
HEADERS        = uart.h
MCU_TARGET     = atmega32
OPTIMIZE       = -Os

DEFS           = -std=c99
LIBS           = -lm

CC             = avr-gcc

# Override is only needed by avr-lib build system.

override CFLAGS        = -g -Wall $(OPTIMIZE) -mmcu=$(MCU_TARGET) $(DEFS)
override LDFLAGS       = -Wl,-Map,$(PRG).map

OBJCOPY        = avr-objcopy
OBJDUMP        = avr-objdump

all: $(PRG).elf lst hex eep

$(PRG).elf: $(OBJ)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)

# dependency:
uart.o: uart.h uart.c
main.o: $(HEADERS) main.c

clean:
	rm -rf $(OBJ) $(PRG).elf $(PRG).lst $(PRG).bin $(PRG).hex $(PRG).map $(PRG).srec $(PRG).eep

lst:  $(PRG).lst

%.lst: %.elf
	$(OBJDUMP) -h -S $< > $@

%.eep: %.elf
	-$(OBJCOPY) -j .eeprom --set-section-flags=.eeprom="alloc,load" --change-section-lma .eeprom=0 -O ihex $< $@

eep: $(PRG).eep

hex:  $(PRG).hex

%.hex: %.elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@

flash: hex
	avrdude -P usb -c avrispmkII -p m32 -U flash:w:$(PRG).hex

fuse:
	avrdude -P usb -c avrispmkII -p m32 -U lfuse:w:0b10111111:m -U hfuse:w:0b11000001:m

eeprom:
	avrdude -P usb -c avrispmkII -p m32 -U eeprom:w:$(PRG).eep
