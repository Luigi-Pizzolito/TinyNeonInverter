# === CONFIGURATION ===

MCU = attiny10
F_CPU = 8000000UL

TARGET = main
SRC = $(TARGET).c

CC = avr-gcc
OBJCOPY = avr-objcopy
CFLAGS = -Wall -Os -DF_CPU=$(F_CPU) -mmcu=$(MCU) -fno-tree-loop-ivcanon

AVRDUDE = avrdude
PROGRAMMER = usbasp
AVRDUDE_FLAGS = -c $(PROGRAMMER) -p t10 -P usb -B1 -v

# === BUILD RULES ===

all: $(TARGET).hex

%.elf: %.c
	$(CC) $(CFLAGS) -o $@ $<

%.hex: %.elf
	$(OBJCOPY) -O ihex -R .eeprom $< $@

flash: $(TARGET).hex
	$(AVRDUDE) $(AVRDUDE_FLAGS) -U flash:w:$<

clean:
	rm -f *.elf *.hex


