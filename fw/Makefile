TARGET = modbus-flow-meter
OPENOCD_HOST := localhost
CC := arm-none-eabi-gcc
SZ := arm-none-eabi-size
OBJCOPY := arm-none-eabi-objcopy

SOURCES = main.c printf.c

CFLAGS = -mcpu=cortex-m0plus -mthumb -Wall -fdata-sections -ffunction-sections -g -gdwarf-2 -Og -Wno-builtin-declaration-mismatch
LDSCRIPT = linker.ld
LDFLAGS = -mcpu=cortex-m0plus -mthumb -Tlinker.ld -nostdlib -Wl,-Map=$(TARGET).map,--cref -Wl,--gc-sections
OBJECTS = $(SOURCES:.c=.o)

all: $(TARGET).elf

%.o: %.c Makefile
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(notdir $(<:.c=.lst)) $< -o $@

libnosys_stripped.a: $(shell $(CC) -print-file-name=libnosys.a)
	@cp "$<" "$@"
	$(OBJCOPY) -w -R .gnu.warning.* "$@"

$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

clean:
	@-rm -f $(OBJECTS) $(TARGET).elf $(TARGET).map $(SOURCES:.c=.lst)

openocd:
	sudo openocd \
	  -f interface/cmsis-dap.cfg \
	  -f target/stm32l0.cfg


flash: $(TARGET).elf
	gdb-multiarch \
	  $< \
	  -batch \
	  -ex "target extended-remote $(OPENOCD_HOST):3333" \
	  -ex "monitor reset halt" \
	  -ex "load" \
	  -ex "monitor reset run" \
	  -ex "quit"

debug: $(TARGET).elf
	gdb-multiarch \
	  $< \
	  -ex "target extended-remote $(OPENOCD_HOST):3333"

.PHONY: debug flash all clean
