ENTRY(reset_handler)

/* Memory regions */
MEMORY
{
  FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 32K
  RAM   (xrw) : ORIGIN = 0x20000000, LENGTH = 8K
}

/* Stack pointer */
_estack = ORIGIN(RAM) + LENGTH(RAM);

/* Sections */
SECTIONS
{
  .isr_vector :
  {
    KEEP(*(.isr_vector))
  } > FLASH

  .text :
  {
    *(.text)
    *(.text*)
    *(.rodata)
    *(.rodata*)
    *(.init)
    *(.fini)
    _etext = .;
  } > FLASH

  .data : AT (LOADADDR(.text) + SIZEOF(.text))
  {
    _sdata = .;
    *(.data)
    *(.data*)
    _edata = .;
  } > RAM

  _sidata = LOADADDR(.data);

  .bss :
  {
    _sbss = .;
    *(.bss)
    *(.bss*)
    *(COMMON)
    _ebss = .;
  } > RAM
}
