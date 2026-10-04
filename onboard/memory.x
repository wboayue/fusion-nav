/* The ARK FPV's STM32H743 (#41). The filter and the stack sit in DTCM, which no cache stands in
   front of; a batch of the trace and its results sit in AXI SRAM, out of the stack's way. */
MEMORY
{
  FLASH    : ORIGIN = 0x08000000, LENGTH = 2048K
  RAM      : ORIGIN = 0x20000000, LENGTH = 128K
  AXISRAM  : ORIGIN = 0x24000000, LENGTH = 512K
}

SECTIONS
{
  .axisram (NOLOAD) : ALIGN(4)
  {
    *(.axisram .axisram.*);
    . = ALIGN(4);
  } > AXISRAM
} INSERT AFTER .bss;
