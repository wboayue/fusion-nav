/* The ARK FPV's STM32H743 (#41). The filter and the stack sit in DTCM, which no cache stands in
   front of. AXI SRAM holds a batch of the trace and its results, at fixed addresses the firmware
   names (`src/firmware.rs`, `AXI_SRAM`), rather than a section: a section inserted after `.bss`
   is zeroed with it by cortex-m-rt, which then clears every address from DTCM to AXI SRAM and
   faults before `main`. */
MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 2048K
  RAM   : ORIGIN = 0x20000000, LENGTH = 128K
}
