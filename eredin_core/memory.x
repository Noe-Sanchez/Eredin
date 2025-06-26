/* STM32H755ZI memory map for CM7 core */
MEMORY
{
  /* CM7 Flash - starts at 0x08000000, 1MB available */
  FLASH : ORIGIN = 0x08000000, LENGTH = 1024K
  
  /* CM7 DTCM RAM - fastest RAM for CM7 */
  RAM : ORIGIN = 0x20000000, LENGTH = 128K
  
  /* Alternative: Use AXI SRAM if you need more RAM
  RAM : ORIGIN = 0x24000000, LENGTH = 512K */
}
