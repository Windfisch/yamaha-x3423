/* Linker script for the STM32F103C8T6 */
MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 62K /* last page is reserved for parameters */
  RAM : ORIGIN = 0x20000000, LENGTH = 20K
}
