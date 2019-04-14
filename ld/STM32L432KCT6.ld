/*
    GNU linker script for STM32L432 (KC)
*/

/* Specify the memory areas */
MEMORY
{
    FLASH (xr)      : ORIGIN = 0x08000000, LENGTH = 256K /* 256Kib Flash */
    RAM (xrw)       : ORIGIN = 0x20000000, LENGTH = 64K  /* 64KiB normal SRAM (16KB of parity RAM mapped at the end) */
}

/* top end of the stack */
_estack = ORIGIN(RAM) + LENGTH(RAM);

/* define output sections */
SECTIONS
{
  /* The program code and other data goes into FLASH */
  .text :
  {
	. = ALIGN(4);
	KEEP(*(.vector_table)) /* isr vector table */
	*(.text)           /* .text sections (code) */
	*(.text*)          /* .text* sections (code) */
	*(.rodata)         /* .rodata sections (constants, strings, etc.) */
	*(.rodata*)        /* .rodata* sections (constants, strings, etc.) */

	. = ALIGN(4);
	_etext = .;        /* define a global symbol at end of code */
  } >FLASH

  .ARM.extab :
  { 
    . = ALIGN(8);
    *(.ARM.extab* .gnu.linkonce.armextab.*)
    . = ALIGN(8);
  } >FLASH
  .ARM :
  {
    . = ALIGN(8);
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
    . = ALIGN(8);
  } >FLASH

  /* This is the initialized data section
  The program executes knowing that the data is in the RAM
  but the loader puts the initial values in the FLASH (inidata).
  It is one task of the startup to copy the initial values from FLASH to RAM. */
  _sidata = .;  /* This is used by the startup in order to initialize the .data secion */
  .data : AT(_sidata)
  {
	. = ALIGN(4);
	_sdata = .;        /* create a global symbol at data start; used by startup code in order to initialise the .data section in RAM */
	*(.data)           /* .data sections */
	*(.data*)          /* .data* sections */

	. = ALIGN(4);
	_edata = .;        /* define a global symbol at data end; used by startup code in order to initialise the .data section in RAM */
  } >RAM

  /* Uninitialized data section */
  .bss :
  {
	. = ALIGN(4);
	_sbss = .;         /* define a global symbol at bss start; used by startup code */
	*(.bss)
	*(.bss*)
	*(COMMON)

	. = ALIGN(4);
	_ebss = .;         /* define a global symbol at bss end; used by startup code */
  } >RAM

  . = ALIGN(4);
  PROVIDE ( end = . );
  PROVIDE ( _end = . );

  .ARM.attributes 0 : { *(.ARM.attributes) }
}
