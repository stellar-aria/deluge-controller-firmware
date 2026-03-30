/* test.ld — linker script for embedded-test integration test binaries.
 *
 * probe-rs loads these directly into on-chip SRAM via SWD and sets PC to
 * _start. Uses upper OCRAM (0x20010000) to avoid the bootloader area living
 * at the base (0x20000000). The same startup.rs as the firmware is used;
 * the startup calls `main` which embedded-test provides.
 */
OUTPUT_FORMAT("elf32-littlearm", "elf32-bigarm", "elf32-littlearm")
OUTPUT_ARCH(arm)
ENTRY(_start)

MEMORY {
    /* Upper on-chip SRAM — 240 KB leaving 64 KB for stacks near top */
    RAM (rwx) : ORIGIN = 0x20010000, LENGTH = 240K
    /* Stack area — top of OCRAM, just below 0x20300000 */
    STACKS (rw) : ORIGIN = 0x202F6000, LENGTH = 0xA000
    /* SDRAM — available if the sdram test initialises it */
    SDRAM (rwx) : ORIGIN = 0x0C000000, LENGTH = 0x04000000
}

/* Smaller stacks for test binaries (don't need production sizes) */
PROGRAM_STACK_SIZE = 0x4000;   /*  16 KB */
IRQ_STACK_SIZE     = 0x1000;   /*   4 KB */
FIQ_STACK_SIZE     = 0x1000;   /*   4 KB */
SVC_STACK_SIZE     = 0x1000;   /*   4 KB */
ABT_STACK_SIZE     = 0x1000;   /*   4 KB */

INTERNAL_RAM_END = 0x20300000;

SECTIONS {
    /* Vector table must be first — startup.rs places it in .vector_table */
    .vector_table : ALIGN(0x20) {
        KEEP(*(.vector_table))
    } > RAM

    .text : ALIGN(4) {
        *(.text .text.*)
        . = ALIGN(4);
    } > RAM

    .rodata : ALIGN(4) {
        *(.rodata .rodata.*)
        . = ALIGN(4);
    } > RAM

    .data : ALIGN(8) {
        *(.data .data.*)
        . = ALIGN(8);
    } > RAM

    .bss (NOLOAD) : ALIGN(4) {
        __bss_start__ = .;
        *(.bss .bss.*)
        *(COMMON)
        . = ALIGN(4);
        __bss_end__ = .;
    } > RAM

    . = ALIGN(0x10);
    end = .;    /* referenced by startup bootloader-metadata word */
    _end = .;

    /* Exception-mode stacks — NOLOAD, placed just below the program stack */
    .irq_stack (INTERNAL_RAM_END - PROGRAM_STACK_SIZE - ABT_STACK_SIZE - SVC_STACK_SIZE - FIQ_STACK_SIZE - IRQ_STACK_SIZE) (NOLOAD) : {
        irq_stack_start = .;
        . += IRQ_STACK_SIZE;
        irq_stack_end = .;
        fiq_stack_start = .;
        . += FIQ_STACK_SIZE;
        fiq_stack_end = .;
        svc_stack_start = .;
        . += SVC_STACK_SIZE;
        svc_stack_end = .;
        abt_stack_start = .;
        . += ABT_STACK_SIZE;
        abt_stack_end = .;
    } > STACKS

    /* Program (SYS mode) stack at the top of SRAM */
    .program_stack (INTERNAL_RAM_END - PROGRAM_STACK_SIZE) (NOLOAD) : {
        program_stack_start = .;
        . += PROGRAM_STACK_SIZE;
        program_stack_end = .;
    } > STACKS

    /DISCARD/ : {
        *(.ARM.exidx*)
        *(.ARM.extab*)
    }
}
