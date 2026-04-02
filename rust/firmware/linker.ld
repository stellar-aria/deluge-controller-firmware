OUTPUT_FORMAT("elf32-littlearm", "elf32-bigarm", "elf32-littlearm")
OUTPUT_ARCH(arm)
ENTRY(_start)

MEMORY {
    /* 3 MB on-chip SRAM */
    RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 0x00300000

    /* 64 KB reserved at the top of SRAM for the RTT ring buffer.
       Placed just below the stacks (stacks occupy the top 160 KB).
       Cached alias: 0x202B0000–0x202BFFFF
       The uncached mirror is the same physical page + 0x40000000. */
    RTT_RAM  (rw)  : ORIGIN = 0x202B0000, LENGTH = 0x00010000  /* 64 KB cached   */
    NCACHE_RAM (rw): ORIGIN = 0x602B0000, LENGTH = 0x00010000  /* 64 KB uncached  */

    /* 64 MB external SDRAM (CS3) — used in later milestones */
    SDRAM (rwx) : ORIGIN = 0x0C000000, LENGTH = 0x04000000
}

/* Stack sizes */
PROGRAM_STACK_SIZE = 0x8000;   /* 32 KB - application / SYS mode */
IRQ_STACK_SIZE     = 0x2000;   /*  8 KB */
FIQ_STACK_SIZE     = 0x2000;   /*  8 KB */
SVC_STACK_SIZE     = 0x2000;   /*  8 KB */
ABT_STACK_SIZE     = 0x2000;   /*  8 KB */

INTERNAL_RAM_END = 0x20300000;

SECTIONS {
    /* _SEGGER_RTT control block in the uncached SRAM mirror so the debug
       probe can always see RTT writes, even after D-cache is enabled.
       RTT_RAM (cached) and NCACHE_RAM (uncached) are the same physical page;
       reserving both prevents BSS or stacks from being placed there. */
    .rtt_cached_reserve (NOLOAD) : ALIGN(4) { . += 0x10000; } > RTT_RAM
    .rtt_buffer (NOLOAD) : ALIGN(4) {
        KEEP(*(.rtt_buffer .rtt_buffer.*))
        . = ALIGN(4);
    } > NCACHE_RAM

    /*
     * Vector table + bootloader metadata.
     * Must be the very first bytes of the binary at 0x20000000.
     * The bootloader reads fixed offsets past the 8 vector entries:
     *   +0x20: code_start  (word: address of _start)
     *   +0x24: code_end    (word: address of end)
     *   +0x28: code_execute (word: execution entry = _start)
     *   +0x2C: ".BootLoad_ValidProgramTest." signature
     */
    .vector_table : ALIGN(0x20) {
        KEEP(*(.vector_table))
    } > RAM

    /* Main code */
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

    /*
     * Round up to next 64KB boundary so the bootloader knows how much to
     * copy from SPI flash into SRAM (matches original linker script).
     */
    . = ALIGN(0x10000);
    end = .;
    _end = .;

    /* SRAM heap: free space between the image and the RTT/stack reservation */
    __sram_heap_start = end;
    __sram_heap_end   = ORIGIN(RTT_RAM);  /* 0x202B0000 */

    /*
     * Exception-mode stacks — NOLOAD, placed just below the program stack.
     * Stacks grow downward; _end symbols are the initial SP values.
     */
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
    } > RAM

    /* Program (SYS mode) stack at the very top of SRAM */
    .program_stack (INTERNAL_RAM_END - PROGRAM_STACK_SIZE) (NOLOAD) : {
        program_stack_start = .;
        . += PROGRAM_STACK_SIZE;
        program_stack_end = .;
    } > RAM

    /DISCARD/ : {
        *(.ARM.exidx*)
        *(.ARM.extab*)
    }
}
