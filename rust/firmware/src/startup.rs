use core::arch::global_asm;

global_asm!(
    r#"
    /*
     * Vector table — must be section .vector_table so the linker places it
     * at 0x20000000 (first bytes of the binary loaded into SRAM).
     *
     * Layout after the 8 vector entries (offset 0x20 onward):
     *   code_start   .word _start            — bootloader: copy-from address
     *   code_end     .word end               — bootloader: end of image
     *   code_execute .word _start            — bootloader: jump target
     *                .asciz "..."            — bootloader validation signature
     *
     * The literal pool for the LDR pc,=label instructions is placed by the
     * assembler via .ltorg, after the metadata so it does not land between
     * the vector entries and the bootloader words.
     */
    .section .vector_table, "ax"
    .code 32
    .global _start
_start:
    b _reset_handler              /* 0x00  Reset — direct branch, no literal pool */
    ldr pc, =_undef_handler       /* 0x04  Undefined instruction  */
    ldr pc, =_svc_handler         /* 0x08  Supervisor call        */
    ldr pc, =_prefetch_handler    /* 0x0C  Prefetch abort         */
    ldr pc, =_abort_handler       /* 0x10  Data abort             */
    nop                           /* 0x14  Reserved               */
    ldr pc, =_irq_handler         /* 0x18  IRQ                    */
    ldr pc, =_fiq_handler         /* 0x1C  FIQ                    */

    /* Bootloader metadata — fixed offsets from _start */
    .word _start                  /* 0x20  code_start             */
    .word end                     /* 0x24  code_end               */
    .word _start                  /* 0x28  code_execute           */
    .asciz ".BootLoad_ValidProgramTest."   /* 0x2C  signature      */
    .align 4

    /* Force literal pool here so it follows the metadata, not before it */
    .ltorg

    /*
     * Reset handler — minimal startup for first-milestone bare-metal boot.
     *
     * Steps:
     *   1. Multi-core guard (only core 0 proceeds)
     *   2. Disable I-cache, D-cache, MMU (clean slate — skip full MMU init
     *      for now; we run entirely from internal SRAM)
     *   3. Enable NEON/VFP coprocessor access
     *   4. Set up per-mode stacks
     *   5. Zero BSS
     *   6. Branch to rust_main
     */
    .section .text._reset_handler, "ax"
    .code 32
    .global _reset_handler
_reset_handler:
    /* Mask IRQ, FIQ and async aborts immediately — old firmware may have
     * left CPSR with interrupts enabled. With VBAR=0 any stray exception
     * would branch into SPI flash (the old bootloader at 0x00000000). */
    cpsid aif

    /*
     * 1a. Disable MMU, D-cache, I-cache — leave HIVEC alone for now.
     *
     * Do NOT clear HIVEC here. HIVEC must stay set (ROM vectors) until VBAR
     * is written, so any exception in this window still reaches the ROM
     * handlers rather than branching to address 0.
     */
    mrc p15, 0, r0, c1, c0, 0    /* Read SCTLR                   */
    bic r0, r0, #(1 << 12)       /* I-cache off                  */
    bic r0, r0, #(1 << 2)        /* D-cache off                  */
    bic r0, r0, #(1 << 0)        /* MMU off                      */
    mcr p15, 0, r0, c1, c0, 0
    isb
    dsb

    /* 1b. Set VBAR to our vector table BEFORE clearing HIVEC. */
    ldr r0, =_start
    mcr p15, 0, r0, c12, c0, 0   /* Write VBAR                   */
    isb

    /* 1c. Now safe to clear HIVEC — exceptions will use VBAR = _start. */
    mrc p15, 0, r0, c1, c0, 0
    bic r0, r0, #(1 << 13)       /* Clear HIVEC (use low vectors) */
    mcr p15, 0, r0, c1, c0, 0
    isb

    /* 2. Multi-core guard: put secondary cores to sleep */
    mrc p15, 0, r0, c0, c0, 5    /* Read MPIDR                   */
    ands r0, r0, #0xF             /* Core ID in bits[3:0]         */
    wfine                         /* Secondary core: wait for int  */
    bne _reset_handler            /* Loop if still not core 0     */

    /*
     * 3. Enable CPU writes to data-retention RAM (0x20000000–0x2001FFFF).
     *
     * CPG.SYSCR3 at 0xFCFE0408 — bit[3:0] enable the four retention RAM
     * banks. Reset default is 0x00 (all banks write-protected). Without
     * this, any CPU store to retention RAM causes a Data Abort.
     *
     * Safe to access now: MMU is disabled, so 0xFCFE0408 is a direct
     * physical access to the CPG peripheral (Strongly-Ordered).
     *
     * Dummy read after write is required by the hardware spec.
     */
    ldr r1, =0xFCFE0408           /* CPG.SYSCR3 address           */
    mov r0, #0x0F                 /* Enable all 4 banks           */
    strb r0, [r1]
    ldrb r0, [r1]                 /* Dummy read                   */

    /*
     * NOTE: VFP/NEON enable is skipped here. Enabling it via CPACR +
     * VMSR FPEXC requires either the bootloader to have run (normal path)
     * or full CPG clock/PLL initialization first (JTAG path). This will
     * be added when the CPG init milestone is implemented.
     *
     * The Rust target features (+neon,+vfp3,+d32) are also disabled in
     * .cargo/config.toml so the compiler does not emit VFP instructions.
     */

    /* 3. Set up per-mode stacks (interrupts disabled throughout) */
    msr cpsr_c, #0xD2             /* IRQ mode, I+F masked         */
    ldr sp, =irq_stack_end
    msr cpsr_c, #0xD1             /* FIQ mode                     */
    ldr sp, =fiq_stack_end
    msr cpsr_c, #0xD3             /* SVC mode                     */
    ldr sp, =svc_stack_end
    msr cpsr_c, #0xD7             /* ABT mode                     */
    ldr sp, =abt_stack_end
    msr cpsr_c, #0xDF             /* SYS mode (application)       */
    ldr sp, =program_stack_end

    /* 5. Zero BSS */
    ldr r0, =__bss_start__
    ldr r1, =__bss_end__
    mov r2, #0
.Lbss_loop:
    cmp r0, r1
    strlt r2, [r0], #4
    blt .Lbss_loop

    /* 6. Branch to Rust entry point (never returns) */
    bl rust_main
    b .                           /* Safety: infinite loop        */

    /* Default exception handlers — halt the core */
    .global _undef_handler
    .global _svc_handler
    .global _prefetch_handler
    .global _abort_handler
    .global _irq_handler
    .global _fiq_handler
_undef_handler:     b .
_svc_handler:       b .
_prefetch_handler:  b .
_abort_handler:     b .
_irq_handler:       b .
_fiq_handler:       b .
"#
);
