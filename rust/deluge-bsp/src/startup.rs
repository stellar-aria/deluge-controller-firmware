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
     * Enable NEON/VFP coprocessor access.
     *
     * CPACR[23:22]=CP11 and CPACR[21:20]=CP10 are set to 0b11 (full access
     * in privileged and unprivileged modes).  Then FPEXC.EN (bit 30) is set
     * to power on the VFP/NEON hardware.
     *
     * This does NOT require STB/CPG clock init — VFP is CPU-internal.
     * It must be done here (before Rust) because stb::init() in rust_main
     * may itself emit VFP instructions (the toolchain is free to use NEON
     * for memset etc. once features are re-enabled).
     */
    mrc p15, 0, r0, c1, c0, 2    /* Read CPACR                   */
    orr r0, r0, #(0xF << 20)     /* Enable CP10 and CP11         */
    mcr p15, 0, r0, c1, c0, 2    /* Write CPACR                  */
    isb
    mov r0, #0x40000000           /* FPEXC.EN = 1                 */
    vmsr FPEXC, r0

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
    bl main
    b .                           /* Safety: infinite loop        */

    /* Default exception handlers — halt the core */
    .global _undef_handler
    .global _svc_handler
    .global _prefetch_handler
    .global _abort_handler
    .global _fiq_handler
_undef_handler:     b .
_svc_handler:       b .
_prefetch_handler:  b .
_abort_handler:     b .
_fiq_handler:       b .
"#
);

// ---------------------------------------------------------------------------
// IRQ handler — full GIC dispatch with ARM errata workarounds
// ---------------------------------------------------------------------------
//
// This is a faithful port of `irqfiq_handler.S` from the Renesas BSP,
// adapted to call the Rust `gic_dispatch(icciar)` function.
//
// ARM errata handled:
//   801120  — dummy ICCHPIR read before ICCIAR ensures correct data
//   733075  — spurious ID 0 or >=1022: read-modify-write ICDIPR0, re-read ICCIAR
//
// Register usage on entry (IRQ mode, IRQ/FIQ disabled):
//   LR_irq = interrupted PC + 4
//   SPSR_irq = CPSR of interrupted mode
//
// Stack layout built by this handler (IRQ stack → SYS stack):
//   IRQ stack: [SPSR] [LR_irq - 4]      (srsdb / manual push/pop)
//   SYS stack: r0-r3, r12               (caller-saved AAPCS)
//              alignment pad (0 or 4)
//              r0(=icciar), r1(=pad), r2, r3, r4, lr_sys   (around bl gic_dispatch)

global_asm!(
    r#"
    .equ GICC_IAR_ADDR,   0xE820200C
    .equ GICC_EOIR_ADDR,  0xE8202010
    .equ GICC_HPPIR_ADDR, 0xE8202018
    .equ GICD_IPR0_ADDR,  0xE8201400
    .equ SYS_MODE, 0x1F
    .equ IRQ_MODE, 0x12

    .section .text._irq_handler, "ax"
    .code 32
    .global _irq_handler
    .type _irq_handler, %function
_irq_handler:
    /* Adjust LR: on IRQ entry LR_irq = interrupted PC + 4; subtract 4
     * so MOVS PC,LR returns to the correct instruction. */
    sub     lr, lr, #4

    /* Push return address and SPSR onto the IRQ-mode stack. */
    push    {{lr}}
    mrs     lr, spsr
    push    {{lr}}

    /* Switch to SYS mode (same stack as application; IRQ/FIQ still masked). */
    cps     #SYS_MODE

    /* Save caller-saved AAPCS registers. */
    push    {{r0-r3, r12}}

    /* ---- ARM Errata 801120: dummy HPPIR read before ICCIAR ---- */
    ldr     r2, =GICC_HPPIR_ADDR
    ldr     r2, [r2]

    /* Read Interrupt Acknowledge Register -> r3 = raw icciar */
    ldr     r2, =GICC_IAR_ADDR
    ldr     r3, [r2]

    /* Extract interrupt ID (bits [9:0]) for errata-733075 check. */
    ubfx    r0, r3, #0, #10

    /* ---- ARM Errata 733075: SGI ID 0 or spurious IDs >= 1022 ---- */
    cmp     r0, #0
    beq     .Lirq_errata_733075
    ldr     r1, =1022
    cmp     r0, r1
    bge     .Lirq_errata_733075
    b       .Lirq_post_errata

.Lirq_errata_733075:
    /* Read-modify-write ICDIPR0 to resolve the errata, then re-read ICCIAR. */
    ldr     r2, =GICD_IPR0_ADDR
    ldr     r0, [r2]
    str     r0, [r2]
    dsb
    ldr     r2, =GICC_HPPIR_ADDR
    ldr     r2, [r2]
    ldr     r2, =GICC_IAR_ADDR
    ldr     r3, [r2]

.Lirq_post_errata:
    /* r0 = full icciar (used for both gic_dispatch arg and ICCEOIR write). */
    mov     r0, r3

    /* Align stack to 8 bytes (AAPCS requirement for BL). */
    mov     r1, sp
    and     r1, r1, #4
    sub     sp, sp, r1

    /* Push: r0(icciar), r1(pad amount), r2, r3, r4(padding reg), lr_sys. */
    push    {{r0-r4, lr}}

    /* Call Rust dispatch: gic_dispatch(icciar) -- r0 already set. */
    bl      gic_dispatch

    /* Restore registers; r0 restored to icciar for the EOI write below. */
    pop     {{r0-r4, lr}}

    /* Undo stack alignment. */
    add     sp, sp, r1

    /* Disable IRQ, memory + instruction barriers, then write EOI. */
    cpsid   i
    dsb
    isb
    ldr     r2, =GICC_EOIR_ADDR
    str     r0, [r2]

    /* Restore caller-saved registers. */
    pop     {{r0-r3, r12}}

    /* Return to interrupted mode: switch to IRQ mode, restore SPSR + LR. */
    cps     #IRQ_MODE
    pop     {{lr}}
    msr     spsr_cxsf, lr
    pop     {{lr}}
    movs    pc, lr          /* return from IRQ exception */

    .size _irq_handler, . - _irq_handler
"#
);
