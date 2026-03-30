//! On-device SDRAM integration test.
//!
//! Writes 16 distinct 32-bit values to SDRAM base (0x0C00_0000) and reads
//! them back. A mismatch indicates an address-line or data-bus fault.
//!
//! Requires full init chain: STB → MMU → L1 cache → L2 cache → SDRAM.
//!
//! Run on-device: `cargo test-fw -p deluge-bsp --test sdram`
#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]

#[cfg(not(target_os = "none"))]
fn main() {}

#[cfg(target_os = "none")]
use deluge_test as _;

#[cfg(target_os = "none")]
#[deluge_test::tests]
mod tests {
    use rza1::{cache, mmu, sdram, stb};

    #[init]
    fn init() {
        unsafe {
            stb::init();
            mmu::init_and_enable();
            cache::l1_enable();
            cache::l2_init();
            sdram::init();
        }
    }

    /// Write a 16-word marching pattern to SDRAM and verify it reads back.
    ///
    /// After writing, each dirty L1 D-cache line covering the test region is
    /// cleaned and invalidated to PoC before reading back, so the reads reach
    /// SDRAM rather than cache. The Cortex-A9 cache line is 32 bytes; the 64
    /// bytes written here span exactly two cache lines.
    #[test]
    fn read_write_pattern() {
        const BASE: *mut u32 = 0x0C00_0000 as *mut u32;
        const WORDS: usize = 16;
        const CACHE_LINE: usize = 32;
        const TOTAL_BYTES: usize = WORDS * core::mem::size_of::<u32>();

        unsafe {
            for i in 0..WORDS {
                BASE.add(i).write_volatile(0xDEAD_0000 | i as u32);
            }

            // Clean + invalidate every cache line in the write region to PoC
            // so that reads below bypass L1/L2 and actually reach SDRAM.
            let base_addr = BASE as usize;
            let mut offset = 0;
            while offset < TOTAL_BYTES {
                cortex_ar::cache::clean_and_invalidate_data_cache_line_to_poc(
                    (base_addr + offset) as u32,
                );
                offset += CACHE_LINE;
            }
            core::arch::asm!("dsb", options(nomem, nostack));

            for i in 0..WORDS {
                let got = BASE.add(i).read_volatile();
                let want = 0xDEAD_0000 | i as u32;
                assert_eq!(got, want, "SDRAM[{i}]: got {got:#010x} want {want:#010x}");
            }
        }
    }
}
