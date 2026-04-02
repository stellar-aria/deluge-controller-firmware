//! `deluge-test` — on-device test harness proc-macro for the Deluge / RZA1L.
//!
//! Provides `#[deluge_test::tests]`, `#[init]`, `#[test]`, and `#[timeout(N)]`
//! attributes with an API surface compatible with `embedded-test`.
//!
//! Unlike `embedded-test`, communication with the host runner uses **RTT only**
//! (no semihosting / SVC #0x123456), which is essential for the Cortex-A9 on
//! the RZA1L where the probe-rs semihosting intercept does not fire.
//!
//! ## Protocol (RTT channel 0 in both directions)
//!
//! ```text
//! Device → Host:
//!   TESTS:<n>\n           — total test count
//!   <name>:<timeout_s>\n  — one line per test
//!   READY\n               — device is waiting for a RUN command
//!   PASS:<name>\n         — test passed
//!   FAIL:<name>:<file>:<line>\n  — test failed (panic info)
//!   DONE\n                — end of result; device self-jumps to _reset_handler
//!
//! Host → Device:
//!   RUN:<name>\n          — run exactly one test
//! ```
//!
//! After sending `DONE` the device executes `b _reset_handler` — a software
//! branch back to the assembly startup routine in SRAM, which re-zeros BSS,
//! re-runs `main`, re-initialises RTT, and waits for the next `READY`/`RUN`
//! cycle.  No hardware reset is issued, so the NOR FLASH at CS0 is never
//! accessed.
//!
//! ## Async support
//!
//! Pass `executor = embassy_executor::Executor::new()` to the `tests` attribute:
//!
//! ```ignore
//! #[deluge_test::tests(executor = embassy_executor::Executor::new())]
//! mod tests { ... }
//! ```
//!
//! The generated code creates a single Embassy `Executor` and spawns the test
//! session as an `#[embassy_executor::task]`.  Both sync and async `#[test]`
//! functions are dispatched through the same task, with async tests awaited
//! and sync ones called directly.

use proc_macro::TokenStream;
use proc_macro2::TokenStream as TokenStream2;
use quote::quote;
use syn::{
    parse::{Parse, ParseStream},
    parse_macro_input, Expr, Ident, Item, ItemMod, LitInt, Token,
};

// ---------------------------------------------------------------------------
// Attribute argument parser
// ---------------------------------------------------------------------------

/// Optional `executor = <expr>` argument to `#[tests]`.
struct TestsAttr {
    executor: Option<Expr>,
}

impl Parse for TestsAttr {
    fn parse(input: ParseStream) -> syn::Result<Self> {
        if input.is_empty() {
            return Ok(Self { executor: None });
        }
        let key: Ident = input.parse()?;
        if key != "executor" {
            return Err(syn::Error::new(
                key.span(),
                "expected `executor = <expr>` or empty",
            ));
        }
        let _eq: Token![=] = input.parse()?;
        let val: Expr = input.parse()?;
        Ok(Self {
            executor: Some(val),
        })
    }
}

// ---------------------------------------------------------------------------
// Test-case metadata collected during the module parse
// ---------------------------------------------------------------------------

struct TestCase {
    name: Ident,
    timeout_secs: u64,
    is_async: bool,
}

// ---------------------------------------------------------------------------
// #[deluge_test::tests] — the main attribute macro
// ---------------------------------------------------------------------------

#[proc_macro_attribute]
pub fn tests(attr: TokenStream, item: TokenStream) -> TokenStream {
    let attr = parse_macro_input!(attr as TestsAttr);
    let module = parse_macro_input!(item as ItemMod);

    let (_, content) = match &module.content {
        Some(c) => c,
        None => {
            return syn::Error::new_spanned(&module, "module must have an inline body")
                .to_compile_error()
                .into();
        }
    };

    let mod_ident = &module.ident;
    let mod_vis = &module.vis;
    let mod_attrs = &module.attrs;

    let mut init_ident: Option<Ident> = None;
    let mut test_cases: Vec<TestCase> = Vec::new();
    let mut cleaned_items: Vec<TokenStream2> = Vec::new();

    for item in content {
        match item {
            Item::Fn(fn_item) => {
                let has_init = fn_item.attrs.iter().any(|a| a.path().is_ident("init"));
                let has_test = fn_item.attrs.iter().any(|a| a.path().is_ident("test"));

                if has_init {
                    init_ident = Some(fn_item.sig.ident.clone());
                    let mut f = fn_item.clone();
                    f.attrs.retain(|a| !a.path().is_ident("init"));
                    f.vis = syn::Visibility::Public(syn::token::Pub::default());
                    cleaned_items.push(quote! { #f });
                } else if has_test {
                    // Collect `#[timeout(N)]` if present.
                    let timeout_secs = fn_item
                        .attrs
                        .iter()
                        .find(|a| a.path().is_ident("timeout"))
                        .and_then(|a| a.parse_args::<LitInt>().ok())
                        .and_then(|l| l.base10_parse::<u64>().ok())
                        .unwrap_or(60);
                    let is_async = fn_item.sig.asyncness.is_some();
                    let name = fn_item.sig.ident.clone();
                    test_cases.push(TestCase {
                        name,
                        timeout_secs,
                        is_async,
                    });
                    // Strip `#[test]` and `#[timeout]`; keep everything else.
                    let mut f = fn_item.clone();
                    f.attrs
                        .retain(|a| !a.path().is_ident("test") && !a.path().is_ident("timeout"));
                    f.vis = syn::Visibility::Public(syn::token::Pub::default());
                    cleaned_items.push(quote! { #f });
                } else {
                    cleaned_items.push(quote! { #fn_item });
                }
            }
            other => cleaned_items.push(quote! { #other }),
        }
    }

    // -----------------------------------------------------------------------
    // Derive announcement strings and plain name strings
    // -----------------------------------------------------------------------

    let n_tests = test_cases.len();

    // Per-test announce line: "name:timeout_secs"
    let announce_lines: Vec<String> = test_cases
        .iter()
        .map(|t| format!("{}:{}", t.name, t.timeout_secs))
        .collect();

    // Plain names for the panic-handler lookup array.
    let name_strs: Vec<String> = test_cases.iter().map(|t| t.name.to_string()).collect();

    // -----------------------------------------------------------------------
    // init call (always unsafe-wrapped since existing init fns use unsafe blocks)
    // -----------------------------------------------------------------------

    let init_call = match &init_ident {
        Some(name) => quote! { unsafe { #mod_ident::#name(); } },
        None => quote! {},
    };

    // -----------------------------------------------------------------------
    // Shared RTT-init + announce block
    // -----------------------------------------------------------------------

    let rtt_init_announce = quote! {
        // Reset the current-test sentinel so panics in init() are labelled "init".
        CURRENT_TEST_IDX.store(usize::MAX, core::sync::atomic::Ordering::Relaxed);

        let _channels = rtt_target::rtt_init! {
            up: {
                0: {
                    size: 1024,
                    name: "Terminal"
                }
            }
            down: {
                0: {
                    size: 256,
                    name: "Terminal"
                }
            }
        };
        rtt_target::set_print_channel(_channels.up.0);
        let mut _down = _channels.down.0;

        #init_call

        rtt_target::rprintln!("TESTS:{}", #n_tests);
        for _line in [#(#announce_lines),*].iter() {
            rtt_target::rprintln!("{}", _line);
        }
        rtt_target::rprintln!("READY");
    };

    // -----------------------------------------------------------------------
    // Dispatch match arms (both sync and async variants)
    // -----------------------------------------------------------------------

    let sync_arms: Vec<TokenStream2> = test_cases
        .iter()
        .enumerate()
        .map(|(i, tc)| {
            let name_str = tc.name.to_string();
            let fn_ident = &tc.name;
            quote! {
                #name_str => {
                    CURRENT_TEST_IDX.store(#i, core::sync::atomic::Ordering::Relaxed);
                    #mod_ident::#fn_ident();
                    rtt_target::rprintln!("PASS:{}", #name_str);
                }
            }
        })
        .collect();

    let async_arms: Vec<TokenStream2> = test_cases
        .iter()
        .enumerate()
        .map(|(i, tc)| {
            let name_str = tc.name.to_string();
            let fn_ident = &tc.name;
            if tc.is_async {
                quote! {
                    #name_str => {
                        CURRENT_TEST_IDX.store(#i, core::sync::atomic::Ordering::Relaxed);
                        #mod_ident::#fn_ident().await;
                        rtt_target::rprintln!("PASS:{}", #name_str);
                    }
                }
            } else {
                quote! {
                    #name_str => {
                        CURRENT_TEST_IDX.store(#i, core::sync::atomic::Ordering::Relaxed);
                        #mod_ident::#fn_ident();
                        rtt_target::rprintln!("PASS:{}", #name_str);
                    }
                }
            }
        })
        .collect();

    // -----------------------------------------------------------------------
    // Read-and-dispatch block: synchronous (busy-poll RTT, then call test)
    // -----------------------------------------------------------------------

    let read_dispatch_sync = quote! {
        let mut _buf = [0u8; 64];
        loop {
            let _n = _deluge_test_rtt_readline(&mut _down, &mut _buf);
            let _cmd = core::str::from_utf8(&_buf[.._n]).unwrap_or("");
            if let Some(_tname) = _cmd.strip_prefix("RUN:") {
                let _tname = _tname.trim_end_matches('\r');
                match _tname {
                    #(#sync_arms)*
                    _ => {
                        rtt_target::rprintln!(
                            "FAIL:?:deluge_test:0:unknown test `{}`",
                            _tname
                        );
                    }
                }
                rtt_target::rprintln!("DONE");
                // Drain the RTT up-channel before self-jumping: if we branch
                // to _reset_handler while the buffer still holds unread bytes,
                // rtt_init! will overwrite WrOff/RdOff and those bytes are
                // lost before JLink can read them.
                //
                // SEGGER RTT control block layout (32-bit ARM, rtt-target 0.6):
                //   _SEGGER_RTT + 36 = up[0].WrOff  (written by device)
                //   _SEGGER_RTT + 40 = up[0].RdOff  (advanced by JLink)
                unsafe {
                    extern "C" { static _SEGGER_RTT: u8; }
                    let base = &_SEGGER_RTT as *const u8 as usize;
                    let wr = (base + 36) as *const u32;
                    let rd = (base + 40) as *const u32;
                    while core::ptr::read_volatile(wr) != core::ptr::read_volatile(rd) {
                        core::arch::asm!("nop");
                    }
                    core::arch::asm!("b _reset_handler", options(noreturn));
                }
            }
        }
    };

    // -----------------------------------------------------------------------
    // Read-and-dispatch block: async (used inside an embassy task)
    // -----------------------------------------------------------------------

    let read_dispatch_async = quote! {
        let mut _buf = [0u8; 64];
        let _n = _deluge_test_rtt_readline(&mut _down, &mut _buf);
        let _cmd = core::str::from_utf8(&_buf[.._n]).unwrap_or("");
        if let Some(_tname) = _cmd.strip_prefix("RUN:") {
            let _tname = _tname.trim_end_matches('\r');
            match _tname {
                #(#async_arms)*
                _ => {
                    rtt_target::rprintln!(
                        "FAIL:?:deluge_test:0:unknown test `{}`",
                        _tname
                    );
                }
            }
            rtt_target::rprintln!("DONE");
        }
        // Drain the RTT up-channel before self-jumping (see sync path comment).
        unsafe {
            extern "C" { static _SEGGER_RTT: u8; }
            let base = &_SEGGER_RTT as *const u8 as usize;
            let wr = (base + 36) as *const u32;
            let rd = (base + 40) as *const u32;
            while core::ptr::read_volatile(wr) != core::ptr::read_volatile(rd) {
                core::arch::asm!("nop");
            }
            core::arch::asm!("b _reset_handler", options(noreturn));
        }
    };

    // -----------------------------------------------------------------------
    // Generated runner code (sync vs async path)
    // -----------------------------------------------------------------------

    let runner_code = if let Some(executor_expr) = &attr.executor {
        // Async path: wrap the session in an Embassy task.
        // The `#[embassy_executor::task]` attribute macro is expanded by the
        // compiler when it processes the generated code.
        quote! {
            #[embassy_executor::task]
            async fn _deluge_test_runner_task() {
                #rtt_init_announce
                #read_dispatch_async
            }

            #[no_mangle]
            pub unsafe extern "C" fn main() -> ! {
                // MaybeUninit avoids requiring Executor::new() to be const.
                static mut _EXECUTOR_STORAGE:
                    core::mem::MaybeUninit<embassy_executor::Executor> =
                    core::mem::MaybeUninit::uninit();
                let _executor: &'static mut embassy_executor::Executor =
                    unsafe { _EXECUTOR_STORAGE.write(#executor_expr) };
                _executor.run(|_spawner| {
                    _spawner.spawn(_deluge_test_runner_task().unwrap());
                });
            }
        }
    } else {
        // Sync path: everything runs directly in main().
        quote! {
            #[no_mangle]
            pub unsafe extern "C" fn main() -> ! {
                #rtt_init_announce
                #read_dispatch_sync
            }
        }
    };

    // -----------------------------------------------------------------------
    // Panic handler
    // -----------------------------------------------------------------------

    // Build a const array expression for name lookup; guard against n_tests==0.
    let name_lookup = if n_tests > 0 {
        quote! {
            const _NAMES: &[&str] = &[#(#name_strs),*];
            if _idx < _NAMES.len() { _NAMES[_idx] } else { "?" }
        }
    } else {
        quote! { "?" }
    };

    // -----------------------------------------------------------------------
    // Final output
    // -----------------------------------------------------------------------

    let output = quote! {
        // The test module with cleaned attributes (no #[init] / #[test] / #[timeout]).
        #(#mod_attrs)*
        #mod_vis mod #mod_ident {
            #(#cleaned_items)*
        }

        // Index of the currently-running test; usize::MAX = init / no test.
        // Stored in .data (non-zero init) — not clobbered by BSS zero on self-jump.
        // On the first flash the probe loads the correct initial value from the ELF.
        // After each self-jump .data is not re-zeroed, so we explicitly reset this
        // at the top of every main() invocation.
        static CURRENT_TEST_IDX: core::sync::atomic::AtomicUsize =
            core::sync::atomic::AtomicUsize::new(usize::MAX);

        /// Busy-polls the RTT down channel, collecting bytes into `buf` until a
        /// `\n` is seen or the buffer is full.  Returns the number of bytes
        /// written (not including the newline).
        #[inline(never)]
        fn _deluge_test_rtt_readline(
            ch: &mut rtt_target::DownChannel,
            buf: &mut [u8; 64],
        ) -> usize {
            let mut pos = 0usize;
            loop {
                if pos >= buf.len() {
                    return pos;
                }
                let mut b = [0u8; 1];
                if ch.read(&mut b) == 1 {
                    if b[0] == b'\n' {
                        return pos;
                    }
                    buf[pos] = b[0];
                    pos += 1;
                }
            }
        }

        #runner_code

        #[panic_handler]
        fn _deluge_test_panic(info: &core::panic::PanicInfo) -> ! {
            let _idx = CURRENT_TEST_IDX.load(core::sync::atomic::Ordering::Relaxed);
            let _test_name: &str = if _idx == usize::MAX {
                "init"
            } else {
                #name_lookup
            };
            if let Some(_loc) = info.location() {
                rtt_target::rprintln!(
                    "FAIL:{}:{}:{}",
                    _test_name,
                    _loc.file(),
                    _loc.line(),
                );
            } else {
                rtt_target::rprintln!("FAIL:{}:?:0", _test_name);
            }
            rtt_target::rprintln!("DONE");
            // Drain RTT then self-jump (see dispatch code for explanation).
            unsafe {
                extern "C" { static _SEGGER_RTT: u8; }
                let base = &_SEGGER_RTT as *const u8 as usize;
                let wr = (base + 36) as *const u32;
                let rd = (base + 40) as *const u32;
                while core::ptr::read_volatile(wr) != core::ptr::read_volatile(rd) {
                    core::arch::asm!("nop");
                }
                core::arch::asm!("b _reset_handler", options(noreturn));
            }
        }
    };

    output.into()
}
