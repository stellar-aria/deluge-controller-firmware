//! deluge-test-runner — Cargo test runner for RZA1L (Cortex-A9) targets.
//!
//! Uses `JLinkGDBServerCLExe` + `arm-none-eabi-gdb` for target control and
//! the JLink RTT telnet port for bidirectional I/O with the test firmware.
//!
//! ## Protocol (RTT channel 0; device→host unless noted)
//!
//!   TESTS:<n>\n                — number of tests
//!   <name>:<timeout_secs>\n   — one line per test
//!   READY\n                   — ready to receive RUN commands
//!   → RUN:<name>\n            — host→device
//!   PASS:<name>\n             — test passed; device self-jumps
//!   FAIL:<name>:<file>:<line> — test failed; device self-jumps
//!   DONE\n                    — after PASS/FAIL; device is mid-self-jump
//!
//! ## Target control (GDB commands — same as launch.json "Rust firmware" config)
//!
//!   monitor reset                           — JLink script: halt + SYSCR3=0x0F
//!   monitor cp15 1,0,0,0  = SCTLR_SAFE     — disable MMU/caches
//!   monitor cp15 12,0,0,0 = entry_addr      — set VBAR
//!   monitor exec SetRTTAddr <addr>           — tell JLink exact RTT block address (no scan)
//!   load <elf>                              — write image to SRAM via SWD
//!   set $cpsr = 0x1DF                       — SYS mode, ARM state
//!   set $pc   = entry_addr
//!   continue

use anyhow::{bail, Context, Result};
use clap::Parser;
use object::{Object, ObjectSymbol};
use std::io::{Read, Write};
use std::net::TcpStream;
use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};
use std::time::{Duration, Instant};

// SCTLR: MMU off, D-cache off, I-cache off, unaligned access OK.
// Matches "monitor cp15 1, 0, 0, 0 = 0x00C50078" in launch.json.
const SCTLR_SAFE: u32 = 0x00C5_0078;
// CPSR: SYS mode (0x1F), ARM state (T=0), A/I/F masked.
const CPSR_SYS_ARM: u32 = 0x0000_01DF;

// ─── CLI args ────────────────────────────────────────────────────────────────

#[derive(Parser)]
#[clap(about = "Cargo test runner for RZA1L Cortex-A9 via JLink GDB + RTT")]
struct Args {
    /// ELF test binary (passed by cargo as the test runner argument).
    elf: PathBuf,

    /// Target device name.
    #[clap(long, default_value = "R7S721020")]
    device: String,

    /// Debug probe interface.
    #[clap(long, default_value = "SWD")]
    interface: String,

    /// arm-none-eabi-gdb binary.
    #[clap(long, default_value = "arm-none-eabi-gdb")]
    gdb: String,

    /// JLinkGDBServerCLExe binary.
    #[clap(long, default_value = "JLinkGDBServerCLExe")]
    jlink_server: String,

    /// Path to rza1_debug.JLinkScript (looked up in cwd by default).
    #[clap(long, default_value = "rza1_debug.JLinkScript")]
    jlink_script: PathBuf,

    /// JLink GDB server port.
    #[clap(long, default_value_t = 2331u16)]
    gdb_port: u16,

    /// JLink RTT telnet port.
    #[clap(long, default_value_t = 19021u16)]
    rtt_port: u16,

    /// Only run tests whose name contains this substring.
    #[clap(long)]
    filter: Option<String>,

    /// List test names (requires hardware).
    #[clap(long, hide = true)]
    list: bool,

    /// Print every RTT line to stderr as it arrives.
    #[clap(long, short = 'v')]
    verbose: bool,
}

// ─── ELF helpers ─────────────────────────────────────────────────────────────

fn elf_entry(data: &[u8]) -> Result<u32> {
    let obj = object::File::parse(data).context("parsing ELF")?;
    Ok(obj.entry() as u32)
}

fn elf_rtt_addr(data: &[u8]) -> Option<u32> {
    let obj = object::File::parse(data).ok()?;
    obj.symbols()
        .find(|s| s.name().ok().map_or(false, |n| n == "_SEGGER_RTT"))
        .map(|s| s.address() as u32)
}

// ─── JLink GDB server ────────────────────────────────────────────────────────

struct JLinkServer {
    child: Child,
}

impl JLinkServer {
    fn start(args: &Args) -> Result<Self> {
        let mut cmd = Command::new(&args.jlink_server);
        cmd.args([
            "-device",        &args.device,
            "-if",            &args.interface,
            "-endian",        "little",
            "-nogui",
            "-ir",
            "-port",          &args.gdb_port.to_string(),
            "-rtttelnetport", &args.rtt_port.to_string(),
        ]);

        if args.jlink_script.exists() {
            cmd.args(["-jlinkscriptfile", &args.jlink_script.to_string_lossy()]);
            eprintln!("[deluge-test-runner] JLink script: {}", args.jlink_script.display());
        } else {
            eprintln!(
                "[deluge-test-runner] WARNING: JLink script not found at {} \
                 — 'monitor reset' will do a hardware reset",
                args.jlink_script.display()
            );
        }

        cmd.stdout(Stdio::null()).stderr(Stdio::null());
        let child = cmd.spawn().context("spawning JLinkGDBServerCLExe")?;

        // Wait until the GDB port is accepting connections (≤15 s).
        let deadline = Instant::now() + Duration::from_secs(15);
        loop {
            if Instant::now() > deadline {
                bail!("JLinkGDBServerCLExe did not open port {} within 15 s", args.gdb_port);
            }
            if TcpStream::connect(format!("127.0.0.1:{}", args.gdb_port)).is_ok() {
                break;
            }
            std::thread::sleep(Duration::from_millis(100));
        }
        eprintln!("[deluge-test-runner] JLink GDB server ready on port {}", args.gdb_port);
        Ok(JLinkServer { child })
    }
}

impl Drop for JLinkServer {
    fn drop(&mut self) {
        let _ = self.child.kill();
        let _ = self.child.wait();
    }
}

// ─── GDB batch invocations ───────────────────────────────────────────────────

/// Spawn arm-none-eabi-gdb --batch.  Each element of `cmds` becomes a `-ex`.
/// Returns immediately; the child blocks on `continue`.
fn gdb_spawn(gdb: &str, cmds: &[String]) -> Result<Child> {
    let mut cmd = Command::new(gdb);
    cmd.arg("--batch");
    for c in cmds {
        cmd.args(["-ex", c.as_str()]);
    }
    cmd.stdout(Stdio::null()).stderr(Stdio::null());
    cmd.spawn().context("spawning arm-none-eabi-gdb")
}

/// Flash the ELF and start it.  GDB blocks on `continue`; call returns the child.
fn gdb_flash_and_run(args: &Args, elf: &Path, entry_addr: u32, rtt_addr: Option<u32>) -> Result<Child> {
    let target  = format!("target remote :{}", args.gdb_port);
    let sctlr   = format!("monitor cp15 1, 0, 0, 0 = {SCTLR_SAFE:#010x}");
    let vbar    = format!("monitor cp15 12, 0, 0, 0 = {entry_addr:#010x}");
    let load    = format!("load {}", elf.display());
    // SetRTTAddr bypasses memory scanning — JLink knows the address immediately.
    // Fall back to searching the full SRAM region if the symbol wasn't found.
    let rtt_hint = match rtt_addr {
        Some(a) => format!("monitor exec SetRTTAddr {a:#010x}"),
        None    => "monitor exec SetRTTSearchRanges 0x20010000 0x20060000".to_owned(),
    };
    let cpsr = format!("set $cpsr = {CPSR_SYS_ARM:#010x}");
    let pc   = format!("set $pc = {entry_addr:#010x}");

    eprintln!("[deluge-test-runner] GDB: flash + run @ {entry_addr:#010x}");
    gdb_spawn(&args.gdb, &[
        target, "monitor reset".into(), sctlr, vbar,
        load, rtt_hint, cpsr, pc, "continue".into(),
    ])
}

/// Halt the running target, restore CPU state, resume — no re-flash.
fn gdb_restart(args: &Args, entry_addr: u32) -> Result<Child> {
    let target = format!("target remote :{}", args.gdb_port);
    let sctlr  = format!("monitor cp15 1, 0, 0, 0 = {SCTLR_SAFE:#010x}");
    let vbar   = format!("monitor cp15 12, 0, 0, 0 = {entry_addr:#010x}");
    let cpsr   = format!("set $cpsr = {CPSR_SYS_ARM:#010x}");
    let pc     = format!("set $pc = {entry_addr:#010x}");

    eprintln!("[deluge-test-runner] GDB: restart @ {entry_addr:#010x}");
    gdb_spawn(&args.gdb, &[
        target, "monitor halt".into(), sctlr, vbar, cpsr, pc, "continue".into(),
    ])
}

// ─── RTT TCP helpers ─────────────────────────────────────────────────────────

struct Rtt {
    stream:  TcpStream,
    acc:     String,
    verbose: bool,
}

impl Rtt {
    fn connect(port: u16, timeout: Duration) -> Result<Self> {
        let deadline = Instant::now() + timeout;
        loop {
            match TcpStream::connect(format!("127.0.0.1:{port}")) {
                Ok(s) => {
                    s.set_read_timeout(Some(Duration::from_millis(20)))?;
                    eprintln!("[deluge-test-runner] RTT connected on port {port}");
                    return Ok(Rtt { stream: s, acc: String::new(), verbose: false });
                }
                Err(_) if Instant::now() < deadline => {
                    std::thread::sleep(Duration::from_millis(100));
                }
                Err(e) => return Err(e).with_context(|| format!("connecting to RTT port {port}")),
            }
        }
    }

    fn writeln(&mut self, msg: &str) -> Result<()> {
        write!(self.stream, "{msg}\n").context("RTT write")
    }

    /// Read one complete line (stripped of CR/LF), or `None` on deadline.
    fn readline(&mut self, deadline: Instant) -> Result<Option<String>> {
        let mut buf = [0u8; 256];
        loop {
            if let Some(pos) = self.acc.find('\n') {
                let line: String = self.acc.drain(..=pos).collect();
                let s = line.trim_end_matches('\n').trim_end_matches('\r').to_owned();
                if !s.is_empty() {
                    if self.verbose {
                        eprintln!("[RTT] {s}");
                    }
                    return Ok(Some(s));
                }
                continue;
            }

            if Instant::now() > deadline {
                return Ok(None);
            }

            match self.stream.read(&mut buf) {
                Ok(0) => return Err(anyhow::anyhow!("RTT TCP connection closed by server")),
                Ok(n) => { self.acc.push_str(std::str::from_utf8(&buf[..n]).unwrap_or("")); }
                Err(ref e)
                    if e.kind() == std::io::ErrorKind::WouldBlock
                    || e.kind() == std::io::ErrorKind::TimedOut => {}
                Err(e) => return Err(e).context("RTT read"),
            }
        }
    }

    /// Discard lines until `pred` matches; return the matching line or `None` on deadline.
    fn wait_for(&mut self, pred: impl Fn(&str) -> bool, deadline: Instant) -> Result<Option<String>> {
        loop {
            match self.readline(deadline)? {
                None           => return Ok(None),
                Some(l) if pred(&l) => return Ok(Some(l)),
                _ => {}
            }
        }
    }
}

// ─── Main ─────────────────────────────────────────────────────────────────

fn main() -> Result<()> {
    let args = Args::parse();

    let elf_data   = std::fs::read(&args.elf)
        .with_context(|| format!("reading ELF {}", args.elf.display()))?;
    let entry_addr = elf_entry(&elf_data)?;
    let rtt_addr   = elf_rtt_addr(&elf_data);

    eprintln!("[deluge-test-runner] ELF:         {}", args.elf.display());
    eprintln!("[deluge-test-runner] entry:        {entry_addr:#010x}");
    if let Some(a) = rtt_addr {
        eprintln!("[deluge-test-runner] _SEGGER_RTT: {a:#010x}");
    }

    if args.list {
        eprintln!("[deluge-test-runner] --list requires hardware; run without --list");
        return Ok(());
    }

    // ── Start JLink GDB server ────────────────────────────────────────────
    let _jlink = JLinkServer::start(&args)?;

    // ── Flash + run ───────────────────────────────────────────────────────
    let mut gdb = gdb_flash_and_run(&args, &args.elf, entry_addr, rtt_addr)?;

    // ── Connect to RTT ────────────────────────────────────────────────────
    let mut rtt = Rtt::connect(args.rtt_port, Duration::from_secs(15))
        .context("RTT connect — ensure JLinkGDBServerCLExe is started with -rtttelnetport")?;
    rtt.verbose = args.verbose;

    // ── Parse TESTS: announcement ─────────────────────────────────────────
    let tests_line = rtt
        .wait_for(|l| l.starts_with("TESTS:"), Instant::now() + Duration::from_secs(15))?
        .context("timed out waiting for TESTS: announcement")?;
    let n_tests: usize = tests_line["TESTS:".len()..].trim().parse()
        .context("parsing test count")?;

    let mut all_tests: Vec<(String, u64)> = Vec::with_capacity(n_tests);
    for _ in 0..n_tests {
        let line = rtt
            .readline(Instant::now() + Duration::from_secs(5))?
            .context("timed out reading test announce line")?;
        let (name, timeout) = line
            .rsplit_once(':')
            .map(|(n, t)| (n.to_owned(), t.trim().parse::<u64>().unwrap_or(60)))
            .unwrap_or_else(|| (line, 60));
        all_tests.push((name, timeout));
    }

    rtt.wait_for(|l| l == "READY", Instant::now() + Duration::from_secs(5))?
        .context("timed out waiting for READY")?;

    let tests_to_run: Vec<_> = all_tests.iter()
        .filter(|(name, _)| args.filter.as_deref().map_or(true, |f| name.contains(f)))
        .collect();

    eprintln!(
        "[deluge-test-runner] running {}/{} tests",
        tests_to_run.len(), all_tests.len()
    );

    // ── Run tests ────────────────────────────────────────────────────────
    let mut passed = 0usize;
    let mut failed = 0usize;

    for (test_name, timeout_secs) in &tests_to_run {
        println!("test {test_name} ...");
        let deadline = Instant::now() + Duration::from_secs(*timeout_secs);

        let run_cmd = format!("RUN:{test_name}");
        if args.verbose { eprintln!("[runner→RTT] {run_cmd}"); }
        rtt.writeln(&run_cmd)?;

        let result = rtt.wait_for(
            |l| l.starts_with("PASS:") || l.starts_with("FAIL:"),
            deadline,
        );
        // Consume DONE before the firmware fully self-jumps.
        let _ = rtt.wait_for(|l| l == "DONE", Instant::now() + Duration::from_secs(3));

        match result? {
            None => {
                println!("test {test_name} ... FAILED (TIMEOUT after {timeout_secs}s)");
                failed += 1;
                // Kill GDB, drop RTT, restart cleanly.
                let _ = gdb.kill();
                let _ = gdb.wait();
                drop(rtt);
                gdb = gdb_restart(&args, entry_addr)?;
                std::thread::sleep(Duration::from_millis(500));
                rtt = Rtt::connect(args.rtt_port, Duration::from_secs(15))
                    .context("RTT reconnect after timeout")?;
                rtt.verbose = args.verbose;
                let _ = rtt.wait_for(|l| l == "READY", Instant::now() + Duration::from_secs(10));
            }
            Some(line) if line.starts_with("PASS:") => {
                println!("test {test_name} ... ok");
                passed += 1;
                let _ = rtt.wait_for(|l| l == "READY", Instant::now() + Duration::from_secs(10));
            }
            Some(line) => {
                let detail = line.strip_prefix("FAIL:").unwrap_or(&line).to_owned();
                println!("test {test_name} ... FAILED");
                eprintln!("  {detail}");
                failed += 1;
                let _ = rtt.wait_for(|l| l == "READY", Instant::now() + Duration::from_secs(10));
            }
        }
    }

    // ── Summary ──────────────────────────────────────────────────────────
    let status = if failed == 0 { "ok" } else { "FAILED" };
    println!(
        "\ntest result: {status}. {passed} passed; {failed} failed; \
         0 ignored; 0 measured; 0 filtered out\n"
    );

    let _ = gdb.kill();
    let _ = gdb.wait();
    // _jlink dropped here — kills JLink GDB server

    if failed > 0 {
        std::process::exit(1);
    }
    Ok(())
}
