//! Bring-up, the link to the host, and the measurement.
//!
//! The host sends one-byte commands over USB CDC, polled: no interrupt is ever unmasked, so
//! nothing preempts a timed call, and interrupts are masked around it besides.
//!
//! | byte | reply |
//! | --- | --- |
//! | `i` | one line, `onboard proto=… build=… commit=… rustc=… opt=… lto=… cpu=… fpu=… sysclk=… cache=… fz=…` |
//! | `n` | a fresh [`Machine`], before a run's first batch |
//! | `w`, `c` | warm or cold: `c` invalidates both caches before every timed call |
//! | `b` | a `u32` length and that many bytes of whole frames; replies `R`, a `u32` count, and per frame cycles (`u32`), stack bytes (`u32`), the outcome (`u16`) and flags (`u8`), and a pad byte |
//! | `f` | `fmodf(2^k, 2π)` timed for `k` from −8 to 127; replies `F`, a `u32` count and `(k, cycles)` as two `i32`/`u32` |
//! | `r` | reboots into the ROM bootloader, for `dfu-util` |
//!
//! Flags: 1 the outcome matches the host's, 2 the digest does, 4 a denormal was an input
//! (`FPSCR.IDC`), 8 the call ran off the painted stack, 16 the record did not decode.

use core::arch::asm;
use core::mem::MaybeUninit;
use core::ptr::{addr_of_mut, read_volatile, write_volatile};
use core::sync::atomic::{Ordering, compiler_fence};

use cortex_m::peripheral::{CPUID, DWT, SCB};
use cortex_m_rt::entry;
use stm32h7xx_hal::{
    pac,
    prelude::*,
    rcc::rec::UsbClkSel,
    usb_hs::{USB2, UsbBus},
};
use usb_device::{bus::UsbBusAllocator, prelude::*};
use usbd_serial::SerialPort;

use onboard::{FRAME_OVERHEAD, Frame, Machine, Outcome, Record, Returned};

/// Bumped when a command or a reply changes shape; `tools/onboard.py` refuses another.
const PROTOCOL: u32 = 1;
const SYSCLK: u32 = 400_000_000;

/// A batch of whole frames, and room for a result per frame of the smallest size.
const BATCH: usize = 256 * 1024;
const RESULT: usize = 12;
const MAX_CALLS: usize = BATCH / (1 + FRAME_OVERHEAD);

/// The stack below a timed call that is painted, and so the deepest it can measure: the host
/// walk's peak is under 12 KB (`data/footprint.txt`, `stack_peak`).
const PAINTED: usize = 32 * 1024;
const PAINT: u32 = 0x5AA5_C33C;

/// AXI SRAM, 512 KB that nothing else on this firmware uses: the batch, then its results.
const AXI_SRAM: usize = 0x2400_0000;
const AXI_SRAM_SIZE: usize = 512 * 1024;
const _: () = assert!(BATCH + MAX_CALLS * RESULT <= AXI_SRAM_SIZE);
static mut EP_MEMORY: MaybeUninit<[u32; 1024]> = MaybeUninit::uninit();

unsafe extern "C" {
    /// The end of `.bss` in DTCM, from cortex-m-rt's link script: the painted stack must stay
    /// above it.
    static __ebss: u32;
}

// --- Reboot to DFU, as ark-fpv-discovery's `src/main.rs` at b07f131 does it -------------------

const BOOTLOADER_MAGIC: u32 = 0xB007_0DF1;
/// The STM32H743's system-memory bootloader (ST AN2606).
const SYSTEM_BOOTLOADER: *const u32 = 0x1FF0_9800 as *const u32;

/// In `.uninit`, which cortex-m-rt does not zero, so it survives the reset that carries it.
#[unsafe(link_section = ".uninit.BOOT_FLAG")]
static mut BOOT_FLAG: MaybeUninit<u32> = MaybeUninit::uninit();

fn reboot_to_bootloader() -> ! {
    unsafe {
        addr_of_mut!(BOOT_FLAG)
            .cast::<u32>()
            .write_volatile(BOOTLOADER_MAGIC)
    };
    SCB::sys_reset();
}

/// First thing after reset, before any clock is touched.
fn maybe_enter_bootloader() {
    unsafe {
        let flag = addr_of_mut!(BOOT_FLAG).cast::<u32>();
        if flag.read_volatile() == BOOTLOADER_MAGIC {
            flag.write_volatile(0);
            cortex_m::asm::bootload(SYSTEM_BOOTLOADER);
        }
    }
}

// --- The link ------------------------------------------------------------------------------

struct Link<'a> {
    device: UsbDevice<'a, UsbBus<USB2>>,
    serial: SerialPort<'a, UsbBus<USB2>>,
}

impl Link<'_> {
    fn poll(&mut self) {
        self.device.poll(&mut [&mut self.serial]);
    }

    fn read_byte(&mut self) -> u8 {
        let mut byte = [0u8];
        self.read_exact(&mut byte);
        byte[0]
    }

    fn read_exact(&mut self, buffer: &mut [u8]) {
        let mut at = 0;
        while at < buffer.len() {
            self.poll();
            if let Ok(n) = self.serial.read(&mut buffer[at..]) {
                at += n;
            }
        }
    }

    fn read_u32(&mut self) -> u32 {
        let mut bytes = [0u8; 4];
        self.read_exact(&mut bytes);
        u32::from_le_bytes(bytes)
    }

    fn write_all(&mut self, mut bytes: &[u8]) {
        while !bytes.is_empty() {
            self.poll();
            if let Ok(n) = self.serial.write(bytes) {
                bytes = &bytes[n..];
            }
        }
        while self.serial.flush().is_err() {
            self.poll();
        }
    }
}

/// A line of text, formatted without the heap; it reports no `f32`, so formatting reaches no
/// float code.
struct Line {
    bytes: [u8; 256],
    length: usize,
}

impl core::fmt::Write for Line {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let end = self.length + s.len();
        let slot = self
            .bytes
            .get_mut(self.length..end)
            .ok_or(core::fmt::Error)?;
        slot.copy_from_slice(s.as_bytes());
        self.length = end;
        Ok(())
    }
}

// --- Measurement ---------------------------------------------------------------------------

struct Measured {
    cycles: u32,
    stack: u32,
    returned: Returned,
    denormal: bool,
    overflowed: bool,
}

fn stack_pointer() -> usize {
    let sp: usize;
    unsafe { asm!("mov {}, sp", out(reg) sp, options(nomem, nostack, preserves_flags)) };
    sp
}

fn fpscr() -> u32 {
    let bits: u32;
    unsafe { asm!("vmrs {}, fpscr", out(reg) bits, options(nomem, nostack)) };
    bits
}

fn set_fpscr(bits: u32) {
    unsafe { asm!("vmsr fpscr, {}", in(reg) bits, options(nomem, nostack)) };
}

/// `FPSCR.IDC`, input denormal cumulative.
const IDC: u32 = 1 << 7;
/// `FPSCR.FZ`, flush to zero: off at reset, and left off, so the board's arithmetic is the
/// host's bit for bit.
const FZ: u32 = 1 << 24;

/// Make one call, timed, with the stack beneath it painted.
///
/// Everything between painting and scanning is this function's own frame and the call: no
/// other function runs below its stack pointer, which is why the barriers, the register reads
/// and both loops are inline here rather than calls that would leave frames in the paint.
#[inline(never)]
fn measure(machine: &mut Machine, record: &Record) -> Measured {
    let top = stack_pointer();
    let bottom = top - PAINTED;
    let mut word = bottom;
    while word < top {
        unsafe { write_volatile(word as *mut u32, PAINT) };
        word += 4;
    }
    set_fpscr(fpscr() & !IDC);
    unsafe { asm!("cpsid i", "dsb", "isb", options(nostack)) };
    compiler_fence(Ordering::SeqCst);
    let start = DWT::cycle_count();
    let returned = machine.execute(record);
    let end = DWT::cycle_count();
    compiler_fence(Ordering::SeqCst);
    unsafe { asm!("cpsie i", options(nostack)) };
    let denormal = fpscr() & IDC != 0;
    let mut lowest = bottom;
    while lowest < top && unsafe { read_volatile(lowest as *const u32) } == PAINT {
        lowest += 4;
    }
    Measured {
        cycles: end.wrapping_sub(start),
        stack: (top - lowest) as u32,
        returned,
        denormal,
        overflowed: lowest == bottom,
    }
}

/// Run every whole frame in `batch`, writing a result per frame; the count.
fn run(
    machine: &mut Machine,
    batch: &[u8],
    results: &mut [u8],
    cold: bool,
    scb: &mut SCB,
    cpuid: &mut CPUID,
) -> usize {
    let mut rest = batch;
    let mut count = 0;
    while let Some((frame, after)) = Frame::split(rest) {
        rest = after;
        let Some(slot) = results.get_mut(count * RESULT..(count + 1) * RESULT) else {
            break;
        };
        let mut flags = 0u8;
        let (cycles, stack, outcome) = match Record::decode(frame.record) {
            Some(record) => {
                if cold {
                    scb.invalidate_icache();
                    scb.clean_invalidate_dcache(cpuid);
                }
                let measured = measure(machine, &record);
                let outcome = Outcome::of(&measured.returned);
                flags |= u8::from(outcome == frame.outcome);
                flags |= u8::from(machine.digest(outcome) == frame.digest) << 1;
                flags |= u8::from(measured.denormal) << 2;
                flags |= u8::from(measured.overflowed) << 3;
                (measured.cycles, measured.stack, outcome.0)
            }
            None => {
                flags |= 1 << 4;
                (0, 0, 0)
            }
        };
        slot[..4].copy_from_slice(&cycles.to_le_bytes());
        slot[4..8].copy_from_slice(&stack.to_le_bytes());
        slot[8..10].copy_from_slice(&outcome.to_le_bytes());
        slot[10] = flags;
        slot[11] = 0;
        count += 1;
    }
    count
}

/// `fmodf(2^k, 2π)`, the reduction `wrap_pi` makes, timed across the exponent gap
/// (`DESIGN.md`, "Execution time bounded by constants").
fn sweep_fmodf(link: &mut Link) {
    const FIRST: i32 = -8;
    const LAST: i32 = 127;
    let count = (LAST - FIRST + 1) as u32;
    link.write_all(b"F");
    link.write_all(&count.to_le_bytes());
    for k in FIRST..=LAST {
        let x = f32::from_bits(((k + 127) as u32) << 23);
        let x = core::hint::black_box(x);
        unsafe { asm!("cpsid i", "dsb", "isb", options(nostack)) };
        let start = DWT::cycle_count();
        let r = libm::fmodf(x, core::f32::consts::TAU);
        let end = DWT::cycle_count();
        unsafe { asm!("cpsie i", options(nostack)) };
        core::hint::black_box(r);
        link.write_all(&k.to_le_bytes());
        link.write_all(&end.wrapping_sub(start).to_le_bytes());
    }
}

fn info(link: &mut Link, cold: bool) {
    use core::fmt::Write;
    let mut line = Line {
        bytes: [0; 256],
        length: 0,
    };
    let _ = writeln!(
        line,
        "onboard proto={PROTOCOL} build={} commit={} rustc={} opt={} lto={} cpu={} fpu={} \
         sysclk={SYSCLK} cache={} fz={}",
        env!("ONBOARD_BUILD"),
        env!("ONBOARD_COMMIT"),
        env!("ONBOARD_RUSTC"),
        env!("ONBOARD_OPT"),
        env!("ONBOARD_LTO"),
        env!("ONBOARD_CPU"),
        env!("ONBOARD_FPU"),
        if cold { "cold" } else { "warm" },
        u8::from(fpscr() & FZ != 0),
    );
    let length = line.length;
    link.write_all(&line.bytes[..length]);
}

// --- Status LEDs ---------------------------------------------------------------------------
//
// With no probe, the LEDs are the only report from before USB enumerates: red at `main`, green
// once the clocks run, blue once USB is built, white on a hard fault, yellow on a panic. Red
// `PE3`, green `PE4`, blue `PE5`, active low (ark-fpv-discovery's `docs/ark-fpv-board.md`).
// Raw registers rather than the HAL's pins, so the fault handlers can reach them.

const RCC_AHB4ENR: *mut u32 = 0x5802_44E0 as *mut u32;
const GPIOE_MODER: *mut u32 = 0x5802_1000 as *mut u32;
const GPIOE_BSRR: *mut u32 = 0x5802_1018 as *mut u32;

fn leds_init() {
    unsafe {
        write_volatile(RCC_AHB4ENR, read_volatile(RCC_AHB4ENR) | 1 << 4);
        let _ = read_volatile(RCC_AHB4ENR);
        let moder = read_volatile(GPIOE_MODER) & !(0b11_11_11 << 6);
        write_volatile(GPIOE_MODER, moder | 0b01_01_01 << 6);
    }
    leds(false, false, false);
}

fn leds(red: bool, green: bool, blue: bool) {
    let mut bsrr = 0u32;
    for (pin, on) in [(3, red), (4, green), (5, blue)] {
        // Low lights: reset bit to turn on, set bit to turn off.
        bsrr |= if on { 1 << (pin + 16) } else { 1 << pin };
    }
    unsafe { write_volatile(GPIOE_BSRR, bsrr) };
}

#[cortex_m_rt::exception]
unsafe fn HardFault(_: &cortex_m_rt::ExceptionFrame) -> ! {
    leds(true, true, true);
    loop {
        cortex_m::asm::nop();
    }
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    leds(true, true, false);
    loop {
        cortex_m::asm::nop();
    }
}

/// Bring-up failed: nothing to report it over, so stop.
fn halt() -> ! {
    loop {
        cortex_m::asm::wfi();
    }
}

/// Zero a buffer in place and hand it out: `MaybeUninit::write([0; N])` may build the array on
/// the stack first.
///
/// # Safety
///
/// Called once per static, at boot, so the reference it returns is the only one.
unsafe fn zeroed<T, const N: usize>(slot: *mut MaybeUninit<[T; N]>) -> &'static mut [T; N] {
    unsafe {
        slot.cast::<u8>().write_bytes(0, size_of::<[T; N]>());
        &mut *slot.cast::<[T; N]>()
    }
}

#[entry]
fn main() -> ! {
    maybe_enter_bootloader();
    leds_init();
    leds(true, false, false);
    let (Some(mut cp), Some(dp)) = (cortex_m::Peripherals::take(), pac::Peripherals::take()) else {
        halt()
    };

    let pwr = dp.PWR.constrain();
    let vos = pwr.freeze();
    let rcc = dp.RCC.constrain();
    let mut ccdr = rcc.sys_ck(SYSCLK.Hz()).freeze(vos, &dp.SYSCFG);
    let _ = ccdr.clocks.hsi48_ck();
    ccdr.peripheral.kernel_usb_clk_mux(UsbClkSel::Hsi48);
    leds(false, true, false);

    cp.SCB.enable_icache();
    cp.SCB.enable_dcache(&mut cp.CPUID);
    cp.DCB.enable_trace();
    DWT::unlock();
    cp.DWT.enable_cycle_counter();

    let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
    let usb = USB2::new(
        dp.OTG2_HS_GLOBAL,
        dp.OTG2_HS_DEVICE,
        dp.OTG2_HS_PWRCLK,
        gpioa.pa11.into_alternate(),
        gpioa.pa12.into_alternate(),
        ccdr.peripheral.USB2OTG,
        &ccdr.clocks,
    );
    let ep_memory = unsafe { zeroed(addr_of_mut!(EP_MEMORY)) };
    let Some(bus) =
        cortex_m::singleton!(: UsbBusAllocator<UsbBus<USB2>> = UsbBus::new(usb, ep_memory))
    else {
        halt()
    };
    let serial = SerialPort::new(bus);
    let Ok(builder) = UsbDeviceBuilder::new(bus, UsbVidPid(0x1209, 0x0001)).strings(&[
        usb_device::device::StringDescriptors::default()
            .manufacturer("fusion-nav")
            .product("onboard (#41)")
            .serial_number("001"),
    ]) else {
        halt()
    };
    let device = builder.device_class(usbd_serial::USB_CLASS_CDC).build();
    let mut link = Link { device, serial };
    leds(false, false, true);

    // Taken once, here: no other code names AXI SRAM, and the two do not overlap.
    let (batch, results) = unsafe {
        let base = AXI_SRAM as *mut u8;
        base.write_bytes(0, BATCH + MAX_CALLS * RESULT);
        (
            core::slice::from_raw_parts_mut(base, BATCH),
            core::slice::from_raw_parts_mut(base.add(BATCH), MAX_CALLS * RESULT),
        )
    };
    let mut machine = Machine::default();
    let mut cold = false;

    loop {
        match link.read_byte() {
            b'i' => info(&mut link, cold),
            b'n' => machine = Machine::default(),
            b'w' => cold = false,
            b'c' => cold = true,
            b'r' => reboot_to_bootloader(),
            b'f' => sweep_fmodf(&mut link),
            b'b' => {
                let length = link.read_u32() as usize;
                let Some(bytes) = batch.get_mut(..length) else {
                    // Too long for the buffer: answer with no results, which the host refuses.
                    link.write_all(b"R");
                    link.write_all(&0u32.to_le_bytes());
                    continue;
                };
                link.read_exact(bytes);
                // The painted region must clear `.bss`; checked where `measure` will run, one
                // frame below this one.
                let ebss = core::ptr::addr_of!(__ebss) as usize;
                let count = if stack_pointer() - 4096 - PAINTED > ebss {
                    run(
                        &mut machine,
                        bytes,
                        results,
                        cold,
                        &mut cp.SCB,
                        &mut cp.CPUID,
                    )
                } else {
                    0
                };
                link.write_all(b"R");
                link.write_all(&(count as u32).to_le_bytes());
                link.write_all(&results[..count * RESULT]);
            }
            _ => {}
        }
    }
}
