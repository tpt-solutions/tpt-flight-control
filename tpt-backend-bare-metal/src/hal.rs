//! Hardware abstraction layer for entry-class STM32 MCUs (F4 / F7 / H7).
//!
//! The crate is `#![no_std]` and builds both for the host (where a RAM mirror
//! of the peripherals is used, so the exact same driver code can be unit
//! tested) and for `thumbv7em-none-eabihf` (where the register block overlays
//! the real memory-mapped peripherals via volatile access).
//!
//! The two backends implement the same [`RegisterInterface`]; all driver logic
//! below is written against that interface and is therefore target-agnostic.

#![allow(dead_code)]

use core::ptr;

/// A GPIO port.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GpioBank {
    A,
    B,
    C,
}

/// A clock-gated peripheral (subset used by the bring-up).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Peripheral {
    GpioA,
    GpioB,
    GpioC,
    Usart2,
    Tim2,
}

/// Low-level register access contract. Implemented once per target.
pub trait RegisterInterface {
    /// Enable a peripheral clock in the RCC.
    fn rcc_enable(&mut self, p: Peripheral);
    /// Set the output data register of a GPIO bank (`set` bits only).
    fn gpio_set(&mut self, bank: GpioBank, pins: u32);
    /// Clear the output data register of a GPIO bank (`reset` bits only).
    fn gpio_clear(&mut self, bank: GpioBank, pins: u32);
    /// Read the input data register of a GPIO bank.
    fn gpio_read(&self, bank: GpioBank) -> u32;
    /// Configure `pins` of `bank` as push-pull output (MODER = 0b01).
    fn gpio_config_out(&mut self, bank: GpioBank, pins: u32);

    /// Write one byte to the telemetry USART data register (blocks until TXE).
    fn usart_write(&mut self, byte: u8);
    /// True when the USART transmit data register is empty.
    fn usart_tx_ready(&self) -> bool;

    /// Set the auto-reload value of TIM2 (period in timer ticks).
    fn tim_set_arr(&mut self, arr: u32);
    /// Current free-running microsecond clock derived from SysTick/TIM2.
    fn monotonic_micros(&self) -> u64;
    /// Advance the simulated clock by `us` (host only; no-op on real HW).
    fn advance_clock(&mut self, _us: u64) {}
}

// ---------------------------------------------------------------------------
// STM32F4 register block base addresses (representative; F7/H7 share layout).
// ---------------------------------------------------------------------------
const RCC_BASE: u32 = 0x4002_3800;
const GPIOA_BASE: u32 = 0x4002_0000;
const GPIOB_BASE: u32 = 0x4002_0400;
const GPIOC_BASE: u32 = 0x4002_0800;
const USART2_BASE: u32 = 0x4000_4400;
const TIM2_BASE: u32 = 0x4000_0000;
const TIM_CNT: u32 = 0x24;

const GPIO_MODER: u32 = 0x00;
const GPIO_ODR: u32 = 0x14;
const GPIO_IDR: u32 = 0x10;
const RCC_AHB1ENR: u32 = 0x30;
const RCC_APB1ENR: u32 = 0x40;
const USART_SR: u32 = 0x00;
const USART_DR: u32 = 0x04;
const TIM_ARR: u32 = 0x2C;

#[inline]
fn rd(base: u32, off: u32) -> u32 {
    unsafe { ptr::read_volatile((base + off) as *const u32) }
}
#[inline]
fn wr(base: u32, off: u32, val: u32) {
    unsafe { ptr::write_volatile((base + off) as *mut u32, val) }
}

fn gpio_base(bank: GpioBank) -> u32 {
    match bank {
        GpioBank::A => GPIOA_BASE,
        GpioBank::B => GPIOB_BASE,
        GpioBank::C => GPIOC_BASE,
    }
}

fn rcc_bit(p: Peripheral) -> (u32, u8) {
    match p {
        Peripheral::GpioA => (RCC_AHB1ENR, 0),
        Peripheral::GpioB => (RCC_AHB1ENR, 1),
        Peripheral::GpioC => (RCC_AHB1ENR, 2),
        Peripheral::Usart2 => (RCC_APB1ENR, 17),
        Peripheral::Tim2 => (RCC_APB1ENR, 0),
    }
}

/// Tiny append-only byte buffer used by the host backend to capture USART
/// output without `alloc`. Mirrors just enough of a `Vec<u8>` for tests.
#[derive(Clone)]
pub struct TxLog {
    buf: [u8; 256],
    len: usize,
}
impl Default for TxLog {
    fn default() -> Self {
        Self {
            buf: [0u8; 256],
            len: 0,
        }
    }
}
impl TxLog {
    pub fn push(&mut self, b: u8) {
        if self.len < self.buf.len() {
            self.buf[self.len] = b;
            self.len += 1;
        }
    }
    pub fn as_slice(&self) -> &[u8] {
        &self.buf[..self.len]
    }
    pub fn clear(&mut self) {
        self.len = 0;
    }
}

// ---------------------------------------------------------------------------
// Real-MMIO backend (target = ARM).
// ---------------------------------------------------------------------------
#[cfg(target_arch = "arm")]
mod backend {
    use super::*;

    /// Zero-sized handle overlaying the real peripherals.
    #[derive(Default, Clone)]
    pub struct Regs;

    impl RegisterInterface for Regs {
        fn rcc_enable(&mut self, p: Peripheral) {
            let (off, bit) = rcc_bit(p);
            let v = rd(RCC_BASE, off);
            wr(RCC_BASE, off, v | (1u32 << bit));
        }
        fn gpio_set(&mut self, bank: GpioBank, pins: u32) {
            let base = gpio_base(bank);
            let v = rd(base, GPIO_ODR);
            wr(base, GPIO_ODR, v | pins);
        }
        fn gpio_clear(&mut self, bank: GpioBank, pins: u32) {
            let base = gpio_base(bank);
            let v = rd(base, GPIO_ODR);
            wr(base, GPIO_ODR, v & !pins);
        }
        fn gpio_read(&self, bank: GpioBank) -> u32 {
            rd(gpio_base(bank), GPIO_IDR)
        }
        fn gpio_config_out(&mut self, bank: GpioBank, pins: u32) {
            let base = gpio_base(bank);
            let mut m = rd(base, GPIO_MODER);
            for pin in 0..16u32 {
                if (pins >> pin) & 1 != 0 {
                    m = (m & !(0b11 << (pin * 2))) | (0b01 << (pin * 2));
                }
            }
            wr(base, GPIO_MODER, m);
        }
        fn usart_write(&mut self, byte: u8) {
            while !self.usart_tx_ready() {}
            wr(USART2_BASE, USART_DR, byte as u32);
        }
        fn usart_tx_ready(&self) -> bool {
            (rd(USART2_BASE, USART_SR) & 0x80) != 0
        }
        fn tim_set_arr(&mut self, arr: u32) {
            wr(TIM2_BASE, TIM_ARR, arr);
        }
        fn monotonic_micros(&self) -> u64 {
            rd(TIM2_BASE, TIM_CNT) as u64
        }
    }
}

// ---------------------------------------------------------------------------
// Host backend: a by-value RAM mirror so the driver code can be exercised on
// the PC and inspected by tests. No globals / no `alloc` required.
// ---------------------------------------------------------------------------
#[cfg(not(target_arch = "arm"))]
mod backend {
    use super::*;

    /// In-RAM copy of the peripheral register state.
    #[derive(Default, Clone)]
    pub struct Regs {
        pub gpio_moder: [u32; 3],
        pub gpio_odr: [u32; 3],
        pub gpio_idr: [u32; 3],
        pub rcc_ahb1enr: u32,
        pub rcc_apb1enr: u32,
        pub usart_sr: u32,
        pub usart_dr: u32,
        pub tim_arr: u32,
        pub tim_cnt: u32,
        /// Simulated free-running microsecond clock.
        pub clock_us: u64,
        /// Bytes written to the USART (telemetry sink for tests).
        pub tx_log: TxLog,
    }

    impl RegisterInterface for Regs {
        fn rcc_enable(&mut self, p: Peripheral) {
            let (off, bit) = rcc_bit(p);
            if off == RCC_AHB1ENR {
                self.rcc_ahb1enr |= 1u32 << bit;
            } else {
                self.rcc_apb1enr |= 1u32 << bit;
            }
        }
        fn gpio_set(&mut self, bank: GpioBank, pins: u32) {
            self.gpio_odr[bank as usize] |= pins;
        }
        fn gpio_clear(&mut self, bank: GpioBank, pins: u32) {
            self.gpio_odr[bank as usize] &= !pins;
        }
        fn gpio_read(&self, bank: GpioBank) -> u32 {
            self.gpio_idr[bank as usize]
        }
        fn gpio_config_out(&mut self, bank: GpioBank, pins: u32) {
            let mut m = self.gpio_moder[bank as usize];
            for pin in 0..16u32 {
                if (pins >> pin) & 1 != 0 {
                    m = (m & !(0b11 << (pin * 2))) | (0b01 << (pin * 2));
                }
            }
            self.gpio_moder[bank as usize] = m;
        }
        fn usart_write(&mut self, byte: u8) {
            while !self.usart_tx_ready() {}
            self.usart_dr = byte as u32;
            self.tx_log.push(byte);
        }
        fn usart_tx_ready(&self) -> bool {
            (self.usart_sr & 0x80) != 0
        }
        fn tim_set_arr(&mut self, arr: u32) {
            self.tim_arr = arr;
        }
        fn monotonic_micros(&self) -> u64 {
            self.clock_us
        }
        fn advance_clock(&mut self, us: u64) {
            self.clock_us += us;
        }
    }

    impl Regs {
        /// Construct a host mirror with the USART TX-ready flag pre-set.
        pub fn new() -> Self {
            Self {
                usart_sr: 0x80,
                ..Default::default()
            }
        }
    }
}

pub use backend::Regs;

/// Bring up the minimum peripherals for flight: GPIO for status/motor
/// indicators, USART2 for telemetry, TIM2 for the scheduler tick.
pub fn init_clocks_and_io(regs: &mut impl RegisterInterface) {
    regs.rcc_enable(Peripheral::GpioA);
    regs.rcc_enable(Peripheral::GpioB);
    regs.rcc_enable(Peripheral::GpioC);
    regs.rcc_enable(Peripheral::Usart2);
    regs.rcc_enable(Peripheral::Tim2);
    // Configure PB0..PB3 as outputs (motor arm/enable indicators).
    regs.gpio_config_out(GpioBank::B, 0x000F);
    // 1 kHz scheduler tick on TIM2.
    regs.tim_set_arr(1000);
}

/// Transmit a byte slice over the telemetry USART.
pub fn write_telemetry(regs: &mut impl RegisterInterface, bytes: &[u8]) {
    for &b in bytes {
        regs.usart_write(b);
    }
}
