#![no_std]
#![no_main]

use cortex_m_rt::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::{InputPin, OutputPin};
use embedded_time::fixed_point::FixedPoint;
use panic_probe as _;

use rp_pico as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    gpio::DynPin,
    multicore::{Multicore, Stack},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

use bitvec::prelude::*;
use itertools::enumerate;

static mut CORE1_STACK: Stack<4096> = Stack::new();
fn core1_task() -> ! {
    let mut pac = unsafe { pac::Peripherals::steal() };
    let core = unsafe { pac::CorePeripherals::steal() };
    let mut sio = Sio::new(pac.SIO);
    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let sys_freq = sio.fifo.read_blocking();
    let mut delay = cortex_m::delay::Delay::new(core.SYST, sys_freq);

    let mut cols: [DynPin; 8] = [
        pins.gpio7.into(),
        pins.gpio8.into(),
        pins.gpio9.into(),
        pins.gpio10.into(),
        pins.gpio11.into(),
        pins.gpio12.into(),
        pins.gpio13.into(),
        pins.gpio14.into(),
    ];
    for col in &mut cols {
        col.into_pull_up_input();
    }

    let mut rows: [DynPin; 7] = [
        pins.gpio0.into(),
        pins.gpio1.into(),
        pins.gpio2.into(),
        pins.gpio3.into(),
        pins.gpio4.into(),
        pins.gpio5.into(),
        pins.gpio6.into(),
    ];
    for row in &mut rows {
        row.into_push_pull_output();
        row.set_high().unwrap();
    }

    let mut prev_state = bitarr![u32, Lsb0; 0; 49];
    loop {
        for (n_row, row) in enumerate(&mut rows) {
            row.set_low().unwrap();
            for (n_col, col) in enumerate(&cols) {
                let mut nth_key = n_row * 8 + n_col;

                // skip first 4 keys that don't exist
                if nth_key < 4 {
                    continue;
                }
                nth_key -= 4;

                let current_state = col.is_low().unwrap();
                if prev_state[nth_key] != current_state {
                    info!("Key #{} set to {}", nth_key, current_state);
                    prev_state.set(nth_key, current_state);
                }
            }
            row.set_high().unwrap();
            delay.delay_us(1);
        }
    }
}

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let mut sio = Sio::new(pac.SIO);

    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut _delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    // let pins = bsp::Pins::new(
    //     pac.IO_BANK0,
    //     pac.PADS_BANK0,
    //     sio.gpio_bank0,
    //     &mut pac.RESETS,
    // );

    // Setup second core
    let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio);
    let cores = mc.cores();
    let core1 = &mut cores[1];
    let _test = core1.spawn(core1_task, unsafe { &mut CORE1_STACK.mem });
    let sys_freq = clocks.system_clock.freq().integer();
    sio.fifo.write(sys_freq);

    loop {}
}
