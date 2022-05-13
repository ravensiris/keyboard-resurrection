#![no_std]
#![no_main]

use core::convert::TryFrom;

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
    usb::UsbBus,
    watchdog::Watchdog,
};
use usb_device::{
    class_prelude::UsbBusAllocator,
    device::{UsbDeviceBuilder, UsbVidPid},
};
use usbd_midi::{
    data::{
        byte::u7::U7,
        midi::{channel::Channel, message::Message, notes::Note},
        usb::constants::USB_CLASS_NONE,
        usb_midi::{cable_number::CableNumber, usb_midi_event_packet::UsbMidiEventPacket},
    },
    midi_device::MidiClass,
};

use bitvec::prelude::*;
use heapless::spsc::Queue;
use itertools::enumerate;

enum SimpleMessage {
    NoteOn(Note),
    NoteOff(Note),
}

impl SimpleMessage {
    fn new(is_on: bool, note: Note) -> SimpleMessage {
        if is_on {
            SimpleMessage::NoteOn(note)
        } else {
            SimpleMessage::NoteOff(note)
        }
    }
}

impl From<SimpleMessage> for Message {
    fn from(msg: SimpleMessage) -> Self {
        match msg {
            SimpleMessage::NoteOn(note) => Message::NoteOn(Channel::Channel1, note, U7::MAX),
            SimpleMessage::NoteOff(note) => Message::NoteOff(Channel::Channel1, note, U7::MAX),
        }
    }
}

impl From<SimpleMessage> for u8 {
    fn from(msg: SimpleMessage) -> Self {
        let (is_on_bit, note) = match msg {
            SimpleMessage::NoteOff(note) => (0, note),
            SimpleMessage::NoteOn(note) => (1 << 7, note),
        };
        let note_u8: u8 = note.into();
        is_on_bit + note_u8
    }
}

impl From<u8> for SimpleMessage {
    fn from(msg: u8) -> Self {
        let is_on = msg & (1 << 7) != 0;
        let note: Note = Note::try_from(msg & !(1 << 7)).unwrap();
        SimpleMessage::new(is_on, note)
    }
}

const NOTE_MAP: [Note; 49] = [
    Note::C0,
    Note::Cs0,
    Note::D0,
    Note::Ds0,
    Note::E0,
    Note::F0,
    Note::Fs0,
    Note::G0,
    Note::Gs0,
    Note::A0,
    Note::As0,
    Note::B0,
    //
    Note::C1,
    Note::Cs1,
    Note::D1,
    Note::Ds1,
    Note::E1,
    Note::F1,
    Note::Fs1,
    Note::G1,
    Note::Gs1,
    Note::A1,
    Note::As1,
    Note::B1,
    //
    Note::C2,
    Note::Cs2,
    Note::D2,
    Note::Ds2,
    Note::E2,
    Note::F2,
    Note::Fs2,
    Note::G2,
    Note::Gs2,
    Note::A2,
    Note::As2,
    Note::B2,
    //
    Note::C3,
    Note::Cs3,
    Note::D3,
    Note::Ds3,
    Note::E3,
    Note::F3,
    Note::Fs3,
    Note::G3,
    Note::Gs3,
    Note::A3,
    Note::As3,
    Note::B3,
    //
    Note::C4,
];

static mut MESSAGE_QUEUE: Queue<u8, 128> = Queue::new();

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
    let (mut prod, _) = unsafe { MESSAGE_QUEUE.split() };
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
                    let msg = SimpleMessage::new(current_state, NOTE_MAP[nth_key]);
                    prev_state.set(nth_key, current_state);
                    prod.enqueue(msg.into()).unwrap();
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

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

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

    // Setup USB
    let usb_bus = UsbBusAllocator::new(UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));
    let mut midi = MidiClass::new(&usb_bus, 1, 0).unwrap();
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x5e4))
        .product("MIDI Test")
        .device_class(USB_CLASS_NONE)
        .build();

    let (_, mut cons) = unsafe { MESSAGE_QUEUE.split() };
    loop {
        // HACK: No checking for polls
        // REASON: It ain't working chief. Polls forever.
        usb_dev.poll(&mut [&mut midi]);
        if let Some(msg_u8) = cons.peek() {
            let msg: SimpleMessage = (*msg_u8).into();
            if midi
                .send_message(UsbMidiEventPacket::from_midi(
                    CableNumber::Cable0,
                    msg.into(),
                ))
                .is_ok()
            {
                info!("Sent message {}", msg_u8);
                cons.dequeue();
            } else {
                error!("Failed to send message {}", msg_u8);
            }
        }
    }
}
