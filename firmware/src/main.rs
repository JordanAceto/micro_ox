#![no_std]
#![no_main]

mod board;
mod envelopes;
mod gate_routing;
mod ui;
use crate::board::Board;

use panic_halt as _;

use cortex_m_rt::entry;

#[entry]
fn main() -> ! {
    let mut board = Board::init();

    let leds = [
        board::Led::SAndHTrig,
        board::Led::VcfEnvTrig,
        board::Led::ModEnvTrig,
        board::Led::AutoGate,
    ];

    loop {
        // just a demo of some hardware for now to make sure we can flash the chip
        leds.iter().for_each(|l| {
            board.set_led(*l, true);
            board.delay_ms(10);
            board.set_led(*l, false);
            board.delay_ms(10);
        })
    }
}
