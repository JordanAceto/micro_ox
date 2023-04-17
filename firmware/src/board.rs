use stm32l4xx_hal::{
    adc::{SampleTime, ADC},
    delay::Delay,
    device::{ADC1, SPI1, USART1},
    gpio::{Alternate, Analog, Floating, Input, Output, Pin, PullUp, PushPull, H8, L8},
    hal::spi::{Mode, Phase, Polarity},
    prelude::*,
    serial,
    spi::Spi,
};

/// The physical board structure is represented here
pub struct Board {
    // USART for MIDI
    _midi_tx: serial::Tx<USART1>,
    midi_rx: serial::Rx<USART1>,

    // onboard multiplexer for reading analog and discrete signals
    mux: Mux,

    // SPI for DACs and LED driver
    spi: SpiBus,

    adc1: ADC,
    s_and_h_pin: Pin<Analog, L8, 'A', 0>,

    // gate input pins
    not_ext_gate: Pin<Input<Floating>, H8, 'A', 8>,
    modosc_sqr: Pin<Input<Floating>, H8, 'A', 11>,
    pwm_lfo_sqr: Pin<Input<Floating>, H8, 'A', 12>,

    // general purpose delay
    delay: Delay,

    // binary representation of the LEDs driven by 74HC595 shift register
    led_state: u8,

    // debug pin for misc debug purposes
    debug_pin: Pin<Output<PushPull>, L8, 'B', 0>,
}

impl Board {
    /// `Board::init()` is the board structure with all peripherals initialized.
    pub fn init() -> Self {
        ////////////////////////////////////////////////////////////////////////
        //
        // general peripheral housekeeping, core peripherals and clocks
        //
        ////////////////////////////////////////////////////////////////////////
        let cp = cortex_m::Peripherals::take().unwrap();
        let dp = stm32l4xx_hal::pac::Peripherals::take().unwrap();
        let mut flash = dp.FLASH.constrain();
        let mut rcc = dp.RCC.constrain();
        let mut pwr = dp.PWR.constrain(&mut rcc.apb1r1);

        let clocks = rcc
            .cfgr
            .sysclk(SYST_CLK_FREQ_MHZ.MHz())
            .pclk1(SYST_CLK_FREQ_MHZ.MHz())
            .pclk2(SYST_CLK_FREQ_MHZ.MHz())
            .freeze(&mut flash.acr, &mut pwr);

        let mut delay = Delay::new(cp.SYST, clocks);

        let mut gpioa = dp.GPIOA.split(&mut rcc.ahb2);
        let mut gpiob = dp.GPIOB.split(&mut rcc.ahb2);

        ////////////////////////////////////////////////////////////////////////
        //
        // USART
        //
        ////////////////////////////////////////////////////////////////////////

        let tx_pin = gpioa
            .pa9
            .into_alternate(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);
        let rx_pin =
            gpioa
                .pa10
                .into_alternate(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);

        let usart = serial::Serial::usart1(
            dp.USART1,
            (tx_pin, rx_pin),
            serial::Config::default().baudrate(MIDI_BAUD_RATE_HZ.bps()),
            clocks,
            &mut rcc.apb2,
        );
        let (_midi_tx, midi_rx) = usart.split();

        ////////////////////////////////////////////////////////////////////////
        //
        // ADC
        //
        ////////////////////////////////////////////////////////////////////////

        let mut adc1 = ADC::new(
            dp.ADC1,
            dp.ADC_COMMON,
            &mut rcc.ahb2,
            &mut rcc.ccipr,
            &mut delay,
        );

        adc1.set_sample_time(SampleTime::Cycles640_5);

        unsafe {
            // configure hardware oversampler for 16 bit resolution
            (*ADC1::ptr()).cfgr2.modify(|_, w| {
                w.ovss()
                    .bits(0b0001) // shift right by 1
                    .ovsr()
                    .bits(0b100) // oversample 32x
                    .rovse()
                    .set_bit()
            });
        }

        let s_and_h_pin = gpioa.pa0.into_analog(&mut gpioa.moder, &mut gpioa.pupdr);

        ////////////////////////////////////////////////////////////////////////
        //
        // SPI
        //
        ////////////////////////////////////////////////////////////////////////

        let sck = gpiob
            .pb3
            .into_alternate(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);
        let sdi = gpiob
            .pb4
            .into_alternate(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);
        let sdo = gpiob
            .pb5
            .into_alternate(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);

        let mut spi = SpiBus {
            bus: Spi::spi1(
                dp.SPI1,
                (sck, sdi, sdo),
                Mode {
                    phase: Phase::CaptureOnFirstTransition,
                    polarity: Polarity::IdleHigh,
                },
                SPI_CLK_FREQ_MHZ.MHz(),
                clocks,
                &mut rcc.apb2,
            ),
            chip_sel: (
                gpioa.pa15.into_push_pull_output_in_state(
                    &mut gpioa.moder,
                    &mut gpioa.otyper,
                    PinState::High,
                ),
                gpiob.pb6.into_push_pull_output_in_state(
                    &mut gpiob.moder,
                    &mut gpiob.otyper,
                    PinState::High,
                ),
                gpiob.pb7.into_push_pull_output_in_state(
                    &mut gpiob.moder,
                    &mut gpiob.otyper,
                    PinState::High,
                ),
            ),
        };

        // set the DAC128S085 to WTM mode, so that outputs update after writing to a register
        spi.write(ChipSelect::Cs1, &[0b1001_0000, 0]);

        ////////////////////////////////////////////////////////////////////////
        //
        // MUX
        //
        ////////////////////////////////////////////////////////////////////////

        let mux = Mux {
            sel_n: (
                gpioa
                    .pa3
                    .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper),
                gpioa
                    .pa4
                    .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper),
                gpioa
                    .pa5
                    .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper),
                gpioa
                    .pa6
                    .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper),
            ),
            com_analog: gpioa.pa1.into_analog(&mut gpioa.moder, &mut gpioa.pupdr),
            com_discrete: gpioa
                .pa2
                .into_pull_up_input(&mut gpioa.moder, &mut gpioa.pupdr),
        };

        Self {
            _midi_tx,
            midi_rx,
            mux,
            spi,
            adc1,
            s_and_h_pin,
            delay,
            led_state: 0,
            not_ext_gate: gpioa
                .pa8
                .into_floating_input(&mut gpioa.moder, &mut gpioa.pupdr),
            modosc_sqr: gpioa
                .pa11
                .into_floating_input(&mut gpioa.moder, &mut gpioa.pupdr),
            pwm_lfo_sqr: gpioa
                .pa12
                .into_floating_input(&mut gpioa.moder, &mut gpioa.pupdr),
            debug_pin: gpiob.pb0.into_push_pull_output_in_state(
                &mut gpiob.moder,
                &mut gpiob.otyper,
                PinState::Low,
            ),
        }
    }

    /// `board.midi_read()` is the optional byte read from the MIDI UART
    pub fn midi_read(&mut self) -> Option<u8> {
        match self.midi_rx.read() {
            Ok(byte) => Some(byte),
            _ => None,
        }
    }

    /// `board.read_analog_signal(s)` is the current value of analog signal `s` in `[0.0, 1.0]`
    pub fn read_analog_signal(&mut self, signal: AnalogMuxSignal) -> f32 {
        self.mux.read_analog(&mut self.adc1, signal as u8)
    }

    /// `board.sample_and_hold()` is the current value of the onboard sample & hold, in `[0.0, 1.0]`
    pub fn sample_and_hold(&mut self) -> f32 {
        adc_fs_to_normalized_fl(self.adc1.read(&mut self.s_and_h_pin).unwrap())
    }

    /// `board.read_switch_3_way(s)` is the current state of the enumerated 3-way switch `s`
    pub fn read_switch_3_way(&mut self, switch: Switch3way) -> Switch3wayState {
        // each 3-way switch has an upper and lower pin which are read through the discrete MUX
        let upper_mux_ch;
        let lower_mux_ch;
        match switch {
            // pins are based on the physical PCB layout
            Switch3way::SAndHTrigSrc => {
                upper_mux_ch = 1;
                lower_mux_ch = 0;
            }
            Switch3way::VcfEnvTrigSrc => {
                upper_mux_ch = 3;
                lower_mux_ch = 2;
            }
            Switch3way::ModEnvTrigSrc => {
                upper_mux_ch = 5;
                lower_mux_ch = 4;
            }
            Switch3way::VcaEnvTrigSrc => {
                upper_mux_ch = 8;
                lower_mux_ch = 9;
            }
            Switch3way::AutoGateSrc => {
                upper_mux_ch = 12;
                lower_mux_ch = 13;
            }
            Switch3way::AutoGateLogic => {
                upper_mux_ch = 10;
                lower_mux_ch = 11;
            }
            Switch3way::VcaCtlSrc => {
                upper_mux_ch = 7;
                lower_mux_ch = 6;
            }
        }

        switch_3_way_state(
            self.mux.read_discrete(upper_mux_ch),
            self.mux.read_discrete(lower_mux_ch),
        )
    }

    /// `board.dac8162_set_vout(v, c)` writes the voltage `v` to channel `c` of the onboard DAC.
    ///
    /// # Arguments
    ///
    /// * `v_out` - The analog voltage to write, clamped to `[0.0, DAC8162_MAX_VOLTS]`
    ///
    /// * `channel` - The enumerated DAC channel to write to
    pub fn dac8162_set_vout(&mut self, v_out: f32, channel: Dac8162Channel) {
        let v_out = v_out.max(0.0_f32).min(DAC8162_MAX_VOUT);

        let val_u14 = (v_out * DAC8162_COUNTS_PER_VOLT) as u16;
        // move the value out of DB0 and DB1
        let val_u14 = val_u14 << 2;
        // split it into bytes
        let low_byte = (val_u14 & 0xFF) as u8;
        let mid_byte = (val_u14 >> 8) as u8;
        let high_byte = channel as u8 | 0b0001_1000; // write to channel and update output

        self.spi
            .write(ChipSelect::Cs0, &[high_byte, mid_byte, low_byte]);
    }

    /// `board.dac128S085_set_vout(v, c)` writes the voltage `v` to channel `c` of the onboard DAC.
    ///
    /// # Arguments
    ///
    /// * `v_out` - The analog voltage to write, clamped to `[0.0, DAC128S085_MAX_VOLTS]`
    ///
    /// * `channel` - The enumerated DAC channel to write to
    #[allow(non_snake_case)]
    pub fn dac128S085_set_vout(&mut self, v_out: f32, channel: Dac128S085Channel) {
        let v_out = v_out.max(0.0_f32).min(DAC128S085_MAX_VOUT);

        let val_u12 = (v_out * DAC128S085_COUNTS_PER_VOLT) as u16;

        // split it into bytes
        let low_byte = (val_u12 & 0xFF) as u8;
        let high_byte = (channel as u8) << 4 | (val_u12 >> 8) as u8;

        self.spi.write(ChipSelect::Cs1, &[high_byte, low_byte]);
    }

    /// `board.set_led(l, s)` sets enumerated LED `l` to boolean state `s`
    pub fn set_led(&mut self, led: Led, state: bool) {
        if state {
            self.led_state |= led as u8;
        } else {
            self.led_state &= !(led as u8);
        }
        self.spi.write(ChipSelect::Cs2, &[self.led_state])
    }

    /// `board.ext_gate()` is the current state of the external gate input
    pub fn ext_gate(&mut self) -> bool {
        self.not_ext_gate.is_low()
    }

    /// `board.modosc_sqr()` is the current state of the MODOSC Square wave gate input
    pub fn modosc_sqr(&mut self) -> bool {
        self.modosc_sqr.is_high()
    }

    /// `board.pwm_lfo_sqr()` is the current state of the PWM LFO Square wave input
    pub fn pwm_lfo_sqr(&mut self) -> bool {
        self.pwm_lfo_sqr.is_high()
    }

    /// `board.delay_ms(ms)` causes the board to busy-wait for `ms` milliseconds
    pub fn delay_ms(&mut self, ms: u32) {
        self.delay.delay_ms(ms);
    }

    /// `board.set_debug_pin(s)` writes the value `s` to the debug pin, setting it high or low
    pub fn set_debug_pin(&mut self, state: bool) {
        if state {
            self.debug_pin.set_high()
        } else {
            self.debug_pin.set_low()
        }
    }
}

/// `switch_3_way_state(u, l)` is the 3-way switch state matching the pin states `u` and `l`
///
/// Each 3-way switch has an upper and lower pin, the state is governed by the combination of pin states
fn switch_3_way_state(upper_pin_state: bool, lower_pin_state: bool) -> Switch3wayState {
    match (upper_pin_state, lower_pin_state) {
        (false, true) => Switch3wayState::Up,
        (true, true) => Switch3wayState::Middle,
        _ => Switch3wayState::Down,
    }
}

////////////////////////////////////////////////////////////////////////////////
//
// Public constants
//
////////////////////////////////////////////////////////////////////////////////

/// The frequency of the main system clock
pub const SYST_CLK_FREQ_MHZ: u32 = 80;

/// The maximum value that can be produced by the Analog to Digital Converters.
pub const ADC_MAX: u16 = 0xFFF0;

/// The maximum value that can be written to the onboard Digital to Analog Converter.
pub const DAC8162_MAX_COUNT: u16 = (1 << 14) - 1;
pub const DAC128S085_MAX_COUNT: u16 = (1 << 12) - 1;

/// The maximum analog voltage that the DAC can produce after onboard amplification
pub const DAC8162_MAX_VOUT: f32 = 10.0_f32;
pub const DAC128S085_MAX_VOUT: f32 = 2.5_f32;

////////////////////////////////////////////////////////////////////////////////
//
// Private constants
//
////////////////////////////////////////////////////////////////////////////////

/// The baud rate required for MIDI communication
const MIDI_BAUD_RATE_HZ: u32 = 31_250;

/// The SPI clock frequency to use
const SPI_CLK_FREQ_MHZ: u32 = 10;

/// The number of DAC counts for 1 volt output
const DAC8162_COUNTS_PER_VOLT: f32 = DAC8162_MAX_COUNT as f32 / DAC8162_MAX_VOUT;
const DAC128S085_COUNTS_PER_VOLT: f32 = DAC128S085_MAX_COUNT as f32 / DAC128S085_MAX_VOUT;

////////////////////////////////////////////////////////////////////////////////
//
// Private helper functions
//
////////////////////////////////////////////////////////////////////////////////

/// `adc_fs_to_normalized_fl(v)` is the integer adc value normalized to [0.0, +1.0]
///
/// If the input value would overflow the output range it is clamped.
fn adc_fs_to_normalized_fl(val: u16) -> f32 {
    let val = val.min(ADC_MAX);
    (val as f32) / (ADC_MAX as f32)
}

////////////////////////////////////////////////////////////////////////////////
//
// Public Enums
//
////////////////////////////////////////////////////////////////////////////////

/// Valid states of a 3-way switch are represented here
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Switch3wayState {
    Up,
    Middle,
    Down,
}

/// Enumerated 3-way switches are represented here. The physical board has a number of switches for specific functions.
#[derive(Clone, Copy)]
pub enum Switch3way {
    SAndHTrigSrc,
    VcfEnvTrigSrc,
    ModEnvTrigSrc,
    VcaEnvTrigSrc,
    AutoGateSrc,
    AutoGateLogic,
    VcaCtlSrc,
}

/// Enumerated multiplexed analog signals are represented here.
#[derive(Clone, Copy)]
pub enum AnalogMuxSignal {
    VcfEnvAttack = 9,
    VcfEnvDecay = 8,
    VcfEnvSustain = 7,
    VcfEnvRelease = 6,
    ModEnvAttack = 12,
    ModEnvDecay = 13,
    ModEnvSustain = 14,
    ModEnvRelease = 15,
    VcaEnvAttack = 0,
    VcaEnvRelease = 1,
    ModOscRiseTime = 2,
    SAndHGlide = 10,
    Portamento = 11,
}

/// Channels of the onboard DAC8162 are represented here
#[derive(Clone, Copy)]
pub enum Dac8162Channel {
    A = 0b000,
    B = 0b001,
}

/// Channels of the onboard DAC128S085 are represented here
#[derive(Clone, Copy)]
pub enum Dac128S085Channel {
    A = 0b000,
    B = 0b001,
    C = 0b010,
    D = 0b011,
    E = 0b100,
    F = 0b101,
    G = 0b110,
    H = 0b111,
}

/// Enumerated LEDs are represented here.
#[derive(Clone, Copy)]
pub enum Led {
    // values are based on the physical PCB layout, LEDs are driven by a 75HC595 shift register
    SAndHTrig = (1 << 1),
    VcfEnvTrig = (1 << 2),
    ModEnvTrig = (1 << 3),
    AutoGate = (1 << 4),
    VcaTrig = (1 << 5),
    ModOscOnOff = (1 << 7),
}

////////////////////////////////////////////////////////////////////////////////
//
// Private Structs and enums
//
////////////////////////////////////////////////////////////////////////////////

#[allow(clippy::type_complexity)]
struct Mux {
    sel_n: (
        Pin<Output<PushPull>, L8, 'A', 3>,
        Pin<Output<PushPull>, L8, 'A', 4>,
        Pin<Output<PushPull>, L8, 'A', 5>,
        Pin<Output<PushPull>, L8, 'A', 6>,
    ),
    com_analog: Pin<Analog, L8, 'A', 1>,
    com_discrete: Pin<Input<PullUp>, L8, 'A', 2>,
}

impl Mux {
    fn select_channel(&mut self, channel: u8) {
        // write the combo of s0..s3 to select the channel
        let channel = channel.min(15);
        if channel & 1 == 1 {
            self.sel_n.0.set_high();
        } else {
            self.sel_n.0.set_low();
        }
        if (channel >> 1) & 1 == 1 {
            self.sel_n.1.set_high();
        } else {
            self.sel_n.1.set_low();
        }
        if (channel >> 2) & 1 == 1 {
            self.sel_n.2.set_high();
        } else {
            self.sel_n.2.set_low();
        }
        if (channel >> 3) & 1 == 1 {
            self.sel_n.3.set_high();
        } else {
            self.sel_n.3.set_low();
        }
    }

    fn read_discrete(&mut self, channel: u8) -> bool {
        self.select_channel(channel);
        self.com_discrete.is_high()
    }

    fn read_analog(&mut self, adc: &mut ADC, channel: u8) -> f32 {
        self.select_channel(channel);
        adc_fs_to_normalized_fl(adc.read(&mut self.com_analog).unwrap())
    }
}

#[allow(clippy::type_complexity)]
struct SpiBus {
    bus: Spi<
        SPI1,
        (
            Pin<Alternate<PushPull, 5>, L8, 'B', 3>, // SCK
            Pin<Alternate<PushPull, 5>, L8, 'B', 4>, // SDI (unused)
            Pin<Alternate<PushPull, 5>, L8, 'B', 5>, // SDO
        ),
    >,
    // manual chip select pins
    chip_sel: (
        Pin<Output<PushPull>, H8, 'A', 15>,
        Pin<Output<PushPull>, L8, 'B', 6>,
        Pin<Output<PushPull>, L8, 'B', 7>,
    ),
}

enum ChipSelect {
    Cs0,
    Cs1,
    Cs2,
}

impl SpiBus {
    fn write(&mut self, cs: ChipSelect, words: &[u8]) {
        match cs {
            ChipSelect::Cs0 => self.chip_sel.0.set_low(),
            ChipSelect::Cs1 => self.chip_sel.1.set_low(),
            ChipSelect::Cs2 => self.chip_sel.2.set_low(),
        }
        self.bus.write(words).unwrap();
        match cs {
            ChipSelect::Cs0 => self.chip_sel.0.set_high(),
            ChipSelect::Cs1 => self.chip_sel.1.set_high(),
            ChipSelect::Cs2 => self.chip_sel.2.set_high(),
        }
    }
}
