use cortex_m::peripheral::NVIC;

use stm32l4xx_hal::{
    adc::{SampleTime, ADC},
    delay::Delay,
    device::{Interrupt, ADC1, EXTI, RNG, SPI1, TIM1, USART1},
    gpio::{Alternate, Analog, Edge, Floating, Input, Output, Pin, PullUp, PushPull, H8, L8},
    hal::spi::{Mode, Phase, Polarity},
    pac::interrupt,
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

    // ADC for reading the sample & hold and analog MUX
    adc1: ADC,
    s_and_h_pin: Pin<Analog, L8, 'A', 0>,

    // general purpose delay
    delay: Delay,

    // binary representation of the LEDs driven by 74HC595 shift register
    led_state: u8,

    // state of the various gate/trigger inputs
    ext_gate_pin: Pin<Input<Floating>, L8, 'A', 7>,
    pwm_lfo_sqr_pin: Pin<Input<Floating>, L8, 'B', 0>,
    modosc_sqr_pin: Pin<Input<Floating>, L8, 'B', 1>,

    // debug pin for misc debug purposes
    debug_pin: Pin<Output<PushPull>, H8, 'A', 12>,
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
        let mut dp = stm32l4xx_hal::pac::Peripherals::take().unwrap();
        let mut flash = dp.FLASH.constrain();
        let mut rcc = dp.RCC.constrain();
        let mut pwr = dp.PWR.constrain(&mut rcc.apb1r1);

        let clocks = rcc
            .cfgr
            .hsi48(true) // needed for RNG
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
        // PWM White Noise Generator
        //
        ////////////////////////////////////////////////////////////////////////

        // RNG and PWM are set up to continuously generate a noise signal on pin PA8
        // this signal is updated in the RNG interrupt
        let _rng = dp.RNG.enable(&mut rcc.ahb2, clocks);
        unsafe {
            // enable RNG interrupts, each time a new random number is ready the interrupt fires
            (*RNG::ptr()).cr.modify(|_, w| w.ie().set_bit());
            NVIC::unmask(Interrupt::RNG);
        }

        // configure TIM1 for PWM generation, the initial freq is just a placeholder, the actual frequency will depend
        // on the WHITE_NOISE_PWM_MAX_COUNT, and will be around SYST_CLK_FREQ_MHZ / WHITE_NOISE_PWM_MAX_COUNT
        let pwm_pin =
            gpioa
                .pa8
                .into_alternate(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);
        let mut pwm = dp.TIM1.pwm(pwm_pin, 1.kHz(), clocks, &mut rcc.apb2);
        unsafe {
            (*TIM1::ptr()).psc.write(|w| w.bits(0));
            (*TIM1::ptr())
                .arr
                .write(|w| w.bits(WHITE_NOISE_PWM_MAX_COUNT));
            (*TIM1::ptr())
                .ccr1
                .write(|w| w.ccr().bits(WHITE_NOISE_PWM_MAX_COUNT as u16 / 2));
        }
        pwm.enable();

        ////////////////////////////////////////////////////////////////////////
        //
        // GPIO interrupt pins
        //
        ////////////////////////////////////////////////////////////////////////

        let mut modosc_on_off_toggle = gpioa
            .pa11
            .into_floating_input(&mut gpioa.moder, &mut gpioa.pupdr);
        modosc_on_off_toggle.make_interrupt_source(&mut dp.SYSCFG, &mut rcc.apb2);
        modosc_on_off_toggle.enable_interrupt(&mut dp.EXTI);
        // rising edges only, when the user taps the switch it toggles the MODOSC on/off
        modosc_on_off_toggle.trigger_on_edge(&mut dp.EXTI, Edge::Rising);

        unsafe {
            NVIC::unmask(Interrupt::EXTI15_10); // MODOSC TOGGLE on PA11
        }

        ////////////////////////////////////////////////////////////////////////
        //
        // Create self
        //
        ////////////////////////////////////////////////////////////////////////

        Self {
            _midi_tx,
            midi_rx,
            mux,
            spi,
            adc1,
            s_and_h_pin,
            delay,
            led_state: 0,

            ext_gate_pin: gpioa
                .pa7
                .into_floating_input(&mut gpioa.moder, &mut gpioa.pupdr),
            pwm_lfo_sqr_pin: gpiob
                .pb0
                .into_floating_input(&mut gpiob.moder, &mut gpiob.pupdr),
            modosc_sqr_pin: gpiob
                .pb1
                .into_floating_input(&mut gpiob.moder, &mut gpiob.pupdr),

            // last_ext_gate_state: false,
            // last_pwm_lfo_sqr_state: false,
            // last_modosc_sqr_state: false,
            debug_pin: gpioa.pa12.into_push_pull_output_in_state(
                &mut gpioa.moder,
                &mut gpioa.otyper,
                PinState::Low,
            ),
        }
    }

    /// `board.read_midi()` is the optional byte read from the MIDI UART
    ///
    /// If MIDI input is expected this function must be called more frequently than the incoming MIDI bytes
    pub fn read_midi(&mut self) -> Option<u8> {
        match self.midi_rx.read() {
            Ok(byte) => Some(byte),
            _ => None,
        }
    }

    /// `board.read_analog_signal(s)` is the current value of analog signal `s` in `[0.0, 1.0]`
    pub fn read_analog_signal(&mut self, signal: AnalogMuxChannel) -> f32 {
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

        switch_3_way_state_from_upper_and_lower(
            self.mux.read_discrete(upper_mux_ch),
            self.mux.read_discrete(lower_mux_ch),
        )
    }

    /// `board.dac8162_set_vout(v, c)` writes voltage `v` to channel `c` of the onboard DAC.
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

    /// `board.dac128S085_set_vout(v, c)` writes voltage `v` to channel `c` of the onboard DAC.
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
        let last_led_state = self.led_state;
        if state {
            self.led_state |= led as u8;
        } else {
            self.led_state &= !(led as u8);
        }
        if last_led_state != self.led_state {
            self.spi.write(ChipSelect::Cs2, &[self.led_state])
        }
    }

    /// `board.ext_gate_state()` is the current state of the External Gate Input ping
    pub fn ext_gate_state(&mut self) -> bool {
        // the external gate is inverted in hardware, so flip it again to get back to right-way-round
        self.ext_gate_pin.is_low()
    }

    /// `board.modosc_sqr_state()` is the current state of the ModOsc Square Gate Input pin
    pub fn modosc_sqr_state(&mut self) -> bool {
        self.modosc_sqr_pin.is_high()
    }

    /// `board.pwm_lfo_sqr_state()` is the current state of the PWM LFO Gate Input pin
    pub fn pwm_lfo_sqr_state(&mut self) -> bool {
        self.pwm_lfo_sqr_pin.is_high()
    }

    /// `board.modosc_toggle_switch_state()` is the current state of the MODOSC toggle switch
    ///
    /// Clicking the physical PCB mounted switch toggles the state between true and false
    pub fn modosc_toggle_switch_state(&mut self) -> bool {
        unsafe { core::ptr::read_volatile(&MODOSC_TOGGLE_SWITCH_STATE) }
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

/// Enumerated multiplexed analog channels are represented here. Not all channels are wired up on the physical PCB
#[derive(Clone, Copy)]
pub enum AnalogMuxChannel {
    I0 = 0,
    I1 = 1,
    I2 = 2,
    // I3..I5 are not wired
    I6 = 6,
    I7 = 7,
    I8 = 8,
    I9 = 9,
    I10 = 10,
    I11 = 11,
    I12 = 12,
    I13 = 13,
    I14 = 14,
    I15 = 15,
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
}

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

/// The maximum value that can be written via PWM as white noise
///
/// This number impacts both the bit-depth of the white noise signal and also the PWM frequency used to generate the
/// white noise output.
const WHITE_NOISE_PWM_MAX_COUNT: u32 = (1 << 10) - 1;

////////////////////////////////////////////////////////////////////////////////
//
// Private Static Variables
//
////////////////////////////////////////////////////////////////////////////////

/// the MODOSC is toggled on and off in an interrupt routine
static mut MODOSC_TOGGLE_SWITCH_STATE: bool = false;

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

////////////////////////////////////////////////////////////////////////////////
//
// Private helper functions and interrupt routines
//
////////////////////////////////////////////////////////////////////////////////

/// interrupt routine to generate white noise via PWM
#[interrupt]
fn RNG() {
    unsafe {
        let rand = ((*RNG::ptr()).dr.read().rndata().bits() % WHITE_NOISE_PWM_MAX_COUNT) as u16;
        (*TIM1::ptr()).ccr1.write(|w| w.ccr().bits(rand))
    };
}

#[interrupt]
fn EXTI15_10() {
    unsafe {
        // clear the pending interrupt
        (*EXTI::ptr()).pr1.write(|w| w.bits(1 << 11));

        // toggle the static signal, this indicates that someone tapped the MODOSC toggle switch
        let toggled_state = !core::ptr::read_volatile(&MODOSC_TOGGLE_SWITCH_STATE);
        core::ptr::write_volatile(&mut MODOSC_TOGGLE_SWITCH_STATE, toggled_state);
    }
}

/// `adc_fs_to_normalized_fl(v)` is the integer adc value normalized to [0.0, +1.0]
///
/// If the input value would overflow the output range it is clamped.
fn adc_fs_to_normalized_fl(val: u16) -> f32 {
    let val = val.min(ADC_MAX);
    (val as f32) / (ADC_MAX as f32)
}

/// `switch_3_way_state_from_upper_and_lower(u, l)` is the 3-way switch state matching the pin states `u` and `l`
///
/// Each 3-way switch has an upper and lower pin, the state is governed by the combination of pin states
fn switch_3_way_state_from_upper_and_lower(
    upper_pin_state: bool,
    lower_pin_state: bool,
) -> Switch3wayState {
    match (upper_pin_state, lower_pin_state) {
        (false, true) => Switch3wayState::Up,
        (true, true) => Switch3wayState::Middle,
        _ => Switch3wayState::Down,
    }
}
