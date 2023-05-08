use cortex_m::peripheral::NVIC;

use stm32l4xx_hal::{
    adc::{SampleTime, Sequence, ADC},
    delay::Delay,
    device::{Interrupt, ADC1, DMA1, EXTI, GPIOA, RNG, SPI1, TIM1, TIM15, TIM2, TIM6, USART1},
    gpio::{Alternate, Edge, Floating, Input, Output, Pin, PullUp, PushPull, H8, L8},
    hal::spi::{Mode, Phase, Polarity},
    pac::{interrupt, ADC2},
    prelude::*,
    rcc::{ClockSecuritySystem, CrystalBypass},
    serial,
    spi::Spi,
    timer::Timer,
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
            .lse(CrystalBypass::Disable, ClockSecuritySystem::Disable) // LSE auto trims the HSI
            .hsi48(true) // needed for RNG
            .sysclk(SYST_CLK_FREQ_MHZ.MHz())
            .pclk1(SYST_CLK_FREQ_MHZ.MHz())
            .pclk2(SYST_CLK_FREQ_MHZ.MHz())
            .freeze(&mut flash.acr, &mut pwr);

        let dma_channels = dp.DMA1.split(&mut rcc.ahb1);

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
        // TIMx periodic timers
        //
        ////////////////////////////////////////////////////////////////////////
        let _tim2 = Timer::tim2(dp.TIM2, TIM2_FREQ_HZ.Hz(), clocks, &mut rcc.apb1r1);

        let _tim6 = Timer::tim6(dp.TIM6, TIM6_FREQ_HZ.Hz(), clocks, &mut rcc.apb1r1);

        let _tim15 = Timer::tim15(dp.TIM15, TIM15_FREQ_HZ.Hz(), clocks, &mut rcc.apb2);

        ////////////////////////////////////////////////////////////////////////
        //
        // ADC1 with DMA to read the sample & hold automatically
        //
        ////////////////////////////////////////////////////////////////////////

        // configure DMA1 to transfer ADC readings to the buffer
        let mut dma1_ch1 = dma_channels.1;
        unsafe {
            dma1_ch1.set_peripheral_address(&dp.ADC1.dr as *const _ as u32, false);
            dma1_ch1.set_memory_address(ADC_DMA_BUFF.as_ptr() as u32, true);
        }
        dma1_ch1.set_transfer_length(NUM_ADC_DMA_SIGNALS as u16);
        unsafe {
            (*DMA1::ptr()).ccr1.modify(|_, w| {
                w.msize()
                    .bits16()
                    .psize()
                    .bits16()
                    .minc()
                    .enabled()
                    .circ()
                    .enabled()
                    .en()
                    .set_bit()
            });
        }

        let mut adc1 = ADC::new(
            dp.ADC1,
            dp.ADC_COMMON,
            &mut rcc.ahb2,
            &mut rcc.ccipr,
            &mut delay,
        );

        let mut s_and_h_pin = gpioa.pa0.into_analog(&mut gpioa.moder, &mut gpioa.pupdr);
        adc1.configure_sequence(&mut s_and_h_pin, Sequence::One, SampleTime::Cycles247_5);

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
            // enable continuous DMA mode
            (*ADC1::ptr())
                .cfgr
                .modify(|_, w| w.dmacfg().set_bit().dmaen().set_bit().cont().set_bit());
        }

        dma1_ch1.start();
        adc1.start_conversion();

        ////////////////////////////////////////////////////////////////////////
        //
        // ADC2 to read the front panel potentiometers manualy via MUX
        //
        ////////////////////////////////////////////////////////////////////////

        // At the time of writing this the stm32l4xx_hal crate has not implemented ADC2, so it is done manually for now

        unsafe {
            // turn on and calibrate ADC2
            (*ADC2::ptr()).cr.modify(|_, w| w.deeppwd().clear_bit());
            (*ADC2::ptr()).cr.modify(|_, w| w.advregen().set_bit());
            delay.delay_us(25_u32);
            (*ADC2::ptr()).cr.modify(|_, w| {
                w.adcal().set_bit();
                w.adcaldif().clear_bit()
            });
            while (*ADC2::ptr()).cr.read().adcal().bit_is_set() {}
            delay.delay_us(1_u32);

            // configure hardware oversampler for 16 bit resolution
            (*ADC2::ptr()).cfgr2.modify(|_, w| {
                w.ovss()
                    .bits(0b0001) // shift right by 1
                    .ovsr()
                    .bits(0b100) // oversample 32x
                    .rovse()
                    .set_bit()
            });

            // set sample time
            (*ADC2::ptr()).smpr1.modify(|_, w| w.smp7().bits(0b100));

            // sequence length of 1
            (*ADC2::ptr()).sqr1.modify(|_, w| w.l().bits(0b0000));
            // single sequence is channel 7 on PA2
            (*ADC2::ptr()).sqr1.modify(|_, w| w.sq1().bits(7));
            // enable ADC2
            (*ADC2::ptr()).cr.modify(|_, w| w.aden().set_bit());

            // we'll read ADC2 in the MUX related functions
        }

        ////////////////////////////////////////////////////////////////////////
        //
        // MUX
        //
        ////////////////////////////////////////////////////////////////////////

        let mux = Mux {
            _sel_n: (
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
            com_discrete: gpioa
                .pa1
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

        // set the DAC128S085 to WTM mode, so that outputs update after each register write
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
            NVIC::unmask(Interrupt::EXTI15_10); // MODOSC TOGGLE switch on PA11
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
        self.mux.select_channel(signal as u32);
        self.mux.read_com_analog()
    }

    /// `board.sample_and_hold()` is the current value of the onboard sample & hold, in `[0.0, 1.0]`
    pub fn sample_and_hold(&mut self) -> f32 {
        unsafe { adc_fs_to_normalized_fl(ADC_DMA_BUFF[0]) }
    }

    /// `board.read_switch_3_way(s)` is the current state of the enumerated 3-way switch `s`
    pub fn read_switch_3_way(&mut self, switch: Switch3way) -> Switch3wayState {
        // each 3-way switch has an upper and lower pin which are read through the discrete MUX with input pullup,
        // "upper" and "lower" here mean that the upper pin is physically farther north when looking at the pcb
        let (upper_mux_ch, lower_mux_ch) = match switch {
            // mux channels are based on the physical PCB layout
            Switch3way::SAndHTrigSrc => (DiscreteMuxChannel::I1, DiscreteMuxChannel::I0),
            Switch3way::VcfEnvTrigSrc => (DiscreteMuxChannel::I3, DiscreteMuxChannel::I2),
            Switch3way::ModEnvTrigSrc => (DiscreteMuxChannel::I5, DiscreteMuxChannel::I4),
            Switch3way::VcaEnvTrigSrc => (DiscreteMuxChannel::I8, DiscreteMuxChannel::I9),
            Switch3way::AutoGateSrc => (DiscreteMuxChannel::I12, DiscreteMuxChannel::I13),
            Switch3way::AutoGateLogic => (DiscreteMuxChannel::I10, DiscreteMuxChannel::I11),
            Switch3way::VcaCtlSrc => (DiscreteMuxChannel::I7, DiscreteMuxChannel::I6),
        };

        // if we change channels and try to read too soon we'll get bogus values
        let mux_settle_time_usec = 1_u32;

        self.mux.select_channel(upper_mux_ch as u32);
        self.delay.delay_us(mux_settle_time_usec); // allow the value to settle
        let upper = self.mux.read_com_discrete();

        self.mux.select_channel(lower_mux_ch as u32);
        self.delay.delay_us(mux_settle_time_usec);
        let lower = self.mux.read_com_discrete();

        match (upper, lower) {
            (false, true) => Switch3wayState::Up,
            (true, true) => Switch3wayState::Middle,
            _ => Switch3wayState::Down, // should only happen with (true, false) but catch unlikely (true, true) as well
                                        // (true, true) means something is wrong with the switch, but the show must go on
        }
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

        // chip select pin is based on the PCB routing
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

        // chip select pin is based on the PCB routing
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
            // chip select pin is based on the PCB routing
            self.spi.write(ChipSelect::Cs2, &[self.led_state])
        }
    }

    /// `board.ext_gate()` is the current state of the External Gate Input ping
    pub fn ext_gate(&mut self) -> bool {
        // the external gate is inverted in hardware, so flip it again to get back to right-way-round
        self.ext_gate_pin.is_low()
    }

    /// `board.modosc_sqr()` is the current state of the ModOsc Square Gate Input pin
    pub fn modosc_sqr(&mut self) -> bool {
        self.modosc_sqr_pin.is_high()
    }

    /// `board.pwm_lfo_sqr()` is the current state of the PWM LFO Gate Input pin
    pub fn pwm_lfo_sqr(&mut self) -> bool {
        self.pwm_lfo_sqr_pin.is_high()
    }

    /// `board.modosc_toggle_switch()` is the current state of the MODOSC toggle switch
    ///
    /// Clicking the physical PCB mounted switch toggles the state between true and false
    pub fn modosc_toggle_switch(&mut self) -> bool {
        unsafe { core::ptr::read_volatile(&MODOSC_TOGGLE_SWITCH_STATE) }
    }

    /// `board.delay_ms(ms)` causes the board to busy-wait for `ms` milliseconds
    pub fn _delay_ms(&mut self, ms: u32) {
        self.delay.delay_ms(ms);
    }

    /// board.tim2_timeout()` is true iff timer TIM2 has timed out, self clearing.
    pub fn tim2_timeout(&self) -> bool {
        unsafe {
            if (*TIM2::ptr()).sr.read().uif().bit() {
                (*TIM2::ptr()).sr.modify(|_, w| w.uif().clear());
                true
            } else {
                false
            }
        }
    }

    /// board.tim6_timeout()` is true iff timer TIM6 has timed out, self clearing.
    pub fn _tim6_timeout(&self) -> bool {
        unsafe {
            if (*TIM6::ptr()).sr.read().uif().bit() {
                (*TIM6::ptr()).sr.modify(|_, w| w.uif().clear());
                true
            } else {
                false
            }
        }
    }

    /// board.tim15_timeout()` is true iff timer TIM15 has timed out, self clearing.
    pub fn _tim15_timeout(&self) -> bool {
        unsafe {
            if (*TIM15::ptr()).sr.read().uif().bit() {
                (*TIM15::ptr()).sr.modify(|_, w| w.uif().clear());
                true
            } else {
                false
            }
        }
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

/// The frequency for periodic timer TIM2
pub const TIM2_FREQ_HZ: u32 = 1_000;

/// The frequency for periodic timer TIM6
pub const TIM6_FREQ_HZ: u32 = 1_001;

/// The frequency for periodic timer TIM15
pub const TIM15_FREQ_HZ: u32 = 33;

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
    // I3, I4, I5 are not wired
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

/// Enumerated multiplexed discrete channels are represented here. Not all channels are wired up on the physical PCB
#[derive(Clone, Copy)]
pub enum DiscreteMuxChannel {
    I0 = 0,
    I1 = 1,
    I2 = 2,
    I3 = 3,
    I4 = 4,
    I5 = 5,
    I6 = 6,
    I7 = 7,
    I8 = 8,
    I9 = 9,
    I10 = 10,
    I11 = 11,
    I12 = 12,
    I13 = 13,
    // I14, I15 are not wired
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

/// ADC readings are stored in a static array via DMA
const NUM_ADC_DMA_SIGNALS: usize = 1;
static mut ADC_DMA_BUFF: [u16; NUM_ADC_DMA_SIGNALS] = [0; NUM_ADC_DMA_SIGNALS];

////////////////////////////////////////////////////////////////////////////////
//
// Private Structs and enums
//
////////////////////////////////////////////////////////////////////////////////

/// The onboard MUX made up of two 74HC4067PW chips is represented here
///
/// One chip handles the analog potentiometers and one handles the discrete switches
#[allow(clippy::type_complexity)]
struct Mux {
    _sel_n: (
        Pin<Output<PushPull>, L8, 'A', 3>,
        Pin<Output<PushPull>, L8, 'A', 4>,
        Pin<Output<PushPull>, L8, 'A', 5>,
        Pin<Output<PushPull>, L8, 'A', 6>,
    ),
    com_discrete: Pin<Input<PullUp>, L8, 'A', 1>,
}

impl Mux {
    /// `m.select_channel(c)` writes the correct states to S0..S3 to select channel `c` in `[0..15]`
    fn select_channel(&mut self, channel: u32) {
        // write the combo of s0..s3 to select the channel
        let channel = channel.min(0xF);

        unsafe {
            // this only works because the MUX select pins are all adjacent and in-order: pins A3, A4, A5, and A6

            // reset the MUX select pins, starts at BR3
            (*GPIOA::ptr()).bsrr.write(|w| w.bits(0xF << 19));
            // set the MUX select pins to the channel, starts at BS3
            (*GPIOA::ptr()).bsrr.write(|w| w.bits(channel << 3));
        }
    }

    /// `m.read_com_discrete()` is the state of the discrete mux common pin
    fn read_com_discrete(&mut self) -> bool {
        self.com_discrete.is_high()
    }

    /// `m.read_com_analog()` is the state of the analog mux common pin in `[0.0, 1.0]`
    fn read_com_analog(&mut self) -> f32 {
        unsafe {
            // start the conversion
            (*ADC2::ptr()).cr.modify(|_, w| w.adstart().set_bit());
            // wait to complete
            while (*ADC2::ptr()).cr.read().adstart().bit_is_set() {}
            // read the data register and convert to float in [0.0, 1.0]
            let d = (*ADC2::ptr()).dr.read().bits() as u16;
            adc_fs_to_normalized_fl(d)
        }
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
