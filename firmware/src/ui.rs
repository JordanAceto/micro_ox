use crate::board::{AnalogMuxChannel, Board, Switch3way, Switch3wayState};

/// The user interface is represented here (i.e. the front panel pots and switches that the user interacts with)
pub struct Ui {
    s_and_h_trig_src: TriggerSource,
    vcf_env_trig_src: TriggerSource,
    mod_env_trig_src: TriggerSource,
    vca_env_trig_src: TriggerSource,

    auto_gate_src: AutoGateSource,

    auto_gate_logic_mode: AutoGateLogicMode,

    vca_ctl_mode: VcaCtlMode,

    portamento_time: f32,
    s_and_h_glide_time: f32,
    modosc_rise_time: f32,

    vcf_env_attack: f32,
    vcf_env_decay: f32,
    vcf_env_sustain: f32,
    vcf_env_release: f32,

    mod_env_attack: f32,
    mod_env_decay: f32,
    mod_env_sustain: f32,
    mod_env_release: f32,

    vca_env_attack: f32,
    vca_env_release: f32,

    // counter to decide which signal to update for a given round
    round_robbin_update_counter: u8,
}

impl Ui {
    /// `UiState::new()` is a new UI state initialized to default values.
    pub fn new() -> Self {
        Self {
            s_and_h_trig_src: TriggerSource::Gate,
            vcf_env_trig_src: TriggerSource::Gate,
            mod_env_trig_src: TriggerSource::Gate,
            vca_env_trig_src: TriggerSource::Gate,

            auto_gate_src: AutoGateSource::PwmLfo,

            auto_gate_logic_mode: AutoGateLogicMode::And,

            vca_ctl_mode: VcaCtlMode::ModEnv,

            portamento_time: 0.0_f32,
            s_and_h_glide_time: 0.0_f32,
            modosc_rise_time: 0.0_f32,

            vcf_env_attack: 0.0_f32,
            vcf_env_decay: 0.0_f32,
            vcf_env_sustain: 0.0_f32,
            vcf_env_release: 0.0_f32,

            mod_env_attack: 0.0_f32,
            mod_env_decay: 0.0_f32,
            mod_env_sustain: 0.0_f32,
            mod_env_release: 0.0_f32,

            vca_env_attack: 0.0_f32,
            vca_env_release: 0.0_f32,

            round_robbin_update_counter: 0,
        }
    }

    /// `ui.update()` updates the UI state by reading and storing the panel control user inputs.
    ///
    /// It is required to periodically call this function to updat the state of the UI controls.
    /// Reading the pots takes a significant amount of time. To avoid spending too much time in this function the
    /// controls are updated "round-robbin" style, meaning that for a given call to this function not all of the UI
    /// controls will be updated, but repeated calls to this function will eventually update all of the controls.
    pub fn update(&mut self, board: &mut Board) {
        // each call to this function updates a subset of the UI controls
        match self.round_robbin_update_counter {
            1 => {
                self.s_and_h_trig_src =
                    switch_3_way_to_trig_src(board.read_switch_3_way(Switch3way::SAndHTrigSrc))
            }
            2 => {
                self.vcf_env_trig_src =
                    switch_3_way_to_trig_src(board.read_switch_3_way(Switch3way::VcfEnvTrigSrc))
            }
            3 => {
                self.mod_env_trig_src =
                    switch_3_way_to_trig_src(board.read_switch_3_way(Switch3way::ModEnvTrigSrc))
            }
            4 => {
                self.vca_env_trig_src =
                    switch_3_way_to_trig_src(board.read_switch_3_way(Switch3way::VcaEnvTrigSrc))
            }
            5 => {
                self.auto_gate_src = match board.read_switch_3_way(Switch3way::AutoGateSrc) {
                    Switch3wayState::Up => AutoGateSource::PwmLfo,
                    Switch3wayState::Middle => AutoGateSource::ModOsc,
                    Switch3wayState::Down => AutoGateSource::Combo,
                }
            }
            6 => {
                self.auto_gate_logic_mode = match board.read_switch_3_way(Switch3way::AutoGateLogic)
                {
                    Switch3wayState::Up => AutoGateLogicMode::And,
                    Switch3wayState::Middle => AutoGateLogicMode::Or,
                    Switch3wayState::Down => AutoGateLogicMode::Xor,
                }
            }
            7 => {
                self.vca_ctl_mode = match board.read_switch_3_way(Switch3way::VcaCtlSrc) {
                    Switch3wayState::Up => VcaCtlMode::ModEnv,
                    Switch3wayState::Middle => VcaCtlMode::Drone,
                    Switch3wayState::Down => VcaCtlMode::Ar,
                }
            }
            8 => {
                self.portamento_time =
                    map_for_glide_or_rise_time(board.read_analog_signal(AnalogMuxChannel::I11))
            }
            9 => {
                self.s_and_h_glide_time =
                    map_for_glide_or_rise_time(board.read_analog_signal(AnalogMuxChannel::I10))
            }
            10 => {
                self.modosc_rise_time =
                    map_for_glide_or_rise_time(board.read_analog_signal(AnalogMuxChannel::I2))
            }
            11 => {
                self.vcf_env_attack =
                    map_for_adsr_time(board.read_analog_signal(AnalogMuxChannel::I9))
            }
            12 => {
                self.vcf_env_decay =
                    map_for_adsr_time(board.read_analog_signal(AnalogMuxChannel::I8))
            }
            13 => self.vcf_env_sustain = board.read_analog_signal(AnalogMuxChannel::I7),
            14 => {
                self.vcf_env_release =
                    map_for_adsr_time(board.read_analog_signal(AnalogMuxChannel::I6))
            }
            15 => {
                self.mod_env_attack =
                    map_for_adsr_time(board.read_analog_signal(AnalogMuxChannel::I12))
            }
            16 => {
                self.mod_env_decay =
                    map_for_adsr_time(board.read_analog_signal(AnalogMuxChannel::I13))
            }
            17 => self.mod_env_sustain = board.read_analog_signal(AnalogMuxChannel::I14),
            18 => {
                self.mod_env_release =
                    map_for_adsr_time(board.read_analog_signal(AnalogMuxChannel::I15))
            }
            19 => {
                self.vca_env_attack =
                    map_for_adsr_time(board.read_analog_signal(AnalogMuxChannel::I0))
            }
            20 => {
                self.vca_env_release =
                    map_for_adsr_time(board.read_analog_signal(AnalogMuxChannel::I1))
            }
            // reset the counter when we get to the end
            _ => self.round_robbin_update_counter = 0,
        }

        // incr the counter so next time we'll update the next control
        self.round_robbin_update_counter += 1;
    }

    /// `ui.scaled_pot(p)` is the scaled value of front panel potentiometer `p`
    ///
    /// The potentiometers are scaled differently depending on purpose, some represent time, some are unitless
    pub fn scaled_pot(&self, pot: Potentiometer) -> f32 {
        match pot {
            Potentiometer::Portamento => self.portamento_time,
            Potentiometer::SAndHGlide => self.s_and_h_glide_time,
            Potentiometer::ModOscRiseTime => self.modosc_rise_time,

            Potentiometer::VcfEnvAttack => self.vcf_env_attack,
            Potentiometer::VcfEnvDecay => self.vcf_env_decay,
            Potentiometer::VcfEnvSustain => self.vcf_env_sustain,
            Potentiometer::VcfEnvRelease => self.vcf_env_release,

            Potentiometer::ModEnvAttack => self.mod_env_attack,
            Potentiometer::ModEnvDecay => self.mod_env_decay,
            Potentiometer::ModEnvSustain => self.mod_env_sustain,
            Potentiometer::ModEnvRelease => self.mod_env_release,

            Potentiometer::VcaEnvAttack => self.vca_env_attack,
            Potentiometer::VcaEnvRelease => self.vca_env_release,
        }
    }

    /// `ui.trigger_switch(s)` is the current setting of front panel trigger switch `s`
    ///
    /// The trigger switches are for routing the trigger signals to the S&H and envelopes, each component gets a switch
    pub fn trigger_switch(&self, dest: TriggerSwitch) -> TriggerSource {
        match dest {
            TriggerSwitch::SampleAndHold => self.s_and_h_trig_src,
            TriggerSwitch::VcfEnv => self.vcf_env_trig_src,
            TriggerSwitch::ModEnv => self.mod_env_trig_src,
            TriggerSwitch::VcaEnv => self.vca_env_trig_src,
        }
    }

    /// `ui.auto_gate_src_switch()` is the current setting of the front panel AUTO GATE SRC switch
    ///
    /// This switch is used for selecting which signals are used as the auto-gate
    pub fn auto_gate_src_switch(&self) -> AutoGateSource {
        self.auto_gate_src
    }

    /// `ui.auto_gate_logic_switch()` is the current setting of the front panel AUTO GATE LOGIC switch
    ///
    /// This switch is used to select how the auto gate sources are combined when in COMBO mode
    pub fn auto_gate_logic_switch(&self) -> AutoGateLogicMode {
        self.auto_gate_logic_mode
    }

    /// `ui.vca_ctl_mode_switch()` is the current value of the front panel VCA control mode switch
    ///
    /// This switch is used to select how the VCA is controlled
    pub fn vca_ctl_mode_switch(&self) -> VcaCtlMode {
        self.vca_ctl_mode
    }
}

/// `switch_3_way_to_trig_src(s3)` maps the 3-way switch state `s3` to a trigger source, as defined by the front panel
fn switch_3_way_to_trig_src(switch_state: Switch3wayState) -> TriggerSource {
    match switch_state {
        Switch3wayState::Up => TriggerSource::Gate,
        Switch3wayState::Middle => TriggerSource::Both,
        Switch3wayState::Down => TriggerSource::Auto,
    }
}

/// `map_for_adsr_time(v)` maps raw value `v` in `[0.0, 1.0]` to a time in seconds for controlling ADSR times
fn map_for_adsr_time(raw_adc_val: f32) -> f32 {
    // TODO
    raw_adc_val
}

/// `map_for_glide_or_rise_time(v)` maps raw value `v` in `[0.0. 1.0]` to a time in seconds for controlling glide
fn map_for_glide_or_rise_time(raw_adc_val: f32) -> f32 {
    // TODO
    raw_adc_val
}

/// Enumerated front panel potentiometers are represented here
#[derive(Clone, Copy)]
pub enum Potentiometer {
    Portamento,
    SAndHGlide,
    ModOscRiseTime,

    VcfEnvAttack,
    VcfEnvDecay,
    VcfEnvSustain,
    VcfEnvRelease,

    ModEnvAttack,
    ModEnvDecay,
    ModEnvSustain,
    ModEnvRelease,

    VcaEnvAttack,
    VcaEnvRelease,
}

/// Enumerated front panel trigger switches are represented here
///
/// Each of the sample&hold and envelopes gets a front panel switch used to control which trigger source will apply
/// to the trigger-able component
#[derive(Clone, Copy)]
pub enum TriggerSwitch {
    SampleAndHold,
    VcfEnv,
    ModEnv,
    VcaEnv,
}

/// Enumerated trigger sources are represented here
///
/// The sample&hold and envelopes may be triggered by either the manual gate, the auto gate, or the combination of both
#[derive(Clone, Copy)]
pub enum TriggerSource {
    Gate,
    Both,
    Auto,
}

/// Enumerated auto-gate sources are represented here
///
/// The auto gate may come from the PWM LFO, the MODOSC, or from a logical combination of both
#[derive(Clone, Copy)]
pub enum AutoGateSource {
    PwmLfo,
    ModOsc,
    Combo,
}

/// Enumerated auto-gate logic modes are represented here
///
/// When the auto-gate SRC is set to Combo, the logic represented by this enum is applied to the PWM LFO and MODOSC
#[derive(Clone, Copy)]
pub enum AutoGateLogicMode {
    And,
    Or,
    Xor,
}

/// Enumerated VCA control modes are represented here
#[derive(Clone, Copy)]
pub enum VcaCtlMode {
    ModEnv,
    Drone,
    Ar,
}
