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
        }
    }

    /// `ui.update()` updates the UI state by reading and storing the panel control user inputs.
    ///
    /// It is required to periodically call this function to updat the state of the UI controls. Since these controls
    /// are manually adjusted by the user, they don't need to be updated very fast, just fast enough that they don't
    /// feel sluggish to the user.
    pub fn update(&mut self, board: &mut Board) {
        self.s_and_h_trig_src =
            switch_3_way_to_trig_src(board.read_switch_3_way(Switch3way::SAndHTrigSrc));

        self.vcf_env_trig_src =
            switch_3_way_to_trig_src(board.read_switch_3_way(Switch3way::VcfEnvTrigSrc));

        self.mod_env_trig_src =
            switch_3_way_to_trig_src(board.read_switch_3_way(Switch3way::ModEnvTrigSrc));

        self.vca_env_trig_src =
            switch_3_way_to_trig_src(board.read_switch_3_way(Switch3way::VcaEnvTrigSrc));

        self.auto_gate_src = match board.read_switch_3_way(Switch3way::AutoGateSrc) {
            Switch3wayState::Up => AutoGateSource::PwmLfo,
            Switch3wayState::Middle => AutoGateSource::ModOsc,
            Switch3wayState::Down => AutoGateSource::Combo,
        };

        self.auto_gate_logic_mode = match board.read_switch_3_way(Switch3way::AutoGateLogic) {
            Switch3wayState::Up => AutoGateLogicMode::And,
            Switch3wayState::Middle => AutoGateLogicMode::Or,
            Switch3wayState::Down => AutoGateLogicMode::Xor,
        };

        self.vca_ctl_mode = match board.read_switch_3_way(Switch3way::VcaCtlSrc) {
            Switch3wayState::Up => VcaCtlMode::ModEnv,
            Switch3wayState::Middle => VcaCtlMode::Drone,
            Switch3wayState::Down => VcaCtlMode::Ar,
        };

        self.portamento_time =
            scale_for_glide_or_rise_time(board.read_analog_signal(AnalogMuxChannel::I11));
        self.s_and_h_glide_time =
            scale_for_glide_or_rise_time(board.read_analog_signal(AnalogMuxChannel::I10));
        self.modosc_rise_time =
            scale_for_glide_or_rise_time(board.read_analog_signal(AnalogMuxChannel::I2));

        self.vcf_env_attack = scale_for_adsr_time(board.read_analog_signal(AnalogMuxChannel::I9));
        self.vcf_env_decay = scale_for_adsr_time(board.read_analog_signal(AnalogMuxChannel::I8));
        self.vcf_env_sustain = board.read_analog_signal(AnalogMuxChannel::I7);
        self.vcf_env_release = scale_for_adsr_time(board.read_analog_signal(AnalogMuxChannel::I6));

        self.mod_env_attack = scale_for_adsr_time(board.read_analog_signal(AnalogMuxChannel::I12));
        self.mod_env_decay = scale_for_adsr_time(board.read_analog_signal(AnalogMuxChannel::I13));
        self.mod_env_sustain = board.read_analog_signal(AnalogMuxChannel::I14);
        self.mod_env_release = scale_for_adsr_time(board.read_analog_signal(AnalogMuxChannel::I15));

        self.vca_env_attack = scale_for_adsr_time(board.read_analog_signal(AnalogMuxChannel::I0));
        self.vca_env_release = scale_for_adsr_time(board.read_analog_signal(AnalogMuxChannel::I1));
    }

    pub fn s_and_h_trig_src(&self) -> TriggerSource {
        self.s_and_h_trig_src
    }

    pub fn vcf_env_trig_src(&self) -> TriggerSource {
        self.vcf_env_trig_src
    }

    pub fn mod_env_trig_src(&self) -> TriggerSource {
        self.mod_env_trig_src
    }

    pub fn vca_env_trig_src(&self) -> TriggerSource {
        self.vca_env_trig_src
    }

    pub fn auto_gate_src(&self) -> AutoGateSource {
        self.auto_gate_src
    }

    pub fn auto_gate_logic_mode(&self) -> AutoGateLogicMode {
        self.auto_gate_logic_mode
    }

    pub fn vca_ctl_mode(&self, board: &mut Board) -> VcaCtlMode {
        self.vca_ctl_mode
    }

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
}

fn switch_3_way_to_trig_src(switch_state: Switch3wayState) -> TriggerSource {
    match switch_state {
        Switch3wayState::Up => TriggerSource::Gate,
        Switch3wayState::Middle => TriggerSource::Both,
        Switch3wayState::Down => TriggerSource::Auto,
    }
}

fn scale_for_adsr_time(raw_adc_val: f32) -> f32 {
    // TODO
    raw_adc_val
}

fn scale_for_glide_or_rise_time(raw_adc_val: f32) -> f32 {
    // TODO
    raw_adc_val
}

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

#[derive(Clone, Copy)]
pub enum TriggerSource {
    Gate,
    Both,
    Auto,
}

#[derive(Clone, Copy)]
pub enum AutoGateSource {
    PwmLfo,
    ModOsc,
    Combo,
}

#[derive(Clone, Copy)]
pub enum AutoGateLogicMode {
    And,
    Or,
    Xor,
}

#[derive(Clone, Copy)]
pub enum VcaCtlMode {
    ModEnv,
    Drone,
    Ar,
}
