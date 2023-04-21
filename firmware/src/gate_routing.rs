use crate::ui::{AutoGateLogicMode, AutoGateSource, TriggerSource, Ui};

/// A structure for routing gate signals is represented here.
///
/// In this system there are a few different gate signals which can trigger the envelope generators. These gate signals
/// are routed and combined based on the positions of the front panel controls. This module implements the logic related
/// to the front panel controls for choosing which gates control which envelope generators.
pub struct GateRouting {
    // last states allow us to detect rising and falling edges
    last_s_and_h_gate: bool,
    last_vcf_env_gate: bool,
    last_mod_env_gate: bool,
    last_vca_env_gate: bool,

    s_and_h_gate: GateState,
    vcf_env_gate: GateState,
    mod_env_gate: GateState,
    vca_env_gate: GateState,
}

impl GateRouting {
    /// `GateRouting::new()` is a new structure representing the gate routing
    pub fn new() -> Self {
        Self {
            // cached last-state boolean signals are used to detect rising and falling edges
            last_s_and_h_gate: false,
            last_vcf_env_gate: false,
            last_mod_env_gate: false,
            last_vca_env_gate: false,

            s_and_h_gate: GateState::Low,
            vcf_env_gate: GateState::Low,
            mod_env_gate: GateState::Low,
            vca_env_gate: GateState::Low,
        }
    }

    /// `gr.s_and_h_gate()` is the current state of the sample&hold gate after being routed through the panel controls
    ///
    /// The `gr.update()` function must be called periodically for this function to be valid
    pub fn s_and_h_gate(&self) -> GateState {
        self.s_and_h_gate
    }

    /// `gr.vcf_env_gate()` is the current state of the VCF ADSR gate after being routed through the panel controls
    ///
    /// The `gr.update()` function must be called periodically for this function to be valid
    pub fn vcf_env_gate(&self) -> GateState {
        self.vcf_env_gate
    }

    /// `gr.mod_env_gate()` is the current state of the MOD ADSR gate after being routed through the panel controls
    ///
    /// The `gr.update()` function must be called periodically for this function to be valid
    pub fn mod_env_gate(&self) -> GateState {
        self.mod_env_gate
    }

    /// `gr.vca_env_gate()` is the current state of the VCA AR gate after being routed through the panel controls
    ///
    /// The `gr.update()` function must be called periodically for this function to be valid
    pub fn vca_env_gate(&self) -> GateState {
        self.vca_env_gate
    }

    /// `gr.update(p, mo, m, e, u)` updates the internal state of the Gate Routing structure based on the boolean inputs
    ///
    /// This function must be called periodically as new gate inputs occur
    pub fn update(
        &mut self,
        pwm_lfo_gate: bool,
        modosc_gate: bool,
        midi_gate: bool,
        ext_gate: bool,
        ui: &mut Ui,
    ) {
        let auto_gate = match ui.auto_gate_src() {
            AutoGateSource::PwmLfo => pwm_lfo_gate,
            AutoGateSource::ModOsc => modosc_gate,
            AutoGateSource::Combo => {
                apply_logic(pwm_lfo_gate, modosc_gate, ui.auto_gate_logic_mode())
            }
        };

        let manual_gate = midi_gate | ext_gate;

        let this_s_and_h_gate =
            combine_ext_and_auto_gate(manual_gate, auto_gate, ui.s_and_h_trig_src());
        self.s_and_h_gate =
            gate_state_from_last_and_curr(self.last_s_and_h_gate, this_s_and_h_gate);
        self.last_s_and_h_gate = this_s_and_h_gate;

        let this_vcf_env_gate =
            combine_ext_and_auto_gate(manual_gate, auto_gate, ui.vcf_env_trig_src());
        self.vcf_env_gate =
            gate_state_from_last_and_curr(self.last_vcf_env_gate, this_vcf_env_gate);
        self.last_s_and_h_gate = this_s_and_h_gate;

        let this_mod_env_gate =
            combine_ext_and_auto_gate(manual_gate, auto_gate, ui.mod_env_trig_src());
        self.mod_env_gate =
            gate_state_from_last_and_curr(self.last_mod_env_gate, this_mod_env_gate);
        self.last_mod_env_gate = this_mod_env_gate;

        let this_vca_env_gate =
            combine_ext_and_auto_gate(manual_gate, auto_gate, ui.vca_env_trig_src());
        self.vca_env_gate =
            gate_state_from_last_and_curr(self.last_vca_env_gate, this_vca_env_gate);
        self.last_vca_env_gate = this_vca_env_gate;
    }
}

/// Enumerated gate states are represented here. A gate signal will be in exactly one of these states at any given time
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum GateState {
    Low,
    Rising,
    High,
    Falling,
}

/// `apply_logic(p1, p2, lm)` applies the logic represented by `lm` to boolean signals `p1` and `p2`
fn apply_logic(p1: bool, p2: bool, logic_mode: AutoGateLogicMode) -> bool {
    match logic_mode {
        AutoGateLogicMode::And => p1 & p2,
        AutoGateLogicMode::Or => p1 | p2,
        AutoGateLogicMode::Xor => p1 ^ p2,
    }
}

/// `combine_ext_and_auto_gate(e, a, t)` combines gates `e` and `a` based on trigger source `t`
fn combine_ext_and_auto_gate(ext_gate: bool, auto_gate: bool, trigger_src: TriggerSource) -> bool {
    match trigger_src {
        TriggerSource::Gate => ext_gate,
        TriggerSource::Auto => auto_gate,
        TriggerSource::Both => ext_gate & auto_gate,
    }
}

/// `gate_state_from_last_and_curr(l, c)` is the gate state that results from the last and current inputs
fn gate_state_from_last_and_curr(last: bool, curr: bool) -> GateState {
    match (last, curr) {
        (false, false) => GateState::Low,
        (false, true) => GateState::Rising,
        (true, true) => GateState::High,
        (true, false) => GateState::Falling,
    }
}
