use crate::ui::{AutoGateLogicMode, AutoGateSource, TriggerSource, TriggerSwitch, Ui};

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

    /// `gr.state(g)` is the current state of gate signal `g` after being routed through the panel controls
    ///
    /// The `gr.update()` function must be called periodically for this function to be valid
    pub fn state(&self, gate: GateSignal) -> GateState {
        match gate {
            GateSignal::SAndH => self.s_and_h_gate,
            GateSignal::VcfEnv => self.vcf_env_gate,
            GateSignal::ModEnv => self.mod_env_gate,
            GateSignal::VcaEnv => self.vca_env_gate,
        }
    }

    /// `gr.update(p, mo, m, e, u)` updates the internal state of the Gate Routing structure based on the boolean inputs
    ///
    /// The various boolean gate singnals are routed and combined according to the position of the front panel UI
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
        let auto_gate = match ui.auto_gate_src_switch() {
            AutoGateSource::PwmLfo => pwm_lfo_gate,
            AutoGateSource::ModOsc => modosc_gate,
            AutoGateSource::Combo => {
                apply_logic(pwm_lfo_gate, modosc_gate, ui.auto_gate_logic_switch())
            }
        };

        let manual_gate = midi_gate | ext_gate;

        let this_s_and_h_gate = combine_manual_and_auto_gate(
            manual_gate,
            auto_gate,
            ui.trigger_switch(TriggerSwitch::SampleAndHold),
        );
        self.s_and_h_gate =
            gate_state_from_last_and_curr(self.last_s_and_h_gate, this_s_and_h_gate);
        self.last_s_and_h_gate = this_s_and_h_gate;

        let this_vcf_env_gate = combine_manual_and_auto_gate(
            manual_gate,
            auto_gate,
            ui.trigger_switch(TriggerSwitch::VcfEnv),
        );
        self.vcf_env_gate =
            gate_state_from_last_and_curr(self.last_vcf_env_gate, this_vcf_env_gate);
        self.last_s_and_h_gate = this_s_and_h_gate;

        let this_mod_env_gate = combine_manual_and_auto_gate(
            manual_gate,
            auto_gate,
            ui.trigger_switch(TriggerSwitch::ModEnv),
        );
        self.mod_env_gate =
            gate_state_from_last_and_curr(self.last_mod_env_gate, this_mod_env_gate);
        self.last_mod_env_gate = this_mod_env_gate;

        let this_vca_env_gate = combine_manual_and_auto_gate(
            manual_gate,
            auto_gate,
            ui.trigger_switch(TriggerSwitch::VcaEnv),
        );
        self.vca_env_gate =
            gate_state_from_last_and_curr(self.last_vca_env_gate, this_vca_env_gate);
        self.last_vca_env_gate = this_vca_env_gate;
    }
}

/// Enumerated gate signals are represented here, each trigger-able component gets its own gate signal
#[derive(Clone, Copy)]
pub enum GateSignal {
    SAndH,
    VcfEnv,
    ModEnv,
    VcaEnv,
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

/// `combine_manual_and_auto_gate(m, a, t)` combines gates `m` and `a` based on trigger source `t`
fn combine_manual_and_auto_gate(
    manual_gate: bool,
    auto_gate: bool,
    trigger_src: TriggerSource,
) -> bool {
    match trigger_src {
        TriggerSource::Gate => manual_gate,
        TriggerSource::Auto => auto_gate,
        TriggerSource::Both => manual_gate & auto_gate,
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
