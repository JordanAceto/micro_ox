use crate::gate_routing::{GateRouting, GateSignal, GateState};
use crate::ui::{Potentiometer, Ui};
use synth_utils::adsr::{Adsr, Input};

/// The Envelope Generators used by the system are represented here
pub struct Envelopes {
    vcf_env: Adsr,
    mod_env: Adsr,
    vca_env: Adsr,
}

impl Envelopes {
    /// `Envelopes::new(sr)` is a new structure holding the system Envelope Generators with sample rate `sr`
    pub fn new(sample_rate_hz: f32) -> Self {
        // The VCA envelope is a simpler AR with only Attack and Release controls.
        // At creation we set Decay to an arbitrary fast time and set Sustain to full-scale. The `D` and `S` controls
        // are never used after this. This will make it act like a typical synth AR generator
        let mut vca_env = Adsr::new(sample_rate_hz);
        vca_env.set_input(Input::Decay(0.001_f32.into()));
        vca_env.set_input(Input::Sustain(1.0_f32.into()));

        // The VCF env and MOD env are full-featured ADSR envelopes, the default initilaization is fine for these
        Self {
            vcf_env: Adsr::new(sample_rate_hz),
            mod_env: Adsr::new(sample_rate_hz),
            vca_env,
        }
    }

    /// `es.vcf_env()` is the current value of the VCF envelope generator in `[0.0, 1.0]`
    pub fn vcf_env(&self) -> f32 {
        self.vcf_env.value()
    }

    /// `es.mod_env()` is the current value of the MOD envelope generator in `[0.0, 1.0]`
    pub fn mod_env(&self) -> f32 {
        self.mod_env.value()
    }

    /// `es.vca_env()` is the current value of the VCA envelope generator in `[0.0, 1.0]`
    pub fn vca_env(&self) -> f32 {
        self.vca_env.value()
    }

    /// `es.tick(gr)` injects the gate signals held by `gr` and ticks each envelope, must be called at the sample rate
    pub fn tick(&mut self, gate_routing: &GateRouting) {
        match gate_routing.state(GateSignal::VcfEnv) {
            GateState::Rising => self.vcf_env.gate_on(),
            GateState::Falling => self.vcf_env.gate_off(),
            _ => (),
        };

        match gate_routing.state(GateSignal::ModEnv) {
            GateState::Rising => self.mod_env.gate_on(),
            GateState::Falling => self.mod_env.gate_off(),
            _ => (),
        };

        match gate_routing.state(GateSignal::VcaEnv) {
            GateState::Rising => self.vca_env.gate_on(),
            GateState::Falling => self.vca_env.gate_off(),
            _ => (),
        };

        self.vcf_env.tick();
        self.mod_env.tick();
        self.vca_env.tick();
    }

    /// `es.update_inputs(ui)` updates all of the Envelope inputs with the current values held by the `ui`
    pub fn update_inputs(&mut self, ui: &mut Ui) {
        self.vcf_env.set_input(Input::Attack(
            ui.scaled_pot(Potentiometer::VcfEnvAttack).into(),
        ));
        self.vcf_env.set_input(Input::Decay(
            ui.scaled_pot(Potentiometer::VcfEnvDecay).into(),
        ));
        self.vcf_env.set_input(Input::Sustain(
            ui.scaled_pot(Potentiometer::VcfEnvSustain).into(),
        ));
        self.vcf_env.set_input(Input::Release(
            ui.scaled_pot(Potentiometer::VcfEnvRelease).into(),
        ));

        self.mod_env.set_input(Input::Attack(
            ui.scaled_pot(Potentiometer::ModEnvAttack).into(),
        ));
        self.mod_env.set_input(Input::Decay(
            ui.scaled_pot(Potentiometer::ModEnvDecay).into(),
        ));
        self.mod_env.set_input(Input::Sustain(
            ui.scaled_pot(Potentiometer::ModEnvSustain).into(),
        ));
        self.mod_env.set_input(Input::Release(
            ui.scaled_pot(Potentiometer::ModEnvRelease).into(),
        ));

        self.vca_env.set_input(Input::Attack(
            ui.scaled_pot(Potentiometer::VcaEnvAttack).into(),
        ));
        self.vca_env.set_input(Input::Release(
            ui.scaled_pot(Potentiometer::VcaEnvRelease).into(),
        ));
        // ignore VCA envelope Decay and Sustain inputs, these were set during initialization and don't need to change
    }
}
