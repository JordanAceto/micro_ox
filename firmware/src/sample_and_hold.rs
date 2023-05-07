use crate::gate_routing::GateState;
use synth_utils::glide_processor::GlideProcessor;

/// The Sample And Hold used by the system is represented here
///
/// Analog signals are mixed in hardware and then digitized by the onboard ADC
pub struct SampleAndHold {
    stepped_value: f32,
    value_with_glide: f32,
    glide: GlideProcessor,
}

impl SampleAndHold {
    /// `SampleAndHold::new(sr)` is a new sample and hold with sample rate `sr`
    pub fn new(sample_rate_hz: f32) -> Self {
        Self {
            stepped_value: 0.0_f32,
            value_with_glide: 0.0_f32,
            glide: GlideProcessor::new(sample_rate_hz),
        }
    }

    /// `sh.tick(i, g)` advances the sample & hold by 1 tick, must be called at the sample rate
    ///
    /// # Arguments
    ///
    /// * `raw_input` - the raw input to the sample & hold, in `[0.0, 1.0]`
    ///
    /// * `gate` - the state of the gate which triggers the sample & hold to hold the input on rising edges
    pub fn tick(&mut self, raw_input: f32, gate: GateState) {
        if gate == GateState::Rising {
            self.stepped_value = raw_input;
        }
        self.value_with_glide = self.glide.process(self.stepped_value)
    }

    /// `sh.value()` is the current value of the sample & hold with glide applied, in `[0.0, 1.0]`
    pub fn value(&self) -> f32 {
        self.value_with_glide
    }

    /// `sh.set_glide_time(t)` sets the portamento time for the internal glide processor to the new time `t`
    ///
    /// # Arguments:
    ///
    /// * `t` - the new value for the glide control time, in `[0.0, 10.0]`
    ///
    /// Times that would be faster than sample_rate/2 are clamped.
    ///
    /// This function can be somewhat costly, so don't call it more than necessary
    pub fn set_glide_time(&mut self, time: f32) {
        self.glide.set_time(time)
    }
}
