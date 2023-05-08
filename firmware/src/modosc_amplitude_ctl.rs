use synth_utils::glide_processor::GlideProcessor;

/// MODOSC Amplitude Control is represented here
///
/// The MODOSC is an analog oscillator intended for modulation purposes. The amplitude of the MODOSC is controlled
/// by the onboard DAC. A front panel switch toggles the MODOSC on and off, and the transition is smoothed out by a
/// glide processor with adjustable time.
pub struct ModOscAmplitudeCtl {
    value: f32,
    glide: GlideProcessor,
}

impl ModOscAmplitudeCtl {
    /// `ModOscAmplitudeCtl::new(sr)` is a new modosc amplitude control structure with sample rate `sr`
    pub fn new(sample_rate_hz: f32) -> Self {
        Self {
            value: 0.0_f32,
            glide: GlideProcessor::new(sample_rate_hz),
        }
    }

    /// `m.tick(i, g)` advances the modosc amplitude control by 1 tick, must be called at the sample rate
    ///
    /// # Arguments
    ///
    /// * `on` - boolean signal to turn the modosc amplitude on or off
    pub fn tick(&mut self, on: bool) {
        // glide between zero and one based on the front panel toggle switch
        if on {
            self.value = self.glide.process(1.0_f32)
        } else {
            self.value = self.glide.process(0.0_f32)
        }
    }

    /// `m.value()` is the current value of the modosc amplitude control signal in `[0.0, 1.0]`
    pub fn value(&self) -> f32 {
        self.value
    }

    /// `m.led_driver_vout()` is the modosc amplitude control as a voltage suitable for driving the push-button LED
    ///
    /// The LED has a diode drop which must be overcome before it lightes up. It is driven from the onboard DAC
    /// which has a maximum voltage of 2.5
    pub fn led_driver_vout(&self) -> f32 {
        // TODO test this and scale as needed so the LED looks good, it might be a happy coincidence that the drop and
        // DAC range work out so we can just add the drop, or we might need to do more scaling and bending to look good
        let diode_drop = 1.5_f32;
        self.value + diode_drop
    }

    /// `m.set_glide_time(t)` sets the rise time for the internal glide processor to the new time `t`
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
