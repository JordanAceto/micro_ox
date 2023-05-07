#![no_std]
#![no_main]

mod board;
mod envelopes;
mod gate_routing;
mod modosc_amplitude_ctl;
mod sample_and_hold;
mod ui;

use crate::board::{
    Board, Dac128S085Channel, Dac8162Channel, Led, DAC128S085_MAX_VOUT, DAC8162_MAX_VOUT,
};
use crate::gate_routing::GateSignal;

use synth_utils::{glide_processor, mono_midi_receiver};

use panic_halt as _;

use cortex_m_rt::entry;

const DAC_UPDATE_SR: f32 = board::TIM2_FREQ_HZ as f32;

#[entry]
fn main() -> ! {
    let mut board = Board::init();
    let mut ui = ui::Ui::new();
    let mut gate_routing = gate_routing::GateRouting::new();

    let mut midi = mono_midi_receiver::MonoMidiReceiver::new(0);
    let mut midi_glide = glide_processor::GlideProcessor::new(DAC_UPDATE_SR);

    let mut envelopes = envelopes::Envelopes::new(DAC_UPDATE_SR);
    let mut sample_and_hold = sample_and_hold::SampleAndHold::new(DAC_UPDATE_SR);
    let mut modosc_amplitude_ctl = modosc_amplitude_ctl::ModOscAmplitudeCtl::new(DAC_UPDATE_SR);

    // set some MIDI controls to full-scale to start, this way if nobody plugs in a MIDI controller the volume,
    // mod-wheel ect won't be all the way down. Use running status to do all the CC messages
    for b in [
        /* volume */ 0xB0, 0x07, 127, /* mod wheel */ 0x01, 127, /* VCF Q */ 0x4A,
        127,
        /* velocity, no longer in CC running-status, arbitrary note-on with max velocity and then turn the note off again */
        0x90, 0, 127, 0x80, 0, 0,
    ] {
        midi.parse(b)
    }

    loop {
        // handle MIDI input as quickly as possible so we don't miss bytes
        if let Some(byte) = board.read_midi() {
            midi.parse(byte)
        };

        if board.tim2_timeout() {
            board.set_debug_pin(true);

            ui.update(&mut board);

            gate_routing.update(
                board.pwm_lfo_sqr(),
                board.modosc_sqr(),
                midi.gate(),
                board.ext_gate(),
                &mut ui,
            );

            ////////////////////////////////////////////////////////////////////
            //
            // Envelopes
            //
            ////////////////////////////////////////////////////////////////////

            envelopes.update_inputs(&mut ui);

            envelopes.tick(&gate_routing);

            board.dac128S085_set_vout(
                envelopes.vcf_env() * DAC128S085_MAX_VOUT,
                Dac128S085Channel::C,
            );

            board.dac128S085_set_vout(
                envelopes.mod_env() * DAC128S085_MAX_VOUT,
                Dac128S085Channel::E,
            );

            // the VCA can be controlled from a few different signals depending on the front panel switch
            let vca_env = match ui.vca_ctl_mode_switch() {
                ui::VcaCtlMode::ModEnv => envelopes.mod_env(),
                ui::VcaCtlMode::Ar => envelopes.vca_env(),
                ui::VcaCtlMode::Drone => 1.0_f32,
            };
            board.dac128S085_set_vout(vca_env * DAC128S085_MAX_VOUT, Dac128S085Channel::A);

            ////////////////////////////////////////////////////////////////////
            //
            // Sample & Hold
            //
            ////////////////////////////////////////////////////////////////////

            sample_and_hold.set_glide_time(ui.scaled_pot(ui::Potentiometer::SAndHGlide));

            // TODO: quantize s&h?
            sample_and_hold.tick(
                board.sample_and_hold(),
                gate_routing.state(gate_routing::GateSignal::SAndH),
            );

            board.dac8162_set_vout(
                sample_and_hold.value() * DAC8162_MAX_VOUT,
                Dac8162Channel::B,
            );

            ////////////////////////////////////////////////////////////////////
            //
            // ModOsc Amplitude Control
            //
            ////////////////////////////////////////////////////////////////////

            modosc_amplitude_ctl.set_glide_time(ui.scaled_pot(ui::Potentiometer::ModOscRiseTime));

            modosc_amplitude_ctl.tick(board.modosc_toggle_switch());

            // the modosc amplitude is controlled by both the front panel on/off switch and MIDI mod-wheel
            board.dac128S085_set_vout(
                modosc_amplitude_ctl.value() * midi.mod_wheel() * board::DAC128S085_MAX_VOUT,
                board::Dac128S085Channel::D,
            );

            ////////////////////////////////////////////////////////////////////
            //
            // MIDI Pitch CV
            //
            ////////////////////////////////////////////////////////////////////

            let midi_1v_per_oct = note_num_to_dac8164_1v_per_oct(midi.note_num());

            midi_glide.set_time(ui.scaled_pot(ui::Potentiometer::Portamento));

            let midi_with_glide = midi_glide.process(midi_1v_per_oct);

            // scale pitch bend for +/- 2 semitones
            let midi_pitch_bend = midi.pitch_bend() * 2.0_f32 / 12.0_f32;

            board.dac8162_set_vout(midi_with_glide + midi_pitch_bend, board::Dac8162Channel::A);

            ////////////////////////////////////////////////////////////////////
            //
            // Misc MIDI controls
            //
            ////////////////////////////////////////////////////////////////////

            board.dac128S085_set_vout(
                midi.vcf_cutoff() * DAC128S085_MAX_VOUT,
                Dac128S085Channel::G,
            );

            board.dac128S085_set_vout(
                midi.vcf_resonance() * DAC128S085_MAX_VOUT,
                Dac128S085Channel::B,
            );

            board.dac128S085_set_vout(midi.volume() * DAC128S085_MAX_VOUT, Dac128S085Channel::H);

            ////////////////////////////////////////////////////////////////////
            //
            // LEDs
            //
            ////////////////////////////////////////////////////////////////////

            board.set_led(Led::SAndHTrig, gate_routing.state(GateSignal::SAndH).into());
            board.set_led(
                Led::VcfEnvTrig,
                gate_routing.state(GateSignal::VcfEnv).into(),
            );
            board.set_led(
                Led::ModEnvTrig,
                gate_routing.state(GateSignal::ModEnv).into(),
            );
            board.set_led(Led::VcaTrig, gate_routing.state(GateSignal::VcaEnv).into());
            board.set_led(
                Led::AutoGate,
                gate_routing.state(GateSignal::AutoGate).into(),
            );

            // the MODOSC LED is driven from an analog voltage to fade in and out
            board.dac128S085_set_vout(
                modosc_amplitude_ctl.led_driver_vout(),
                board::Dac128S085Channel::F,
            );

            board.set_debug_pin(false);
        }
    }
}

/// `note_num_to_dac8164_1v_per_oct(n)` is the note number `n` scaled to 1volt/octave
fn note_num_to_dac8164_1v_per_oct(note_num: u8) -> f32 {
    note_num as f32 / 12.0_f32
}
