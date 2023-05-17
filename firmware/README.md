# Rust Language STM32L412KBU Synthesizer Firmware

## Features
- MIDI receiver listens for MIDI messages via UART
- front panel potentiometers and switches are read by the microcontroller via MUX
- ADSR envelopes are generated digitally
- the Sample & Hold signal is digitized
- White Noise is generated via the built in stm32 hardware RNG and output via PWM
- external DACs generate analog voltages to control various system parameters
- LEDs driven via shift register show the state of the envelope and S&H triggers
