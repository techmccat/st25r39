# ST25R39 examples

This crate (based on [knurling-rs/app-template](https://github.com/knurling-rs/app-template))
contains examples and tests for using the st25r39 driver crate.

## Hardware

The examples are written for a NUCLEO-G474RE devboard with an X-NUCLEO-NFC08A1 (or NFC06A1)
plugged into it, but can be easily adapted to any platform that supports the
embedded-hal 1.0 SPI and digital interfaces

## Examples

### select_nfca.rs

Starts the ST25 oscillator, measures the voltage on the VDD rail and configures the regulator accordingly,
then polls for ISO 14443A tags and tries to initialize them

### tuning.rs

Shows off the automatic antenna tuning (AAT) feature of the ST25R3916B

Place a piece of metal or a NFC card near the antenna to see how the numbers magically worsen

### blinky.rs

Demonstrates using the peripheral's general purpose timer as a delay provider
