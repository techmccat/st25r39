# ST25R39 examples

This crate (based on [knurling-rs/app-template](https://github.com/knurling-rs/app-template))
contains examples and tests for using the st25r39 driver crate.

## Hardware

The examples are written for a NUCLEO-G474RE devboard with an X-NUCLEO-NFC08A1 (or NFC06A1)
plugged into it, but can be easily adapted to any platform that supports the
embedded-hal 1.0 SPI and digital interfaces

## Examples

### init.rs

Starts the ST25 oscillator, measures the voltage on the VDD rail and configures the regulator accordingly
