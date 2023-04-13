# Software design

This directory contains first and foremost the Firmware developed for the drone, as well as any supporting software used for communication and data acquisition, processing, etc.

The rust-based flight controller can be found in
**[rusty-quad](/software/rusty-quad/)**

## Transition from C++ to Rust

The development of the firmware for the drone, has moved from an Arduino (C++) based framework, to Rust. The main micro-controller for development has also transitioned from the ESP32 platform, to the Raspberry Pi Pico (RP2040).

The rust-based firmware, unlike its C++ counterpart, has not yet proven air worthy, since some modules had to be written from scratch. This is due to embedded Rust still being in its infancy.

## Why Rust?

I started experimenting with Rust about a year ago, due to me finding its promises of memory safety, strict (yet somehow incredibly flexible) type system and C-like performance intriguing. On top of that, the modern tooling, package manager and passionate user base had me sold.

Equally intriguing to me, was the prospect of being able to develop micro-controller-embedded applications in Rust, with the amazing [**knurling-rs**](https://knurling.ferrous-systems.com/) tools by Ferrous Systems.

On top of that, Rust allows for high-speed, low overhead asynchronous programs using the built-in async/await syntax. For embedded applications, there is the [**Embassy**](https://github.com/embassy-rs/embassy) project specifically aimed at supporting async on embedded platforms.
