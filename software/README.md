# Software design

This directory contains first and foremost the Firmware developed for the drone, as well as any supporting software used for communication and data acquisition, processing, etc.

## Transition from C++ to Rust

The development of the firmware for the drone, has moved from an Arduino (C++) based framework, to Rust. The main micro-controller for development has also transitioned from the ESP32 platform, to the Raspberry Pi Pico (RP2040).

The rust-based firmware, unlike its C++ counterpart, has not yet proven air worthy, since some modules had to be written from scratch. This is due to embedded Rust still being in its infancy.

## Why Rust?

I started experimenting with Rust about a year ago, due to me finding its promises of memory safety, strict (yet somehow incredibly flexible) type system and C-like performance intriguing. On top of that, the modern tooling, package manager and passionate user base had me sold.

Equally intriguing to me, was the prospect of being able to develop micro-controller-embedded applications in Rust, with the amazing [**knurling-rs**](https://knurling.ferrous-systems.com/) tools by Ferrous Systems.

On top of that, Rust allows for high-speed, low overhead asynchronous programs using the built-in async/await syntax. For embedded applications, there is the [**Embassy**](https://github.com/embassy-rs/embassy) project specifically aimed at supporting async on embedded platforms.

Some of these tools are not yet finished, and sometimes rough around the edges. However, the fact that they are already super powerful is even more reason to start working with them already.

## Progress so far

During the transition from C++ to Rust, some groundwork had to be done, in order to get back to the same level of features. The Embedded Rust ecosystem, while sprawling, is still in its infancy, and much hardware is still unsupported, some of which were developed or modified to fit this project.

----

[DShot protocol implementation for RP2040 using PIO](https://github.com/peterkrull/quad-dshot-pio/)

[Linear Kalman filter implementation in Rust](https://github.com/peterkrull/kalman_filter_rust/)

[(Port) DShot frame encoder in Rust  ](https://github.com/peterkrull/dshot-encoder)

[(Forked) Platform-agnostic no_std SBUS-parser in Rust](https://github.com/peterkrull/sbus)


