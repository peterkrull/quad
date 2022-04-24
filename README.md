# Project quad

This git repository is a personal project of mine to make a well flying autonomous quad copter using cheap off-the-shelf components and good control theory and algorithms.

## Goals of this project

- [x] Design and assemble the initial hardware capable of flight
  - [ ] Produce a smaller PCB mainly with SMD components
- [x] Decide the models (transfer functions) for the motor thrust
- [x] Decide equations of motion for pitch/roll with real-world values
  - [ ] Calculate moments of inertia
- [ ] Decide equations of motion for yaw with real-world values
- [ ] Accomplish fast and responsive pitch, roll, yaw-rate regulators
- [ ] Estimate inertial and rotational position, velocity, and acceleration
- [ ] Be able to hover in place and fly patterns using GPS and IMU data
- [ ] Mission planning for doing missions defined by pre-determined routes
- [ ] Land and take off with little human interaction
- [ ] Refactor software to to build a more modular framework

## Primary topics
- [Design philosophy and 3D models](/README.md)
- [Modelling of drone](/docs/modelling_of_drone.md)
- [Autonomous drone regulation](docs/autonomous_drones.md)
- [Design of main PCB](/hardware/README.md)

![](images/readme_frontimage.jpg)

## Directory contents

- **3dparts** - 3D models used in the project as STEP and .obj files.
- **docs** - Documentation and learning material used in the project.
- **hardware** - Schematics, and PCB layouts as well as finished gerber files.
- **images** - Photos used in readme and some documentation.
- **software** - Software related to the project, including simulations, firmware, etc.
- **tests** - Results from test conducted during this project

## Disclaimer

I do not hold any responsibility for what another person does with the information or content provided in this repository. The repository is provided as-is with no guarentee agains personal or material damage. Everything here should be considered experimental, so please be responsible, but please lean as much as possible while you are browsing around. 