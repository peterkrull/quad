# Project quad

This git repository is a personal project of mine to make a well flying autonomous quad copter using cheap off-the-shelf components and good control theory and algorithms.

---

## Goals of this project
- [ ] Be able to hover in place and fly patterns using GPS and IMU data
- [ ] Take off, fly a mission and land with little human interaction
- [x] Make firmware modular for easy hardware and configuration changes

### Flight-capable hardware
- [x] Design and assemble the initial hardware capable of flight
- [ ] Produce a smaller PCB mainly with SMD components

### State estimation
- [ ] Learn about and document the workings of a Kalman filter
- [ ] Design a rudimentary kalman filter based on 6-DOF or 9-DOF IMUs
- [ ] Estimate translational and rotational position, velocity, and acceleration 

### Modelling of system dynamics
- [x] Decide the models (transfer functions) for the motor thrust
- [x] Decide equations of motion for pitch/roll with real-world values
  - [x] Calculate moments of inertia
- [ ] Decide equations of motion for yaw with real-world values
  - [ ] Calculate moment of inertia

### Controller design
- [ ] Accomplish fast and responsive pitch, roll and yaw regulators
  - [x] Pitch/roll regulator
  - [ ] Yaw regulator
- [ ] Automatically calculate controller gains based on drone hardware

### Flight planner
- [ ] Develop architechture for flight planner (g-code like protocol?)
- [ ] Receive flight data from quadcopter and send commands
- [ ] Create GUI for easy interaction with drone
  - [ ] Use world map for mission planning interface

---

## Primary topics
- [Design philosophy and 3D models](/README.md)
- [Modelling of drone](/docs/README.md)
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