use crate::functions;
use defmt::{self as _, Format};

#[derive(Debug, Format, Clone, Copy, Default, PartialEq)]
pub struct SbusCmd {
    pub thrust: f32,
    pub pitch: f32,
    pub roll: f32,
    pub yaw: f32,
    pub sw_e: TriSwitch,
    pub sw_b: TriSwitch,
    pub sw_c: TriSwitch,
    pub sw_f: TriSwitch,
}

/// Repackages the SBUS packet into a more usable format.
/// Returns `None` if the failsafe activated.
pub fn convert(packet: &sbus::SBusPacket) -> Option<SbusCmd> {
    {
        if packet.failsafe {
            return None;
        }

        Some(SbusCmd {
            thrust: sbus_range_thrust(packet.channels[2]),
            pitch: sbus_range(packet.channels[1], 0.005),
            roll: sbus_range(packet.channels[0], 0.005),
            yaw: sbus_range(packet.channels[3], 0.005),
            sw_e: sbus_switch(packet.channels[4]),
            sw_b: sbus_switch(packet.channels[5]),
            sw_c: sbus_switch(packet.channels[6]),
            sw_f: sbus_switch(packet.channels[7]),
        })
    }
}

fn sbus_range(x: u16, tol: f32) -> f32 {
    let y = functions::mapf(f32::from(x), 172., 1810., -1., 1.);
    if y > tol || y < -tol {
        y
    } else {
        0.0
    }
}

fn sbus_range_thrust(x: u16) -> f32 {
    functions::mapf( f32::from(x), 172., 1810., 0., 1.)
}

fn sbus_switch(x: u16) -> TriSwitch {
    if x < 736 {
        TriSwitch::Idle
    } else if x > 1248 {
        TriSwitch::Active
    } else {
        TriSwitch::Middle
    }
}

#[derive(Debug, Format, PartialEq, Clone, Copy)]
pub enum TriSwitch {
    Idle,
    Middle,
    Active,
}

#[allow(unused)]
impl TriSwitch {
    pub fn is_active(self) -> bool {
        self == TriSwitch::Active
    }

    pub fn is_middle(self) -> bool {
        self == TriSwitch::Middle
    }

    pub fn is_idle(self) -> bool {
        self == TriSwitch::Idle
    }
}

impl Default for TriSwitch {
    fn default() -> Self {
        Self::Idle
    }
}
