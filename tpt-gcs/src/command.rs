//! Operator command model and its on-wire encoding.
//!
//! Commands are sent from the GCS to the vehicle over the TPT-Link *command*
//! channel (see [`crate::link`]). The wire format is a tiny, allocation-free
//! discriminated union so it can be parsed in place on the vehicle.

use tpt_core::FlightMode;

/// A command issued by the operator.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Command {
    /// Arm the vehicle (FSM: Disarmed -> Armed).
    Arm,
    /// Disarm (stop all actuators).
    Disarm,
    /// Begin autonomous takeoff to the target altitude.
    Takeoff,
    /// Begin autonomous landing.
    Land,
    /// Fly to a local-frame waypoint (NED, meters; `yaw` in rad).
    SetWaypoint {
        x: f64,
        y: f64,
        z: f64,
        yaw: f64,
    },
    /// Request a specific flight mode.
    SetMode(FlightMode),
}

impl Command {
    /// Maximum encoded length in bytes.
    pub const MAX_LEN: usize = 1 + 16 + 1;

    /// Pack into `out`, returning the length written or `None` if too small.
    pub fn pack(&self, out: &mut [u8]) -> Option<usize> {
        match self {
            Command::Arm => {
                if out.len() < 1 {
                    return None;
                }
                out[0] = 0;
                Some(1)
            }
            Command::Disarm => {
                if out.len() < 1 {
                    return None;
                }
                out[0] = 1;
                Some(1)
            }
            Command::Takeoff => {
                if out.len() < 1 {
                    return None;
                }
                out[0] = 2;
                Some(1)
            }
            Command::Land => {
                if out.len() < 1 {
                    return None;
                }
                out[0] = 3;
                Some(1)
            }
            Command::SetWaypoint { x, y, z, yaw } => {
                if out.len() < 1 + 16 {
                    return None;
                }
                out[0] = 4;
                out[1..5].copy_from_slice(&(*x as f32).to_le_bytes());
                out[5..9].copy_from_slice(&(*y as f32).to_le_bytes());
                out[9..13].copy_from_slice(&(*z as f32).to_le_bytes());
                out[13..17].copy_from_slice(&(*yaw as f32).to_le_bytes());
                Some(1 + 16)
            }
            Command::SetMode(m) => {
                if out.len() < 2 {
                    return None;
                }
                out[0] = 5;
                out[1] = *m as u8;
                Some(2)
            }
        }
    }

    /// Parse a command from its wire bytes.
    pub fn unpack(buf: &[u8]) -> Option<Command> {
        let disc = *buf.first()?;
        match disc {
            0 => Some(Command::Arm),
            1 => Some(Command::Disarm),
            2 => Some(Command::Takeoff),
            3 => Some(Command::Land),
            4 => {
                if buf.len() < 1 + 16 {
                    return None;
                }
                let f = |i: usize| f32::from_le_bytes([buf[i], buf[i + 1], buf[i + 2], buf[i + 3]]);
                Some(Command::SetWaypoint {
                    x: f(1) as f64,
                    y: f(5) as f64,
                    z: f(9) as f64,
                    yaw: f(13) as f64,
                })
            }
            5 => {
                if buf.len() < 2 {
                    return None;
                }
                let m = match buf[1] {
                    0 => FlightMode::Disarmed,
                    1 => FlightMode::Armed,
                    2 => FlightMode::Takeoff,
                    3 => FlightMode::PositionHold,
                    4 => FlightMode::Land,
                    5 => FlightMode::Failsafe,
                    _ => return None,
                };
                Some(Command::SetMode(m))
            }
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn arm_disarm_round_trip() {
        for c in [Command::Arm, Command::Disarm, Command::Takeoff, Command::Land] {
            let mut buf = [0u8; Command::MAX_LEN];
            let n = c.pack(&mut buf).unwrap();
            assert_eq!(Command::unpack(&buf[..n]), Some(c));
        }
    }

    #[test]
    fn waypoint_round_trip() {
        let c = Command::SetWaypoint {
            x: 12.5,
            y: -3.0,
            z: -2.0,
            yaw: 1.57,
        };
        let mut buf = [0u8; Command::MAX_LEN];
        let n = c.pack(&mut buf).unwrap();
        // Wire format is f32; compare within f32 precision.
        match Command::unpack(&buf[..n]) {
            Some(Command::SetWaypoint { x, y, z, yaw }) => {
                assert!((x - 12.5).abs() < 1e-3);
                assert!((y + 3.0).abs() < 1e-3);
                assert!((z + 2.0).abs() < 1e-3);
                assert!((yaw - 1.57).abs() < 1e-3);
            }
            other => panic!("unexpected: {:?}", other),
        }
    }

    #[test]
    fn set_mode_round_trip() {
        let c = Command::SetMode(FlightMode::PositionHold);
        let mut buf = [0u8; Command::MAX_LEN];
        let n = c.pack(&mut buf).unwrap();
        assert_eq!(Command::unpack(&buf[..n]), Some(c));
    }
}
