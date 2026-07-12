//! Dependency-free console Ground Control Station.
//!
//! Connects to the vehicle over plain std UDP, parses TPT-Link telemetry
//! frames, prints the [`dashboard`], and sends operator [`Command`]s typed at
//! the prompt. This is the runnable, always-buildable GCS; the `gui` feature
//! adds the egui window on top of the same link/telemetry model.

use crate::command::Command;
use crate::dashboard;
use crate::link;
use crate::telemetry::Telemetry;
use std::io::{self, Write};
use std::net::UdpSocket;
use std::time::Duration;

/// A console GCS bound to a local UDP socket, talking to a remote endpoint.
pub struct ConsoleGcs {
    socket: UdpSocket,
    remote: String,
    recv_buf: [u8; 256],
    seq: u16,
    last: Option<Telemetry>,
}

impl ConsoleGcs {
    /// Bind `bind_addr` and target `remote_addr` (e.g. `127.0.0.1:14550`).
    pub fn new(bind_addr: &str, remote_addr: &str) -> io::Result<Self> {
        let socket = UdpSocket::bind(bind_addr)?;
        socket.set_read_timeout(Some(Duration::from_millis(10)))?;
        Ok(Self {
            socket,
            remote: remote_addr.to_string(),
            recv_buf: [0u8; 256],
            seq: 0,
            last: None,
        })
    }

    /// Receive and parse the next telemetry frame (non-blocking).
    pub fn poll(&mut self) -> Option<Telemetry> {
        if let Ok(n) = self.socket.recv(&mut self.recv_buf) {
            if let Some(t) = link::parse_telemetry(&self.recv_buf[..n]) {
                self.last = Some(t);
                return Some(t);
            }
        }
        self.last
    }

    /// Send a command to the vehicle.
    pub fn send(&mut self, cmd: &Command) -> io::Result<()> {
        let mut buf = [0u8; 128];
        let n = link::serialize_command(cmd, self.seq, &mut buf)
            .ok_or_else(|| io::Error::new(io::ErrorKind::Other, "command too large"))?;
        self.seq = self.seq.wrapping_add(1);
        self.socket.send_to(&buf[..n], &self.remote)?;
        Ok(())
    }

    /// Run the interactive loop: print telemetry, read typed commands.
    ///
    /// Commands: `arm`, `disarm`, `takeoff`, `land`, `wp <x> <y> <z> <yaw>`,
    /// `mode <disarmed|armed|takeoff|hold|land|failsafe>`, `quit`.
    pub fn run(&mut self) -> io::Result<()> {
        let stdin = io::stdin();
        println!("TPT GCS console — type `help` for commands, `quit` to exit.");
        loop {
            if let Some(t) = self.poll() {
                println!("{}", dashboard::render(&t));
            }
            print!("gcs> ");
            io::stdout().flush()?;
            let mut line = String::new();
            if stdin.read_line(&mut line)? == 0 {
                break; // EOF
            }
            let line = line.trim();
            if line.is_empty() {
                continue;
            }
            match parse_command_line(line) {
                Some(cmd) => {
                    self.send(&cmd)?;
                    println!("sent: {:?}", cmd);
                }
                None if line == "quit" => break,
                None if line == "help" => print_help(),
                None => println!("unknown command: {}", line),
            }
        }
        Ok(())
    }
}

/// Parse a console command line into a [`Command`].
pub fn parse_command_line(line: &str) -> Option<Command> {
    let mut parts = line.split_whitespace();
    match parts.next()? {
        "arm" => Some(Command::Arm),
        "disarm" => Some(Command::Disarm),
        "takeoff" => Some(Command::Takeoff),
        "land" => Some(Command::Land),
        "wp" => {
            let x: f64 = parts.next()?.parse().ok()?;
            let y: f64 = parts.next()?.parse().ok()?;
            let z: f64 = parts.next()?.parse().ok()?;
            let yaw: f64 = parts.next()?.parse().ok()?;
            Some(Command::SetWaypoint { x, y, z, yaw })
        }
        "mode" => {
            let m = match parts.next()? {
                "disarmed" => tpt_core::FlightMode::Disarmed,
                "armed" => tpt_core::FlightMode::Armed,
                "takeoff" => tpt_core::FlightMode::Takeoff,
                "hold" => tpt_core::FlightMode::PositionHold,
                "land" => tpt_core::FlightMode::Land,
                "failsafe" => tpt_core::FlightMode::Failsafe,
                _ => return None,
            };
            Some(Command::SetMode(m))
        }
        _ => None,
    }
}

fn print_help() {
    println!(
        "commands:\n  arm | disarm | takeoff | land\n  wp <x> <y> <z> <yaw>\n  mode <disarmed|armed|takeoff|hold|land|failsafe>\n  quit"
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_command_lines() {
        assert_eq!(parse_command_line("arm"), Some(Command::Arm));
        assert_eq!(parse_command_line("land"), Some(Command::Land));
        assert_eq!(
            parse_command_line("wp 1 2 -3 0.5"),
            Some(Command::SetWaypoint {
                x: 1.0,
                y: 2.0,
                z: -3.0,
                yaw: 0.5
            })
        );
        assert_eq!(
            parse_command_line("mode hold"),
            Some(Command::SetMode(tpt_core::FlightMode::PositionHold))
        );
        assert_eq!(parse_command_line("bogus"), None);
    }
}
