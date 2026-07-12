//! egui-based desktop Ground Control Station window.
//!
//! Enabled with `--features gui`. Shares the exact same [`Telemetry`] /
//! [`Command`] / [`link`](crate::link) model as the console runner; it only
//! replaces the terminal rendering with an interactive panel. In a real
//! deployment a background thread would receive TPT-Link frames and call
//! [`GcsApp::ingest`]; here the panel exposes the send buttons and shows the
//! latest telemetry.

use crate::command::Command;
use crate::telemetry::Telemetry;

/// The egui GCS application state.
pub struct GcsApp {
    latest: Telemetry,
    wp_x: String,
    wp_y: String,
    wp_z: String,
    wp_yaw: String,
    /// Pending command (set by button handlers; the host loop transmits it).
    pending: Option<Command>,
    log: String,
}

impl GcsApp {
    /// Construct the app (called by `eframe` via `Box::new`).
    pub fn new(_cc: &eframe::CreationContext<'_>) -> Self {
        Self {
            latest: Telemetry::zeroed(),
            wp_x: "0.0".into(),
            wp_y: "0.0".into(),
            wp_z: "-2.0".into(),
            wp_yaw: "0.0".into(),
            pending: None,
            log: String::new(),
        }
    }

    /// Feed the latest telemetry sample from the link layer.
    pub fn ingest(&mut self, t: Telemetry) {
        self.latest = t;
    }

    /// Take the next pending command to transmit (consumes it).
    pub fn take_pending(&mut self) -> Option<Command> {
        self.pending.take()
    }

    fn log_line(&mut self, s: &str) {
        self.log.push_str(s);
        self.log.push('\n');
    }
}

impl eframe::App for GcsApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        egui::CentralPanel::default().show(ctx, |ui| {
            ui.heading("TPT Flight Control — GCS");
            ui.separator();

            // Telemetry readout.
            let t = &self.latest;
            ui.label(format!("Mode: {:?}", t.mode));
            ui.label(format!("Nav : {:?}", t.nav_mode));
            ui.label(format!(
                "Attitude: roll {:.1}° pitch {:.1}° yaw {:.1}°",
                t.roll.to_degrees(),
                t.pitch.to_degrees(),
                t.yaw.to_degrees()
            ));
            ui.label(format!(
                "Position: N {:.2} E {:.2} alt {:.2} m",
                t.position.x,
                t.position.y,
                -t.position.z
            ));
            ui.label(format!("Battery: {:.1}%", t.battery * 100.0));
            ui.separator();

            // Command buttons.
            ui.horizontal(|ui| {
                if ui.button("Arm").clicked() {
                    self.pending = Some(Command::Arm);
                    self.log_line("arm");
                }
                if ui.button("Disarm").clicked() {
                    self.pending = Some(Command::Disarm);
                    self.log_line("disarm");
                }
                if ui.button("Takeoff").clicked() {
                    self.pending = Some(Command::Takeoff);
                    self.log_line("takeoff");
                }
                if ui.button("Land").clicked() {
                    self.pending = Some(Command::Land);
                    self.log_line("land");
                }
            });

            // Waypoint entry.
            ui.separator();
            ui.label("Waypoint (NED m, yaw deg):");
            ui.horizontal(|ui| {
                ui.text_edit_singleline(&mut self.wp_x);
                ui.text_edit_singleline(&mut self.wp_y);
                ui.text_edit_singleline(&mut self.wp_z);
                ui.text_edit_singleline(&mut self.wp_yaw);
                if ui.button("Send WP").clicked() {
                    if let (Ok(x), Ok(y), Ok(z), Ok(yaw)) = (
                        self.wp_x.parse::<f64>(),
                        self.wp_y.parse::<f64>(),
                        self.wp_z.parse::<f64>(),
                        self.wp_yaw.parse::<f64>(),
                    ) {
                        self.pending = Some(Command::SetWaypoint {
                            x,
                            y,
                            z,
                            yaw: yaw.to_radians(),
                        });
                        self.log_line("waypoint sent");
                    } else {
                        self.log_line("invalid waypoint");
                    }
                }
            });

            ui.separator();
            ui.label("Log:");
            ui.monospace(&self.log);
        });
    }
}
