//! PID controller with anti-windup (`spec.txt` §6.2).

use tpt_math::clamp;

/// Gains and limits for a [`Pid`] controller.
#[derive(Debug, Clone, Copy)]
pub struct PidConfig {
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
    /// Output clamp `[min, max]`.
    pub output_min: f64,
    pub output_max: f64,
    /// Integral term clamp `[-integral_limit, integral_limit]` (prevents
    /// windup at the source).
    pub integral_limit: f64,
    /// First-order derivative low-pass time constant (seconds). `0` disables.
    pub derivative_filter_tau: f64,
}

impl PidConfig {
    pub const fn new(kp: f64, ki: f64, kd: f64) -> Self {
        Self {
            kp,
            ki,
            kd,
            output_min: f64::NEG_INFINITY,
            output_max: f64::INFINITY,
            integral_limit: f64::INFINITY,
            derivative_filter_tau: 0.0,
        }
    }
}

/// Discrete PID controller with conditional-integration anti-windup.
///
/// Uses derivative-on-measurement (to avoid derivative kick) and clamps the
/// integrator; integration is suspended while the output is saturated and the
/// error would drive it further into saturation.
#[derive(Debug, Clone, Copy)]
pub struct Pid {
    cfg: PidConfig,
    integral: f64,
    prev_measurement: f64,
    filtered_deriv: f64,
    initialized: bool,
}

impl Pid {
    pub const fn new(cfg: PidConfig) -> Self {
        Self {
            cfg,
            integral: 0.0,
            prev_measurement: 0.0,
            filtered_deriv: 0.0,
            initialized: false,
        }
    }

    /// Reset all internal state.
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_measurement = 0.0;
        self.filtered_deriv = 0.0;
        self.initialized = false;
    }

    /// Update with `setpoint - measurement = error` and the signed measurement.
    ///
    /// Passing the measurement separately enables derivative-on-measurement.
    pub fn update(&mut self, error: f64, measurement: f64, dt: f64) -> f64 {
        if dt <= 0.0 {
            return clamp(self.kp() * error, self.cfg.output_min, self.cfg.output_max);
        }

        // Proportional.
        let p = self.cfg.kp * error;

        // Derivative on measurement (negative sign because d(error)/dt =
        // -d(measurement)/dt when setpoint is constant).
        let deriv = if self.initialized {
            (measurement - self.prev_measurement) / dt
        } else {
            0.0
        };
        let raw_d = -self.cfg.kd * deriv;
        // First-order low-pass on the derivative term.
        if self.cfg.derivative_filter_tau > 0.0 {
            let alpha = dt / (dt + self.cfg.derivative_filter_tau);
            self.filtered_deriv += alpha * (raw_d - self.filtered_deriv);
        } else {
            self.filtered_deriv = raw_d;
        }
        let d = self.filtered_deriv;

        // Tentative output to decide anti-windup.
        let tentative = clamp(p + self.integral + d, self.cfg.output_min, self.cfg.output_max);
        let saturated = tentative == self.cfg.output_min || tentative == self.cfg.output_max;

        // Conditional integration: only accumulate when not saturated, or when
        // the error would reduce the saturation.
        let would_worsen = (tentative == self.cfg.output_max && error > 0.0)
            || (tentative == self.cfg.output_min && error < 0.0);
        if !(saturated && would_worsen) {
            self.integral += self.cfg.ki * error * dt;
            self.integral = clamp(
                self.integral,
                -self.cfg.integral_limit,
                self.cfg.integral_limit,
            );
        }

        self.prev_measurement = measurement;
        self.initialized = true;

        clamp(p + self.integral + d, self.cfg.output_min, self.cfg.output_max)
    }

    fn kp(&self) -> f64 {
        self.cfg.kp
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn drives_to_setpoint() {
        let cfg = PidConfig::new(1.0, 0.5, 0.0);
        let mut pid = Pid::new(cfg);
        let mut y = 0.0f64;
        // Pure-integrator plant dy/dt = u; PI needs several seconds to settle.
        for _ in 0..20_000 {
            let out = pid.update(1.0 - y, y, 0.001);
            y += out * 0.001;
        }
        assert!((y - 1.0).abs() < 0.02, "settled at {y}");
    }

    #[test]
    fn anti_windup_clamps_integral() {
        let mut cfg = PidConfig::new(1.0, 10.0, 0.0);
        cfg.output_min = -1.0;
        cfg.output_max = 1.0;
        cfg.integral_limit = 5.0;
        let mut pid = Pid::new(cfg);
        // Large persistent error with saturated output.
        for _ in 0..10_000 {
            pid.update(10.0, 0.0, 0.001);
        }
        // Output must stay within clamp; integral cannot exceed its limit.
        assert!(pid.integral <= cfg.integral_limit + 1e-9);
        assert!(pid.integral >= -cfg.integral_limit - 1e-9);
    }
}
