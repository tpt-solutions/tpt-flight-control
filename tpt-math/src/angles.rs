//! Angle utilities (all radians).

/// Wrap an angle into `(-pi, pi]`.
#[inline]
pub fn wrap_pi(mut a: f64) -> f64 {
    const TWO_PI: f64 = 2.0 * core::f64::consts::PI;
    // Reduce to (-2pi, 2pi] first to stay stable for large inputs.
    a %= TWO_PI;
    if a > core::f64::consts::PI {
        a -= TWO_PI;
    } else if a <= -core::f64::consts::PI {
        a += TWO_PI;
    }
    a
}

/// Wrap an angle into `[0, 2pi)`.
#[inline]
pub fn wrap_2pi(mut a: f64) -> f64 {
    const TWO_PI: f64 = 2.0 * core::f64::consts::PI;
    a %= TWO_PI;
    if a < 0.0 {
        a += TWO_PI;
    }
    a
}

/// Limit a yaw/heading command to `+-limit` radians after wrapping.
#[inline]
pub fn limit_symmetric(a: f64, limit: f64) -> f64 {
    let w = wrap_pi(a);
    if w > limit {
        limit
    } else if w < -limit {
        -limit
    } else {
        w
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wrap_pi_basic() {
        assert!((wrap_pi(0.0)).abs() < 1e-12);
        // 3π wraps to +π (range is (-π, π]).
        assert!((wrap_pi(3.0 * core::f64::consts::PI) - core::f64::consts::PI).abs() < 1e-12);
        assert!((wrap_pi(-3.0 * core::f64::consts::PI) - core::f64::consts::PI).abs() < 1e-12);
    }

    #[test]
    fn wrap_2pi_basic() {
        assert!((wrap_2pi(0.0)).abs() < 1e-12);
        assert!((wrap_2pi(-core::f64::consts::PI) - core::f64::consts::PI).abs() < 1e-12);
    }

    #[test]
    fn limit_symmetric_clips() {
        assert!((limit_symmetric(0.1, 0.2) - 0.1).abs() < 1e-12);
        assert!((limit_symmetric(1.0, 0.2) - 0.2).abs() < 1e-12);
    }
}
