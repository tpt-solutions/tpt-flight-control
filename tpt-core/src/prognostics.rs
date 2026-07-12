//! Predictive health / prognostics (`spec.txt` §16.3, resilience roadmap).
//!
//! Fixed-capacity trend buffers record recent samples of battery and motor
//! telemetry so the flight controller can estimate *remaining useful life*
//! (RUL) and a scalar health index without any heap allocation. The buffers
//! are ring buffers of compile-time capacity `N`, matching the `no_std` /
//! allocation-free discipline of the rest of `tpt-core`.
//!
//! These types consume the extended [`tpt_abstractions::actuators::Motor`] and
//! [`tpt_abstractions::os::PowerSystem`] trait fields (state of charge, cell
//! voltage, temperature, rpm, load) and feed the `tptlink` `HealthReport`
//! message on [`tpt_protocols::tptlink::Channel::Health`].

use tpt_abstractions::actuators::Motor as MotorTrait;
use tpt_abstractions::os::PowerSystem as PowerSystemTrait;

/// A fixed-capacity ring buffer of `f64` samples for trending.
///
/// Capacity `N` is the maximum number of retained samples. When full, the
/// oldest sample is overwritten. All statistics (mean, min, max, slope) are
/// computed over the retained window in `O(N)` with no allocation.
#[derive(Debug, Clone, Copy)]
pub struct TrendBuffer<const N: usize> {
    buf: [f64; N],
    /// Index of the next write slot.
    head: usize,
    /// Number of valid samples (<= N).
    count: usize,
}

impl<const N: usize> TrendBuffer<N> {
    /// Create an empty buffer. `N` must be >= 1.
    pub const fn new() -> Self {
        Self {
            buf: [0.0f64; N],
            head: 0,
            count: 0,
        }
    }

    /// Number of valid samples currently retained.
    pub const fn len(&self) -> usize {
        self.count
    }

    /// Whether no samples have been recorded yet.
    pub const fn is_empty(&self) -> bool {
        self.count == 0
    }

    /// Whether the buffer has wrapped at least once (full window retained).
    pub const fn is_full(&self) -> bool {
        self.count == N
    }

    /// Push a sample, overwriting the oldest if the buffer is full.
    pub fn push(&mut self, v: f64) {
        self.buf[self.head] = v;
        self.head = (self.head + 1) % N;
        if self.count < N {
            self.count += 1;
        }
    }

    /// The most recently pushed sample, or `None` if empty.
    pub fn latest(&self) -> Option<f64> {
        if self.count == 0 {
            return None;
        }
        let idx = if self.head == 0 { N - 1 } else { self.head - 1 };
        Some(self.buf[idx])
    }

    /// Mean of the retained samples (0 if empty).
    pub fn mean(&self) -> f64 {
        if self.count == 0 {
            return 0.0;
        }
        let mut s = 0.0f64;
        for i in 0..self.count {
            s += self.buf[i];
        }
        s / self.count as f64
    }

    /// Minimum retained sample.
    pub fn min(&self) -> f64 {
        let mut m = f64::INFINITY;
        for i in 0..self.count {
            if self.buf[i] < m {
                m = self.buf[i];
            }
        }
        m
    }

    /// Maximum retained sample.
    pub fn max(&self) -> f64 {
        let mut m = f64::NEG_INFINITY;
        for i in 0..self.count {
            if self.buf[i] > m {
                m = self.buf[i];
            }
        }
        m
    }

    /// Least-squares slope of the trend vs sample index (units/sec-sample).
    ///
    /// A negative slope means the quantity is degrading over time. Returns
    /// `None` if fewer than two samples are available.
    pub fn slope(&self) -> Option<f64> {
        if self.count < 2 {
            return None;
        }
        let n = self.count as f64;
        let mut sx = 0.0f64;
        let mut sy = 0.0f64;
        let mut sxx = 0.0f64;
        let mut sxy = 0.0f64;
        for (k, i) in (0..self.count).enumerate() {
            let x = k as f64;
            let y = self.buf[i];
            sx += x;
            sy += y;
            sxx += x * x;
            sxy += x * y;
        }
        let denom = n * sxx - sx * sx;
        if denom.abs() < 1e-12 {
            return None;
        }
        Some((n * sxy - sx * sy) / denom)
    }
}

impl<const N: usize> Default for TrendBuffer<N> {
    fn default() -> Self {
        Self::new()
    }
}

/// Battery health tracker (`spec.txt` §16.3).
///
/// Maintains trend buffers of state-of-charge, minimum cell voltage, and pack
/// temperature, plus a running estimate of accumulated throughput (A·h) used to
/// model capacity fade. Produces a RUL-style estimate from the SoC discharge
/// trend and a scalar health index in `[0, 1]`.
#[derive(Debug, Clone, Copy)]
pub struct BatteryHealth<const N: usize> {
    soc: TrendBuffer<N>,
    cell_v: TrendBuffer<N>,
    temp: TrendBuffer<N>,
    /// Nominal pack capacity (A·h).
    nominal_capacity_ah: f64,
    /// Accumulated discharged throughput (A·h) since last reset.
    throughput_ah: f64,
}

impl<const N: usize> BatteryHealth<N> {
    /// Create a tracker for a pack of the given nominal capacity (A·h).
    pub const fn new(nominal_capacity_ah: f64) -> Self {
        Self {
            soc: TrendBuffer::new(),
            cell_v: TrendBuffer::new(),
            temp: TrendBuffer::new(),
            nominal_capacity_ah,
            throughput_ah: 0.0,
        }
    }

    /// Record one sample. `load_a` is the instantaneous discharge current (A);
    /// `dt` is the elapsed time (s).
    pub fn update(&mut self, soc: f64, min_cell_v: f64, temp_c: f64, load_a: f64, dt: f64) {
        if dt <= 0.0 {
            return;
        }
        self.soc.push(soc);
        self.cell_v.push(min_cell_v);
        self.temp.push(temp_c);
        self.throughput_ah += (load_a.abs() * dt) / 3600.0;
    }

    /// Latest state of charge in `[0, 1]`.
    pub fn state_of_charge(&self) -> f64 {
        self.soc.latest().unwrap_or(0.0)
    }

    /// Latest minimum cell voltage (V).
    pub fn min_cell_voltage(&self) -> f64 {
        self.cell_v.latest().unwrap_or(0.0)
    }

    /// Latest pack temperature (°C).
    pub fn pack_temperature_c(&self) -> f64 {
        self.temp.latest().unwrap_or(0.0)
    }

    /// Estimated capacity fade in `[0, 1]` (0 = fully faded). Modelled as the
    /// fraction of nominal throughput not yet consumed, with a conservative
    /// 1.5x wear multiplier (high-C / high-temperature operation ages cells
    /// faster than simple throughput would suggest).
    pub fn capacity_retention(&self) -> f64 {
        if self.nominal_capacity_ah <= 0.0 {
            return 0.0;
        }
        let used = (self.throughput_ah * 1.5) / self.nominal_capacity_ah;
        (1.0 - used).clamp(0.0, 1.0)
    }

    /// RUL-style estimate: seconds until SoC reaches 0 at the current discharge
    /// trend, or `None` if the trend is not established or non-degrading.
    pub fn estimate_rul_seconds(&self) -> Option<f64> {
        let slope = self.soc.slope()?;
        let soc = self.state_of_charge();
        if slope >= 0.0 || soc <= 0.0 {
            return None;
        }
        Some(soc / -slope)
    }

    /// Scalar health index in `[0, 1]` combining capacity retention and the
    /// worst recent cell voltage (a cell below 3.0 V is treated as degrading).
    pub fn health_index(&self) -> f64 {
        let cap = self.capacity_retention();
        let v = self.cell_v.min();
        let volt_ok = if v <= 0.0 {
            1.0
        } else if v < 3.0 {
            ((v - 2.5) / 0.5).clamp(0.0, 1.0)
        } else {
            1.0
        };
        (cap * 0.6 + volt_ok * 0.4).clamp(0.0, 1.0)
    }

    /// Build this tracker's view from any [`PowerSystem`] implementer using the
    /// extended prognostics trait fields.
    pub fn ingest<PS: PowerSystemTrait>(&mut self, ps: &PS, load_a: f64, dt: f64) {
        self.update(
            ps.state_of_charge(),
            ps.min_cell_voltage_v(),
            ps.pack_temperature_c(),
            load_a,
            dt,
        );
    }
}

/// Motor health tracker (`spec.txt` §16.3).
///
/// Tracks temperature, normalized load, and rpm trends, plus accumulated
/// operating time, and estimates RUL from a conservative wear model where high
/// temperature and high load accelerate degradation.
#[derive(Debug, Clone, Copy)]
pub struct MotorHealth<const N: usize> {
    temp: TrendBuffer<N>,
    load: TrendBuffer<N>,
    rpm: TrendBuffer<N>,
    /// Accumulated operating time (s).
    operating_s: f64,
}

impl<const N: usize> MotorHealth<N> {
    /// Create an empty tracker.
    pub const fn new() -> Self {
        Self {
            temp: TrendBuffer::new(),
            load: TrendBuffer::new(),
            rpm: TrendBuffer::new(),
            operating_s: 0.0,
        }
    }

    /// Record one sample. `dt` is the elapsed time (s).
    pub fn update(&mut self, temp_c: f64, load: f64, rpm: f64, dt: f64) {
        if dt <= 0.0 {
            return;
        }
        self.temp.push(temp_c);
        self.load.push(load);
        self.rpm.push(rpm);
        self.operating_s += dt;
    }

    /// Latest winding/electronics temperature (°C).
    pub fn temperature_c(&self) -> f64 {
        self.temp.latest().unwrap_or(0.0)
    }

    /// Latest normalized load in `[0, 1]`.
    pub fn load(&self) -> f64 {
        self.load.latest().unwrap_or(0.0)
    }

    /// Latest rotational speed (rpm).
    pub fn rpm(&self) -> f64 {
        self.rpm.latest().unwrap_or(0.0)
    }

    /// Accumulated operating time (s).
    pub const fn operating_seconds(&self) -> f64 {
        self.operating_s
    }

    /// Instantaneous wear rate in `[0, 1]` per hour, combining a temperature
    /// factor (Arrhenius-style exponential above 60 °C) and a load factor.
    pub fn wear_rate_per_hour(&self) -> f64 {
        let temp = self.temp.mean().max(20.0);
        let load = self.load.mean().clamp(0.0, 1.0);
        // Reference life at 20 °C / 50% load ~ 10,000 h.
        let temp_factor = libm::exp((temp - 20.0) / 20.0);
        let load_factor = 0.3 + load;
        (temp_factor * load_factor) / 10_000.0
    }

    /// RUL-style estimate: remaining hours before wear reaches the end-of-life
    /// budget (1.0 cumulative wear). Returns `None` until some operating time
    /// has accrued.
    pub fn estimate_rul_hours(&self) -> Option<f64> {
        if self.operating_s <= 0.0 {
            return None;
        }
        let rate = self.wear_rate_per_hour();
        if rate <= 0.0 {
            return None;
        }
        let consumed = rate * (self.operating_s / 3600.0);
        let remaining = (1.0 - consumed).max(0.0) / rate;
        Some(remaining)
    }

    /// Scalar health index in `[0, 1]` (1 = new).
    pub fn health_index(&self) -> f64 {
        if self.operating_s <= 0.0 {
            return 1.0;
        }
        let rate = self.wear_rate_per_hour();
        let consumed = (rate * self.operating_s / 3600.0).clamp(0.0, 1.0);
        (1.0 - consumed).clamp(0.0, 1.0)
    }

    /// Build this tracker's view from any [`Motor`] implementer using the
    /// extended prognostics trait fields.
    pub fn ingest<M: MotorTrait>(&mut self, m: &M, dt: f64) {
        let load = m.load().clamp(0.0, 1.0);
        // Approximate a temperature-aware load current from normalized load.
        let load_a = load * 40.0;
        self.update(m.temperature_c(), load, m.rpm(), dt);
        let _ = load_a;
    }
}

impl<const N: usize> Default for MotorHealth<N> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn trend_buffer_ring_overwrites_oldest() {
        let mut b: TrendBuffer<3> = TrendBuffer::new();
        assert!(b.is_empty());
        b.push(1.0);
        b.push(2.0);
        b.push(3.0);
        assert!(b.is_full());
        assert_eq!(b.latest(), Some(3.0));
        assert_eq!(b.mean(), 2.0);
        // Overwrite oldest (1.0).
        b.push(4.0);
        assert_eq!(b.mean(), 3.0); // (2+3+4)/3
        assert_eq!(b.min(), 2.0);
        assert_eq!(b.max(), 4.0);
    }

    #[test]
    fn trend_slope_detects_degradation() {
        let mut b: TrendBuffer<8> = TrendBuffer::new();
        for v in [1.0, 0.9, 0.8, 0.7, 0.6, 0.5] {
            b.push(v);
        }
        let s = b.slope().unwrap();
        assert!(s < 0.0, "slope {s}");
    }

    #[test]
    fn battery_rul_from_discharge_trend() {
        let mut h: BatteryHealth<16> = BatteryHealth::new(5.0);
        // Discharge 1% per sample; slope = -0.01 / sample.
        let mut soc = 0.8;
        for _ in 0..10 {
            h.update(soc, 3.7, 25.0, 10.0, 1.0);
            soc -= 0.01;
        }
        let rul = h.estimate_rul_seconds().unwrap();
        // ~0.7 remaining / 0.01 per sample ~= 70 samples.
        assert!(rul > 50.0 && rul < 90.0, "rul {rul}");
        assert!(h.health_index() > 0.0);
    }

    #[test]
    fn battery_capacity_fades_with_throughput() {
        let mut h: BatteryHealth<4> = BatteryHealth::new(1.0);
        // 1.5x wear * 2 A·h throughput / 1 A·h nominal = 3.0 -> clamped to 0.
        h.update(0.5, 3.6, 30.0, 20.0, 360.0);
        assert_eq!(h.capacity_retention(), 0.0);
    }

    #[test]
    fn motor_wear_and_rul() {
        let mut h: MotorHealth<16> = MotorHealth::new();
        // Hot, heavily loaded operation for 1 hour.
        for _ in 0..3600 {
            h.update(90.0, 0.9, 8000.0, 1.0);
        }
        let rate = h.wear_rate_per_hour();
        assert!(rate > 0.0, "wear {rate}");
        let rul = h.estimate_rul_hours().unwrap();
        assert!(rul < 10_000.0, "rul {rul}");
        assert!(h.health_index() < 1.0);
    }

    #[test]
    fn motor_cold_idle_stays_healthy() {
        let mut h: MotorHealth<8> = MotorHealth::new();
        for _ in 0..100 {
            h.update(25.0, 0.1, 2000.0, 1.0);
        }
        assert!(h.health_index() > 0.99);
    }
}
