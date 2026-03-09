use serde::{Deserialize, Serialize};

/// A duration with nanosecond precision.
#[derive(
    Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize, Default,
)]
pub struct Duration {
    pub nanos: i64,
}

impl Duration {
    pub fn from_secs(secs: f64) -> Self {
        Self {
            nanos: (secs * 1_000_000_000.0) as i64,
        }
    }

    pub fn from_millis(millis: i64) -> Self {
        Self {
            nanos: millis * 1_000_000,
        }
    }

    pub fn from_nanos(nanos: i64) -> Self {
        Self { nanos }
    }

    pub fn zero() -> Self {
        Self { nanos: 0 }
    }

    pub fn as_secs_f64(&self) -> f64 {
        self.nanos as f64 / 1_000_000_000.0
    }

    pub fn as_millis(&self) -> i64 {
        self.nanos / 1_000_000
    }

    pub fn is_zero(&self) -> bool {
        self.nanos == 0
    }
}

impl core::ops::Add for Duration {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self {
            nanos: self.nanos + rhs.nanos,
        }
    }
}

impl core::ops::Sub for Duration {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        Self {
            nanos: self.nanos - rhs.nanos,
        }
    }
}

/// A timestamp with nanosecond precision.
#[derive(
    Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize, Default,
)]
pub struct Timestamp {
    pub nanos: u64,
}

impl Timestamp {
    pub fn from_nanos(nanos: u64) -> Self {
        Self { nanos }
    }

    pub fn from_secs(secs: f64) -> Self {
        Self {
            nanos: (secs * 1_000_000_000.0) as u64,
        }
    }

    pub fn zero() -> Self {
        Self { nanos: 0 }
    }

    #[cfg(feature = "std")]
    pub fn now() -> Self {
        let ts = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos() as u64;
        Self { nanos: ts }
    }

    pub fn as_secs_f64(&self) -> f64 {
        self.nanos as f64 / 1_000_000_000.0
    }

    pub fn elapsed_since(&self, earlier: &Self) -> Duration {
        Duration::from_nanos(self.nanos as i64 - earlier.nanos as i64)
    }
}

impl core::ops::Add<Duration> for Timestamp {
    type Output = Self;
    fn add(self, rhs: Duration) -> Self {
        Self {
            nanos: (self.nanos as i64 + rhs.nanos) as u64,
        }
    }
}

impl core::ops::Sub for Timestamp {
    type Output = Duration;
    fn sub(self, rhs: Self) -> Duration {
        Duration::from_nanos(self.nanos as i64 - rhs.nanos as i64)
    }
}

/// Rate helper for fixed-frequency loops.
#[derive(Debug, Clone, Copy)]
pub struct Rate {
    pub period: Duration,
}

impl Rate {
    pub fn from_hz(hz: f64) -> Self {
        Self {
            period: Duration::from_secs(1.0 / hz),
        }
    }

    pub fn hz(&self) -> f64 {
        1.0 / self.period.as_secs_f64()
    }

    pub fn period_secs(&self) -> f64 {
        self.period.as_secs_f64()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn duration_conversions() {
        let d = Duration::from_secs(1.5);
        assert!((d.as_secs_f64() - 1.5).abs() < 1e-6);
        assert_eq!(d.as_millis(), 1500);
    }

    #[test]
    fn duration_arithmetic() {
        let a = Duration::from_millis(100);
        let b = Duration::from_millis(200);
        assert_eq!((a + b).as_millis(), 300);
        assert_eq!((b - a).as_millis(), 100);
    }

    #[test]
    fn timestamp_elapsed() {
        let t1 = Timestamp::from_secs(1.0);
        let t2 = Timestamp::from_secs(2.5);
        let elapsed = t2.elapsed_since(&t1);
        assert!((elapsed.as_secs_f64() - 1.5).abs() < 1e-6);
    }

    #[test]
    fn timestamp_add_duration() {
        let t = Timestamp::from_secs(1.0);
        let d = Duration::from_secs(0.5);
        let t2 = t + d;
        assert!((t2.as_secs_f64() - 1.5).abs() < 1e-6);
    }

    #[test]
    fn rate() {
        let r = Rate::from_hz(10.0);
        assert!((r.hz() - 10.0).abs() < 1e-10);
        assert!((r.period_secs() - 0.1).abs() < 1e-10);
    }
}
