use serde::{Deserialize, Serialize};

/// 2D pose: position (x, y) + heading (theta).
///
/// No timestamp — timestamps are an IPC concern (HORUS adds them).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[repr(C)]
pub struct Pose2D {
    pub x: f64,
    pub y: f64,
    pub theta: f64,
}

impl Default for Pose2D {
    fn default() -> Self {
        Self::identity()
    }
}

impl Pose2D {
    pub fn new(x: f64, y: f64, theta: f64) -> Self {
        Self { x, y, theta }
    }

    pub fn identity() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
        }
    }

    /// Compose two poses: self * other (apply other in self's frame).
    pub fn compose(&self, other: &Self) -> Self {
        let cos_t = self.theta.cos();
        let sin_t = self.theta.sin();
        Self {
            x: self.x + cos_t * other.x - sin_t * other.y,
            y: self.y + sin_t * other.x + cos_t * other.y,
            theta: normalize_angle(self.theta + other.theta),
        }
    }

    /// Inverse of this pose.
    pub fn inverse(&self) -> Self {
        let cos_t = self.theta.cos();
        let sin_t = self.theta.sin();
        Self {
            x: -(cos_t * self.x + sin_t * self.y),
            y: -(-sin_t * self.x + cos_t * self.y),
            theta: normalize_angle(-self.theta),
        }
    }

    /// Euclidean distance to another pose (ignoring heading).
    pub fn distance_to(&self, other: &Self) -> f64 {
        let dx = other.x - self.x;
        let dy = other.y - self.y;
        (dx * dx + dy * dy).sqrt()
    }

    /// Angle from this pose to another point.
    pub fn angle_to(&self, other: &Self) -> f64 {
        let dx = other.x - self.x;
        let dy = other.y - self.y;
        dy.atan2(dx)
    }

    /// Linear interpolation between two poses.
    pub fn interpolate(&self, other: &Self, t: f64) -> Self {
        let t = t.clamp(0.0, 1.0);
        Self {
            x: self.x + t * (other.x - self.x),
            y: self.y + t * (other.y - self.y),
            theta: normalize_angle(self.theta + t * normalize_angle(other.theta - self.theta)),
        }
    }

    /// Convert to nalgebra Isometry2.
    #[cfg(feature = "std")]
    pub fn to_isometry(&self) -> nalgebra::Isometry2<f64> {
        nalgebra::Isometry2::new(nalgebra::Vector2::new(self.x, self.y), self.theta)
    }

    /// Create from nalgebra Isometry2.
    #[cfg(feature = "std")]
    pub fn from_isometry(iso: &nalgebra::Isometry2<f64>) -> Self {
        Self {
            x: iso.translation.x,
            y: iso.translation.y,
            theta: iso.rotation.angle(),
        }
    }
}

/// Normalize angle to [-pi, pi].
pub fn normalize_angle(angle: f64) -> f64 {
    let mut a = angle % (2.0 * core::f64::consts::PI);
    if a > core::f64::consts::PI {
        a -= 2.0 * core::f64::consts::PI;
    } else if a < -core::f64::consts::PI {
        a += 2.0 * core::f64::consts::PI;
    }
    a
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn identity() {
        let p = Pose2D::identity();
        assert_eq!(p.x, 0.0);
        assert_eq!(p.y, 0.0);
        assert_eq!(p.theta, 0.0);
    }

    #[test]
    fn compose_inverse_roundtrip() {
        let a = Pose2D::new(1.0, 2.0, 0.5);
        let b = Pose2D::new(3.0, -1.0, 1.0);
        let c = a.compose(&b);
        let b_recovered = a.inverse().compose(&c);
        assert!((b_recovered.x - b.x).abs() < 1e-10);
        assert!((b_recovered.y - b.y).abs() < 1e-10);
        assert!((b_recovered.theta - b.theta).abs() < 1e-10);
    }

    #[test]
    fn distance() {
        let a = Pose2D::new(0.0, 0.0, 0.0);
        let b = Pose2D::new(3.0, 4.0, 0.0);
        assert!((a.distance_to(&b) - 5.0).abs() < 1e-10);
    }

    #[test]
    fn interpolate_endpoints() {
        let a = Pose2D::new(0.0, 0.0, 0.0);
        let b = Pose2D::new(10.0, 10.0, 1.0);
        let at_0 = a.interpolate(&b, 0.0);
        let at_1 = a.interpolate(&b, 1.0);
        assert!((at_0.x - a.x).abs() < 1e-10);
        assert!((at_1.x - b.x).abs() < 1e-10);
    }

    #[test]
    fn serde_roundtrip() {
        let p = Pose2D::new(1.5, -2.3, 0.78);
        let json = serde_json::to_string(&p).unwrap();
        let p2: Pose2D = serde_json::from_str(&json).unwrap();
        assert_eq!(p, p2);
    }
}
