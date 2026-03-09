use super::Vector3;
use serde::{Deserialize, Serialize};

/// 2D twist: linear and angular velocity for planar robots.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
#[repr(C)]
pub struct Twist2D {
    /// Linear velocity in m/s.
    pub v: f64,
    /// Angular velocity in rad/s.
    pub w: f64,
}

impl Twist2D {
    pub fn new(v: f64, w: f64) -> Self {
        Self { v, w }
    }
    pub fn zero() -> Self {
        Self { v: 0.0, w: 0.0 }
    }
    pub fn is_stopped(&self) -> bool {
        self.v.abs() < 1e-6 && self.w.abs() < 1e-6
    }
    pub fn is_zero(&self) -> bool {
        self.is_stopped()
    }
    pub fn speed(&self) -> f64 {
        self.v.abs()
    }
    pub fn clamp(&self, max_v: f64, max_w: f64) -> Self {
        Self {
            v: self.v.clamp(-max_v, max_v),
            w: self.w.clamp(-max_w, max_w),
        }
    }
}

/// 3D twist: 6-DOF velocity (linear + angular).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
#[repr(C)]
pub struct Twist3D {
    pub linear: Vector3,
    pub angular: Vector3,
}

impl Twist3D {
    pub fn new(linear: Vector3, angular: Vector3) -> Self {
        Self { linear, angular }
    }
    pub fn zero() -> Self {
        Self {
            linear: Vector3::zero(),
            angular: Vector3::zero(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn twist2d_stopped() {
        assert!(Twist2D::zero().is_stopped());
        assert!(!Twist2D::new(0.1, 0.0).is_stopped());
    }
}
