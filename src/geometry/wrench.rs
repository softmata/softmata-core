use super::Vector3;
use crate::sensor::traits::WrenchData;
use core::ops::{Add, Neg};
use serde::{Deserialize, Serialize};

/// 6-DOF wrench: force + torque.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
#[repr(C)]
pub struct Wrench {
    pub force: Vector3,
    pub torque: Vector3,
}

impl Wrench {
    pub fn new(force: Vector3, torque: Vector3) -> Self {
        Self { force, torque }
    }

    pub fn zero() -> Self {
        Self {
            force: Vector3::zero(),
            torque: Vector3::zero(),
        }
    }

    pub fn force_only(fx: f64, fy: f64, fz: f64) -> Self {
        Self {
            force: Vector3::new(fx, fy, fz),
            torque: Vector3::zero(),
        }
    }

    pub fn torque_only(tx: f64, ty: f64, tz: f64) -> Self {
        Self {
            force: Vector3::zero(),
            torque: Vector3::new(tx, ty, tz),
        }
    }

    pub fn force_magnitude(&self) -> f64 {
        self.force.magnitude()
    }

    pub fn torque_magnitude(&self) -> f64 {
        self.torque.magnitude()
    }

    pub fn to_array(&self) -> [f64; 6] {
        [
            self.force.x,
            self.force.y,
            self.force.z,
            self.torque.x,
            self.torque.y,
            self.torque.z,
        ]
    }

    pub fn from_slice(s: &[f64]) -> Self {
        assert!(s.len() >= 6, "Wrench requires at least 6 elements");
        Self {
            force: Vector3::new(s[0], s[1], s[2]),
            torque: Vector3::new(s[3], s[4], s[5]),
        }
    }
}

impl Add for Wrench {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self {
            force: self.force + rhs.force,
            torque: self.torque + rhs.torque,
        }
    }
}

impl Neg for Wrench {
    type Output = Self;
    fn neg(self) -> Self {
        Self {
            force: -self.force,
            torque: -self.torque,
        }
    }
}

impl WrenchData for Wrench {
    fn force(&self) -> [f64; 3] {
        [self.force.x, self.force.y, self.force.z]
    }
    fn torque(&self) -> [f64; 3] {
        [self.torque.x, self.torque.y, self.torque.z]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrip_array() {
        let arr = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0];
        let w = Wrench::from_slice(&arr);
        assert_eq!(arr, w.to_array());
    }

    #[test]
    fn add_neg() {
        let a = Wrench::force_only(1.0, 2.0, 3.0);
        let b = Wrench::torque_only(4.0, 5.0, 6.0);
        let c = a + b;
        assert_eq!(c.force, Vector3::new(1.0, 2.0, 3.0));
        assert_eq!(c.torque, Vector3::new(4.0, 5.0, 6.0));
        let d = -c;
        assert_eq!(d.force, Vector3::new(-1.0, -2.0, -3.0));
    }

    #[test]
    fn wrench_data_trait() {
        let w = Wrench::new(Vector3::new(1.0, 2.0, 3.0), Vector3::new(4.0, 5.0, 6.0));
        assert_eq!(WrenchData::force(&w), [1.0, 2.0, 3.0]);
        assert_eq!(WrenchData::torque(&w), [4.0, 5.0, 6.0]);
    }
}
