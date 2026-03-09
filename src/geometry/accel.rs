use super::Vector3;
use serde::{Deserialize, Serialize};

/// 6-DOF acceleration: linear + angular.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
#[repr(C)]
pub struct Accel {
    pub linear: Vector3,
    pub angular: Vector3,
}

impl Accel {
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
