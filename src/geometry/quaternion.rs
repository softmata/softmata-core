use serde::{Deserialize, Serialize};

/// Quaternion for 3D rotations (x, y, z, w convention).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[repr(C)]
pub struct Quaternion {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}

impl Default for Quaternion {
    fn default() -> Self {
        Self::identity()
    }
}

impl Quaternion {
    pub fn new(x: f64, y: f64, z: f64, w: f64) -> Self {
        Self { x, y, z, w }
    }

    pub fn identity() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        }
    }

    /// Create from axis-angle representation.
    pub fn from_axis_angle(axis: &super::Vector3, angle: f64) -> Self {
        let half = angle * 0.5;
        let s = half.sin();
        let norm = axis.magnitude();
        if norm < 1e-15 {
            return Self::identity();
        }
        Self {
            x: axis.x / norm * s,
            y: axis.y / norm * s,
            z: axis.z / norm * s,
            w: half.cos(),
        }
    }

    /// Create from Euler angles (roll, pitch, yaw) in radians.
    pub fn from_euler(roll: f64, pitch: f64, yaw: f64) -> Self {
        let cr = (roll * 0.5).cos();
        let sr = (roll * 0.5).sin();
        let cp = (pitch * 0.5).cos();
        let sp = (pitch * 0.5).sin();
        let cy = (yaw * 0.5).cos();
        let sy = (yaw * 0.5).sin();
        Self {
            x: sr * cp * cy - cr * sp * sy,
            y: cr * sp * cy + sr * cp * sy,
            z: cr * cp * sy - sr * sp * cy,
            w: cr * cp * cy + sr * sp * sy,
        }
    }

    /// Quaternion norm.
    pub fn norm(&self) -> f64 {
        (self.x * self.x + self.y * self.y + self.z * self.z + self.w * self.w).sqrt()
    }

    /// Return a normalized copy.
    pub fn normalized(&self) -> Self {
        let n = self.norm();
        if n < 1e-15 {
            return Self::identity();
        }
        Self {
            x: self.x / n,
            y: self.y / n,
            z: self.z / n,
            w: self.w / n,
        }
    }

    /// Conjugate (inverse for unit quaternions).
    pub fn conjugate(&self) -> Self {
        Self {
            x: -self.x,
            y: -self.y,
            z: -self.z,
            w: self.w,
        }
    }

    /// Inverse (conjugate / norm^2).
    pub fn inverse(&self) -> Self {
        let n2 = self.x * self.x + self.y * self.y + self.z * self.z + self.w * self.w;
        if n2 < 1e-15 {
            return Self::identity();
        }
        Self {
            x: -self.x / n2,
            y: -self.y / n2,
            z: -self.z / n2,
            w: self.w / n2,
        }
    }

    /// Hamilton product (quaternion multiplication).
    pub fn multiply(&self, other: &Self) -> Self {
        Self {
            x: self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y,
            y: self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x,
            z: self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w,
            w: self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z,
        }
    }

    /// Spherical linear interpolation.
    pub fn slerp(&self, other: &Self, t: f64) -> Self {
        let t = t.clamp(0.0, 1.0);
        let mut dot = self.x * other.x + self.y * other.y + self.z * other.z + self.w * other.w;
        let mut other = *other;
        // Take shorter path
        if dot < 0.0 {
            other = Self {
                x: -other.x,
                y: -other.y,
                z: -other.z,
                w: -other.w,
            };
            dot = -dot;
        }
        if dot > 0.9995 {
            // Linear interpolation for very close quaternions
            let result = Self {
                x: self.x + t * (other.x - self.x),
                y: self.y + t * (other.y - self.y),
                z: self.z + t * (other.z - self.z),
                w: self.w + t * (other.w - self.w),
            };
            return result.normalized();
        }
        let theta = dot.acos();
        let sin_theta = theta.sin();
        let s0 = ((1.0 - t) * theta).sin() / sin_theta;
        let s1 = (t * theta).sin() / sin_theta;
        Self {
            x: s0 * self.x + s1 * other.x,
            y: s0 * self.y + s1 * other.y,
            z: s0 * self.z + s1 * other.z,
            w: s0 * self.w + s1 * other.w,
        }
    }

    /// Rotate a vector by this quaternion.
    pub fn rotate_vector(&self, v: &super::Vector3) -> super::Vector3 {
        let qv = Self::new(v.x, v.y, v.z, 0.0);
        let result = self.multiply(&qv).multiply(&self.conjugate());
        super::Vector3::new(result.x, result.y, result.z)
    }

    /// Convert to nalgebra UnitQuaternion.
    #[cfg(feature = "std")]
    pub fn to_nalgebra(&self) -> nalgebra::UnitQuaternion<f64> {
        nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
            self.w, self.x, self.y, self.z,
        ))
    }

    /// Create from nalgebra UnitQuaternion.
    #[cfg(feature = "std")]
    pub fn from_nalgebra(q: &nalgebra::UnitQuaternion<f64>) -> Self {
        let q = q.quaternion();
        Self {
            x: q.i,
            y: q.j,
            z: q.k,
            w: q.w,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::super::Vector3;
    use super::*;

    #[test]
    fn identity_rotation() {
        let q = Quaternion::identity();
        let v = Vector3::new(1.0, 2.0, 3.0);
        let r = q.rotate_vector(&v);
        assert!((r.x - v.x).abs() < 1e-10);
        assert!((r.y - v.y).abs() < 1e-10);
        assert!((r.z - v.z).abs() < 1e-10);
    }

    #[test]
    fn slerp_endpoints() {
        let a = Quaternion::identity();
        let b =
            Quaternion::from_axis_angle(&Vector3::new(0.0, 0.0, 1.0), core::f64::consts::FRAC_PI_2);
        let at_0 = a.slerp(&b, 0.0);
        let at_1 = a.slerp(&b, 1.0);
        assert!((at_0.w - a.w).abs() < 1e-10);
        assert!((at_1.w - b.w).abs() < 1e-10);
    }

    #[test]
    fn inverse_roundtrip() {
        let q = Quaternion::from_euler(0.3, 0.5, 0.7);
        let qi = q.inverse();
        let product = q.multiply(&qi);
        assert!((product.w - 1.0).abs() < 1e-10);
        assert!((product.x).abs() < 1e-10);
    }

    #[test]
    fn rotate_90_z() {
        let q =
            Quaternion::from_axis_angle(&Vector3::new(0.0, 0.0, 1.0), core::f64::consts::FRAC_PI_2);
        let v = Vector3::new(1.0, 0.0, 0.0);
        let r = q.rotate_vector(&v);
        assert!((r.x).abs() < 1e-10);
        assert!((r.y - 1.0).abs() < 1e-10);
        assert!((r.z).abs() < 1e-10);
    }
}
