use super::{Point3, Quaternion};
use serde::{Deserialize, Serialize};

/// 3D pose: position + orientation.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[repr(C)]
pub struct Pose3D {
    pub position: Point3,
    pub orientation: Quaternion,
}

impl Default for Pose3D {
    fn default() -> Self {
        Self::identity()
    }
}

impl Pose3D {
    pub fn new(position: Point3, orientation: Quaternion) -> Self {
        Self {
            position,
            orientation,
        }
    }

    pub fn identity() -> Self {
        Self {
            position: Point3::origin(),
            orientation: Quaternion::identity(),
        }
    }

    pub fn from_xyz(x: f64, y: f64, z: f64) -> Self {
        Self {
            position: Point3::new(x, y, z),
            orientation: Quaternion::identity(),
        }
    }

    /// Compose two poses (self * other).
    pub fn compose(&self, other: &Self) -> Self {
        let rotated_pos = self.orientation.rotate_vector(&other.position.to_vector());
        Self {
            position: Point3::new(
                self.position.x + rotated_pos.x,
                self.position.y + rotated_pos.y,
                self.position.z + rotated_pos.z,
            ),
            orientation: self.orientation.multiply(&other.orientation).normalized(),
        }
    }

    /// Inverse of this pose.
    pub fn inverse(&self) -> Self {
        let inv_rot = self.orientation.inverse();
        let inv_pos = inv_rot.rotate_vector(&self.position.to_vector());
        Self {
            position: Point3::new(-inv_pos.x, -inv_pos.y, -inv_pos.z),
            orientation: inv_rot,
        }
    }

    /// Linear interpolation.
    pub fn interpolate(&self, other: &Self, t: f64) -> Self {
        let t = t.clamp(0.0, 1.0);
        Self {
            position: Point3::new(
                self.position.x + t * (other.position.x - self.position.x),
                self.position.y + t * (other.position.y - self.position.y),
                self.position.z + t * (other.position.z - self.position.z),
            ),
            orientation: self.orientation.slerp(&other.orientation, t),
        }
    }

    /// Convert to nalgebra Isometry3.
    #[cfg(feature = "std")]
    pub fn to_isometry3(&self) -> nalgebra::Isometry3<f64> {
        nalgebra::Isometry3::from_parts(
            nalgebra::Translation3::new(self.position.x, self.position.y, self.position.z),
            self.orientation.to_nalgebra(),
        )
    }

    /// Create from nalgebra Isometry3.
    #[cfg(feature = "std")]
    pub fn from_isometry3(iso: &nalgebra::Isometry3<f64>) -> Self {
        Self {
            position: Point3::new(iso.translation.x, iso.translation.y, iso.translation.z),
            orientation: Quaternion::from_nalgebra(&iso.rotation),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn compose_inverse_roundtrip() {
        let a = Pose3D::new(
            Point3::new(1.0, 2.0, 3.0),
            Quaternion::from_euler(0.1, 0.2, 0.3),
        );
        let b = Pose3D::new(
            Point3::new(4.0, 5.0, 6.0),
            Quaternion::from_euler(0.4, 0.5, 0.6),
        );
        let c = a.compose(&b);
        let b_recovered = a.inverse().compose(&c);
        assert!((b_recovered.position.x - b.position.x).abs() < 1e-9);
        assert!((b_recovered.position.y - b.position.y).abs() < 1e-9);
        assert!((b_recovered.position.z - b.position.z).abs() < 1e-9);
    }

    #[test]
    fn identity_compose() {
        let a = Pose3D::new(
            Point3::new(1.0, 2.0, 3.0),
            Quaternion::from_euler(0.1, 0.2, 0.3),
        );
        let b = a.compose(&Pose3D::identity());
        assert!((b.position.x - a.position.x).abs() < 1e-10);
        assert!((b.position.y - a.position.y).abs() < 1e-10);
    }
}
