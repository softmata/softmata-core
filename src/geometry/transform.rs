use super::{Point2, Point3, Quaternion, Vector3};
use serde::{Deserialize, Serialize};

/// 2D rigid body transform with optional scale.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[repr(C)]
pub struct Transform2D {
    pub x: f64,
    pub y: f64,
    pub theta: f64,
    pub scale: f64,
}

impl Default for Transform2D {
    fn default() -> Self {
        Self::identity()
    }
}

impl Transform2D {
    pub fn new(x: f64, y: f64, theta: f64) -> Self {
        Self {
            x,
            y,
            theta,
            scale: 1.0,
        }
    }

    pub fn identity() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
            scale: 1.0,
        }
    }

    pub fn with_scale(mut self, scale: f64) -> Self {
        self.scale = scale;
        self
    }

    pub fn compose(&self, other: &Self) -> Self {
        let cos_t = self.theta.cos();
        let sin_t = self.theta.sin();
        Self {
            x: self.x + self.scale * (cos_t * other.x - sin_t * other.y),
            y: self.y + self.scale * (sin_t * other.x + cos_t * other.y),
            theta: self.theta + other.theta,
            scale: self.scale * other.scale,
        }
    }

    pub fn inverse(&self) -> Self {
        let cos_t = self.theta.cos();
        let sin_t = self.theta.sin();
        let inv_s = 1.0 / self.scale;
        Self {
            x: -inv_s * (cos_t * self.x + sin_t * self.y),
            y: -inv_s * (-sin_t * self.x + cos_t * self.y),
            theta: -self.theta,
            scale: inv_s,
        }
    }

    pub fn apply_to_point(&self, p: &Point2) -> Point2 {
        let cos_t = self.theta.cos();
        let sin_t = self.theta.sin();
        Point2::new(
            self.x + self.scale * (cos_t * p.x - sin_t * p.y),
            self.y + self.scale * (sin_t * p.x + cos_t * p.y),
        )
    }
}

/// 3D rigid body transform.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[repr(C)]
pub struct Transform3D {
    pub translation: Vector3,
    pub rotation: Quaternion,
}

impl Default for Transform3D {
    fn default() -> Self {
        Self::identity()
    }
}

impl Transform3D {
    pub fn new(translation: Vector3, rotation: Quaternion) -> Self {
        Self {
            translation,
            rotation,
        }
    }

    pub fn identity() -> Self {
        Self {
            translation: Vector3::zero(),
            rotation: Quaternion::identity(),
        }
    }

    pub fn compose(&self, other: &Self) -> Self {
        let rotated = self.rotation.rotate_vector(&other.translation);
        Self {
            translation: self.translation + rotated,
            rotation: self.rotation.multiply(&other.rotation).normalized(),
        }
    }

    pub fn inverse(&self) -> Self {
        let inv_rot = self.rotation.inverse();
        let inv_trans = inv_rot.rotate_vector(&self.translation);
        Self {
            translation: -inv_trans,
            rotation: inv_rot,
        }
    }

    pub fn apply_to_point(&self, p: &Point3) -> Point3 {
        let rotated = self.rotation.rotate_vector(&p.to_vector());
        Point3::new(
            self.translation.x + rotated.x,
            self.translation.y + rotated.y,
            self.translation.z + rotated.z,
        )
    }

    #[cfg(feature = "std")]
    pub fn from_isometry3(iso: &nalgebra::Isometry3<f64>) -> Self {
        Self {
            translation: Vector3::new(iso.translation.x, iso.translation.y, iso.translation.z),
            rotation: Quaternion::from_nalgebra(&iso.rotation),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn transform3d_compose_inverse() {
        let t = Transform3D::new(
            Vector3::new(1.0, 2.0, 3.0),
            Quaternion::from_euler(0.1, 0.2, 0.3),
        );
        let identity = t.compose(&t.inverse());
        assert!((identity.translation.x).abs() < 1e-9);
        assert!((identity.translation.y).abs() < 1e-9);
        assert!((identity.translation.z).abs() < 1e-9);
        assert!((identity.rotation.w - 1.0).abs() < 1e-9);
    }

    #[test]
    fn transform2d_compose_inverse() {
        let t = Transform2D::new(3.0, 4.0, 0.5);
        let identity = t.compose(&t.inverse());
        assert!((identity.x).abs() < 1e-9);
        assert!((identity.y).abs() < 1e-9);
        assert!((identity.theta).abs() < 1e-9);
    }
}
