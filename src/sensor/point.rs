use super::traits::HasPosition;
use serde::{Deserialize, Serialize};

/// 3D point with XYZ coordinates.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
#[repr(C)]
pub struct PointXYZ {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl PointXYZ {
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }
    pub fn origin() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }
    }
}

impl HasPosition for PointXYZ {
    fn position(&self) -> [f32; 3] {
        [self.x, self.y, self.z]
    }
}

/// 3D point with intensity (LiDAR returns).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
#[repr(C)]
pub struct PointXYZI {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub intensity: f32,
}

impl PointXYZI {
    pub fn new(x: f32, y: f32, z: f32, intensity: f32) -> Self {
        Self { x, y, z, intensity }
    }
}

impl HasPosition for PointXYZI {
    fn position(&self) -> [f32; 3] {
        [self.x, self.y, self.z]
    }
}

/// 3D point with RGB color (RGB-D cameras).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
#[repr(C)]
pub struct PointXYZRGB {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub r: u8,
    pub g: u8,
    pub b: u8,
    pub a: u8,
}

impl PointXYZRGB {
    pub fn new(x: f32, y: f32, z: f32, r: u8, g: u8, b: u8) -> Self {
        Self {
            x,
            y,
            z,
            r,
            g,
            b,
            a: 255,
        }
    }
}

impl HasPosition for PointXYZRGB {
    fn position(&self) -> [f32; 3] {
        [self.x, self.y, self.z]
    }
}

/// 3D point with surface normal.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
#[repr(C)]
pub struct PointXYZNormal {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub nx: f32,
    pub ny: f32,
    pub nz: f32,
}

impl PointXYZNormal {
    pub fn new(x: f32, y: f32, z: f32, nx: f32, ny: f32, nz: f32) -> Self {
        Self {
            x,
            y,
            z,
            nx,
            ny,
            nz,
        }
    }
}

impl HasPosition for PointXYZNormal {
    fn position(&self) -> [f32; 3] {
        [self.x, self.y, self.z]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn point_sizes() {
        assert_eq!(core::mem::size_of::<PointXYZ>(), 12);
        assert_eq!(core::mem::size_of::<PointXYZI>(), 16);
        assert_eq!(core::mem::size_of::<PointXYZRGB>(), 16);
        assert_eq!(core::mem::size_of::<PointXYZNormal>(), 24);
    }

    #[test]
    fn has_position() {
        let p = PointXYZ::new(1.0, 2.0, 3.0);
        assert_eq!(p.position(), [1.0, 2.0, 3.0]);
    }
}
