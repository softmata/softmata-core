use crate::geometry::Pose2D;

// =============================================================================
// Tier 1 Sensor Traits — Algorithm inputs with fundamentally different storage
// =============================================================================

/// Trait for any type providing 2D laser scan data.
pub trait ScanData {
    fn num_ranges(&self) -> usize;
    fn range_at(&self, index: usize) -> f64;
    fn angle_min(&self) -> f64;
    fn angle_max(&self) -> f64;
    fn angle_increment(&self) -> f64;
    fn range_min(&self) -> f64;
    fn range_max(&self) -> f64;

    fn angle_at(&self, index: usize) -> f64 {
        self.angle_min() + (index as f64) * self.angle_increment()
    }

    fn is_range_valid(&self, index: usize) -> bool {
        let r = self.range_at(index);
        r >= self.range_min() && r <= self.range_max() && r.is_finite()
    }
}

/// Trait for any type providing IMU data.
pub trait ImuData {
    /// Orientation as quaternion [x, y, z, w].
    fn orientation(&self) -> [f64; 4];
    /// Angular velocity [x, y, z] in rad/s.
    fn angular_velocity(&self) -> [f64; 3];
    /// Linear acceleration [x, y, z] in m/s^2.
    fn linear_acceleration(&self) -> [f64; 3];
    /// Whether orientation data is available.
    fn has_orientation(&self) -> bool {
        true
    }
}

/// Trait for any type providing odometry data.
pub trait OdometryData {
    fn pose(&self) -> Pose2D;
    fn linear_velocity(&self) -> f64;
    fn angular_velocity(&self) -> f64;
}

/// Trait for any type providing joint state data.
pub trait JointData {
    fn num_joints(&self) -> usize;
    fn position_at(&self, index: usize) -> f64;
    fn velocity_at(&self, index: usize) -> f64;
    fn effort_at(&self, index: usize) -> f64;
    fn joint_name(&self, index: usize) -> Option<&str>;
}

/// Image encoding formats.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum ImageEncoding {
    Rgb8,
    Bgr8,
    Rgba8,
    Bgra8,
    Mono8,
    Mono16,
    Depth32F,
    Depth16U,
}

/// Trait for any type providing image data.
pub trait ImageData {
    fn width(&self) -> u32;
    fn height(&self) -> u32;
    fn encoding(&self) -> ImageEncoding;
    fn data(&self) -> &[u8];
}

/// Trait for any type providing point cloud data.
pub trait PointCloudData {
    fn num_points(&self) -> usize;
    fn point_at(&self, index: usize) -> [f32; 3];
    fn has_intensity(&self) -> bool {
        false
    }
    fn intensity_at(&self, _index: usize) -> f32 {
        0.0
    }
    fn has_color(&self) -> bool {
        false
    }
    fn color_at(&self, _index: usize) -> [u8; 4] {
        [0; 4]
    }
}

/// Trait for any type providing depth image data.
pub trait DepthImageData {
    fn width(&self) -> u32;
    fn height(&self) -> u32;
    /// Depth at pixel (row, col) in meters.
    fn depth_at(&self, row: u32, col: u32) -> f32;
}

/// Trait for any type providing wrench (force/torque) data.
pub trait WrenchData {
    /// Force vector [fx, fy, fz] in Newtons.
    fn force(&self) -> [f64; 3];
    /// Torque vector [tx, ty, tz] in Newton-meters.
    fn torque(&self) -> [f64; 3];

    fn force_magnitude(&self) -> f64 {
        let f = self.force();
        (f[0] * f[0] + f[1] * f[1] + f[2] * f[2]).sqrt()
    }

    fn torque_magnitude(&self) -> f64 {
        let t = self.torque();
        (t[0] * t[0] + t[1] * t[1] + t[2] * t[2]).sqrt()
    }
}

// =============================================================================
// Tier 2 Sensor Traits — Shared between HORUS and terra
// =============================================================================

/// Trait for battery state data.
pub trait BatteryData {
    fn voltage(&self) -> f32;
    fn current(&self) -> f32;
    fn percentage(&self) -> f32;
}

/// Trait for range sensor data.
pub trait RangeData {
    fn range(&self) -> f64;
    fn range_min(&self) -> f64;
    fn range_max(&self) -> f64;

    fn is_valid(&self) -> bool {
        let r = self.range();
        r >= self.range_min() && r <= self.range_max() && r.is_finite()
    }
}

/// Trait for temperature sensor data.
pub trait TemperatureData {
    fn celsius(&self) -> f32;
    fn fahrenheit(&self) -> f32 {
        self.celsius() * 1.8 + 32.0
    }
}

/// Trait for magnetic field sensor data.
pub trait MagneticFieldData {
    /// Magnetic field vector [x, y, z] in microtesla.
    fn magnetic_field(&self) -> [f32; 3];
}

/// Trait for gamepad/joystick input data.
pub trait GamepadData {
    fn num_axes(&self) -> usize;
    fn axis(&self, index: usize) -> f32;
    fn num_buttons(&self) -> usize;
    fn button(&self, index: usize) -> bool;
}

// =============================================================================
// Point traits
// =============================================================================

/// Trait for types that have a 3D position.
pub trait HasPosition {
    fn position(&self) -> [f32; 3];
}
