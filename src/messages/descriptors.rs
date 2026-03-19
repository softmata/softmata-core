//! Pod descriptors for variable-length sensor data.
//!
//! These are fixed-size Pod structs that describe large data (point clouds,
//! images, laser scans) stored in shared memory pools. The descriptor travels
//! through HORUS Topic<T> (zero-copy), while the actual data lives in a
//! shared memory pool referenced by pool_id + slot_id.
//!
//! On embedded (no_std, no HORUS), these work as plain metadata structs —
//! the pool_id/slot_id fields are ignored and data is passed separately.

use bytemuck::{Pod, Zeroable};
use serde::{Deserialize, Serialize};

// =============================================================================
// LaserScan Descriptor
// =============================================================================

/// Descriptor for a 2D laser scan stored in shared memory.
///
/// The actual range data (f32 array) lives in the pool at pool_id/slot_id.
/// This descriptor is Pod-safe for zero-copy IPC.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Pod, Zeroable)]
#[repr(C)]
pub struct LaserScanDesc {
    /// Minimum scan angle in radians.
    pub angle_min: f64,
    /// Maximum scan angle in radians.
    pub angle_max: f64,
    /// Angular distance between measurements in radians.
    pub angle_increment: f64,
    /// Minimum valid range in meters.
    pub range_min: f64,
    /// Maximum valid range in meters.
    pub range_max: f64,
    /// Number of range measurements.
    pub num_ranges: u32,
    /// Shared memory pool ID (0 = no pool, data passed separately).
    pub pool_id: u32,
    /// Slot within the pool.
    pub slot_id: u32,
    /// Padding for alignment.
    pub _pad: u32,
    /// Timestamp in nanoseconds since epoch.
    pub timestamp_ns: u64,
}

impl LaserScanDesc {
    pub fn new(
        angle_min: f64,
        angle_max: f64,
        angle_increment: f64,
        range_min: f64,
        range_max: f64,
        num_ranges: u32,
        timestamp_ns: u64,
    ) -> Self {
        Self {
            angle_min,
            angle_max,
            angle_increment,
            range_min,
            range_max,
            num_ranges,
            pool_id: 0,
            slot_id: 0,
            _pad: 0,
            timestamp_ns,
        }
    }

    /// Compute the angle at a given range index.
    pub fn angle_at(&self, index: u32) -> f64 {
        self.angle_min + (index as f64) * self.angle_increment
    }

    /// Whether this descriptor references a shared memory pool.
    pub fn has_pool(&self) -> bool {
        self.pool_id != 0
    }
}

// =============================================================================
// PointCloud Descriptor
// =============================================================================

/// Point cloud field types.
pub const FIELD_XYZ: u8 = 0;
pub const FIELD_XYZI: u8 = 1;
pub const FIELD_XYZRGB: u8 = 2;
pub const FIELD_XYZRGBN: u8 = 3;

/// Descriptor for a point cloud stored in shared memory.
///
/// Point data lives in the pool. This descriptor carries metadata only.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Pod, Zeroable)]
#[repr(C)]
pub struct PointCloudDesc {
    /// Number of points.
    pub point_count: u32,
    /// Fields per point (3=XYZ, 4=XYZI, etc.).
    pub fields_per_point: u32,
    /// Field type: FIELD_XYZ, FIELD_XYZI, FIELD_XYZRGB, FIELD_XYZRGBN.
    pub field_type: u8,
    /// Whether the cloud is organized (2D grid). 0 = unorganized, 1 = organized.
    pub is_organized: u8,
    /// Padding for alignment.
    pub _pad1: [u8; 2],
    /// Width (columns if organized, else same as point_count).
    pub width: u32,
    /// Height (rows if organized, else 1).
    pub height: u32,
    /// Padding for alignment.
    pub _pad2: u32,
    /// Frame ID (null-terminated UTF-8).
    pub frame_id: [u8; 32],
    /// Shared memory pool ID (0 = no pool).
    pub pool_id: u32,
    /// Slot within the pool.
    pub slot_id: u32,
    /// Timestamp in nanoseconds since epoch.
    pub timestamp_ns: u64,
}

impl PointCloudDesc {
    pub fn new(point_count: u32, field_type: u8, timestamp_ns: u64) -> Self {
        let fields = match field_type {
            FIELD_XYZ => 3,
            FIELD_XYZI => 4,
            FIELD_XYZRGB => 4,
            FIELD_XYZRGBN => 7,
            _ => 3,
        };
        Self {
            point_count,
            fields_per_point: fields,
            field_type,
            is_organized: 0,
            _pad1: [0; 2],
            width: point_count,
            height: 1,
            _pad2: 0,
            frame_id: [0; 32],
            pool_id: 0,
            slot_id: 0,
            timestamp_ns,
        }
    }

    pub fn with_frame_id(mut self, frame: &str) -> Self {
        let bytes = frame.as_bytes();
        let len = bytes.len().min(31);
        self.frame_id[..len].copy_from_slice(&bytes[..len]);
        self
    }

    pub fn frame_id_str(&self) -> &str {
        let end = self.frame_id.iter().position(|&b| b == 0).unwrap_or(32);
        core::str::from_utf8(&self.frame_id[..end]).unwrap_or("")
    }

    pub fn has_pool(&self) -> bool {
        self.pool_id != 0
    }
}

// =============================================================================
// Image Descriptor
// =============================================================================

/// Image encoding constants.
pub const ENC_RGB8: u8 = 0;
pub const ENC_BGR8: u8 = 1;
pub const ENC_RGBA8: u8 = 2;
pub const ENC_MONO8: u8 = 3;
pub const ENC_MONO16: u8 = 4;

/// Descriptor for an image stored in shared memory.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Pod, Zeroable)]
#[repr(C)]
pub struct ImageDesc {
    /// Image width in pixels.
    pub width: u32,
    /// Image height in pixels.
    pub height: u32,
    /// Encoding: ENC_RGB8, ENC_BGR8, ENC_RGBA8, ENC_MONO8, ENC_MONO16.
    pub encoding: u8,
    /// Padding for alignment.
    pub _pad1: [u8; 3],
    /// Row stride in bytes.
    pub step: u32,
    /// Frame ID (null-terminated UTF-8).
    pub frame_id: [u8; 32],
    /// Shared memory pool ID (0 = no pool).
    pub pool_id: u32,
    /// Slot within the pool.
    pub slot_id: u32,
    /// Timestamp in nanoseconds since epoch.
    pub timestamp_ns: u64,
}

impl ImageDesc {
    pub fn new(width: u32, height: u32, encoding: u8, timestamp_ns: u64) -> Self {
        let bpp = match encoding {
            ENC_RGB8 | ENC_BGR8 => 3,
            ENC_RGBA8 => 4,
            ENC_MONO8 => 1,
            ENC_MONO16 => 2,
            _ => 3,
        };
        Self {
            width,
            height,
            encoding,
            _pad1: [0; 3],
            step: width * bpp,
            frame_id: [0; 32],
            pool_id: 0,
            slot_id: 0,
            timestamp_ns,
        }
    }

    pub fn with_frame_id(mut self, frame: &str) -> Self {
        let bytes = frame.as_bytes();
        let len = bytes.len().min(31);
        self.frame_id[..len].copy_from_slice(&bytes[..len]);
        self
    }

    pub fn frame_id_str(&self) -> &str {
        let end = self.frame_id.iter().position(|&b| b == 0).unwrap_or(32);
        core::str::from_utf8(&self.frame_id[..end]).unwrap_or("")
    }

    pub fn data_size(&self) -> usize {
        self.step as usize * self.height as usize
    }
}

// =============================================================================
// Depth Image Descriptor
// =============================================================================

/// Depth encoding constants.
pub const DEPTH_F32: u8 = 0;
pub const DEPTH_U16: u8 = 1;

/// Descriptor for a depth image stored in shared memory.
///
/// Includes camera intrinsics for 3D reconstruction.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Pod, Zeroable)]
#[repr(C)]
pub struct DepthImageDesc {
    /// Image width in pixels.
    pub width: u32,
    /// Image height in pixels.
    pub height: u32,
    /// Depth encoding: DEPTH_F32 (meters) or DEPTH_U16 (millimeters).
    pub encoding: u8,
    /// Padding for alignment.
    pub _pad1: [u8; 7],
    /// Camera focal length X in pixels.
    pub fx: f64,
    /// Camera focal length Y in pixels.
    pub fy: f64,
    /// Camera principal point X in pixels.
    pub cx: f64,
    /// Camera principal point Y in pixels.
    pub cy: f64,
    /// Frame ID (null-terminated UTF-8).
    pub frame_id: [u8; 32],
    /// Shared memory pool ID (0 = no pool).
    pub pool_id: u32,
    /// Slot within the pool.
    pub slot_id: u32,
    /// Timestamp in nanoseconds since epoch.
    pub timestamp_ns: u64,
}

impl DepthImageDesc {
    pub fn new(width: u32, height: u32, fx: f64, fy: f64, cx: f64, cy: f64, timestamp_ns: u64) -> Self {
        Self {
            width,
            height,
            encoding: DEPTH_F32,
            _pad1: [0; 7],
            fx,
            fy,
            cx,
            cy,
            frame_id: [0; 32],
            pool_id: 0,
            slot_id: 0,
            timestamp_ns,
        }
    }

    pub fn with_frame_id(mut self, frame: &str) -> Self {
        let bytes = frame.as_bytes();
        let len = bytes.len().min(31);
        self.frame_id[..len].copy_from_slice(&bytes[..len]);
        self
    }

    pub fn frame_id_str(&self) -> &str {
        let end = self.frame_id.iter().position(|&b| b == 0).unwrap_or(32);
        core::str::from_utf8(&self.frame_id[..end]).unwrap_or("")
    }

    /// RealSense D435 factory defaults at 640x480.
    pub fn realsense_d435(timestamp_ns: u64) -> Self {
        Self::new(640, 480, 382.0, 382.0, 320.0, 240.0, timestamp_ns)
            .with_frame_id("camera_depth")
    }
}

// =============================================================================
// Tests
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn laser_scan_desc_pod() {
        let desc = LaserScanDesc::new(-3.14, 3.14, 0.01, 0.1, 30.0, 628, 42);
        let bytes = bytemuck::bytes_of(&desc);
        let desc2: &LaserScanDesc = bytemuck::from_bytes(bytes);
        assert_eq!(desc.num_ranges, desc2.num_ranges);
        assert_eq!(desc.timestamp_ns, 42);
    }

    #[test]
    fn laser_scan_angle_at() {
        let desc = LaserScanDesc::new(0.0, 6.28, 0.01, 0.1, 30.0, 628, 0);
        assert!((desc.angle_at(100) - 1.0).abs() < 1e-10);
    }

    #[test]
    fn pointcloud_desc_frame_id() {
        let desc = PointCloudDesc::new(1000, FIELD_XYZI, 99).with_frame_id("lidar");
        assert_eq!(desc.frame_id_str(), "lidar");
        assert_eq!(desc.fields_per_point, 4);
        assert_eq!(desc.point_count, 1000);
    }

    #[test]
    fn pointcloud_desc_pod() {
        let desc = PointCloudDesc::new(500, FIELD_XYZ, 0);
        let bytes = bytemuck::bytes_of(&desc);
        let desc2: &PointCloudDesc = bytemuck::from_bytes(bytes);
        assert_eq!(desc.point_count, desc2.point_count);
    }

    #[test]
    fn image_desc_data_size() {
        let desc = ImageDesc::new(640, 480, ENC_RGB8, 0);
        assert_eq!(desc.data_size(), 640 * 480 * 3);
        let mono = ImageDesc::new(640, 480, ENC_MONO8, 0);
        assert_eq!(mono.data_size(), 640 * 480);
    }

    #[test]
    fn depth_image_realsense() {
        let desc = DepthImageDesc::realsense_d435(42);
        assert_eq!(desc.width, 640);
        assert_eq!(desc.height, 480);
        assert_eq!(desc.frame_id_str(), "camera_depth");
        assert!((desc.fx - 382.0).abs() < 1e-10);
    }

    #[test]
    fn all_descriptors_are_pod() {
        // Compile-time verification via Pod derive.
        // Runtime verification via zero-copy roundtrip.
        let scan = LaserScanDesc::new(0.0, 1.0, 0.01, 0.1, 10.0, 100, 0);
        let _ = bytemuck::bytes_of(&scan);
        let pc = PointCloudDesc::new(100, FIELD_XYZ, 0);
        let _ = bytemuck::bytes_of(&pc);
        let img = ImageDesc::new(320, 240, ENC_RGB8, 0);
        let _ = bytemuck::bytes_of(&img);
        let depth = DepthImageDesc::new(320, 240, 300.0, 300.0, 160.0, 120.0, 0);
        let _ = bytemuck::bytes_of(&depth);
    }
}
