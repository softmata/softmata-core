//! Pod sensor message types for zero-copy IPC.

use bytemuck::{Pod, Zeroable};
use serde::{Deserialize, Serialize};

use crate::geometry::{Pose2D, Twist2D};
use crate::sensor::traits::ImuData;

// =============================================================================
// IMU
// =============================================================================

/// Inertial Measurement Unit reading.
///
/// Canonical IMU message for all Softmata projects. Pod-safe for HORUS zero-copy IPC.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Pod, Zeroable)]
#[repr(C)]
pub struct ImuReading {
    /// Linear acceleration [x, y, z] in m/s^2.
    pub linear_acceleration: [f64; 3],
    /// Angular velocity [x, y, z] in rad/s.
    pub angular_velocity: [f64; 3],
    /// Orientation as quaternion [x, y, z, w].
    pub orientation: [f64; 4],
    /// Timestamp in nanoseconds since epoch.
    pub timestamp_ns: u64,
}

impl ImuReading {
    pub fn new(
        linear_acceleration: [f64; 3],
        angular_velocity: [f64; 3],
        orientation: [f64; 4],
        timestamp_ns: u64,
    ) -> Self {
        Self {
            linear_acceleration,
            angular_velocity,
            orientation,
            timestamp_ns,
        }
    }
}

impl ImuData for ImuReading {
    fn orientation(&self) -> [f64; 4] {
        self.orientation
    }
    fn angular_velocity(&self) -> [f64; 3] {
        self.angular_velocity
    }
    fn linear_acceleration(&self) -> [f64; 3] {
        self.linear_acceleration
    }
}

// =============================================================================
// Velocity Command
// =============================================================================

/// Velocity command for differential-drive robots.
///
/// 2-DOF: linear (forward) and angular (rotation). For 6-DOF use Twist3D.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Pod, Zeroable)]
#[repr(C)]
pub struct CmdVel {
    /// Linear velocity in m/s (forward positive).
    pub linear: f64,
    /// Angular velocity in rad/s (counter-clockwise positive).
    pub angular: f64,
    /// Timestamp in nanoseconds since epoch.
    pub timestamp_ns: u64,
}

impl CmdVel {
    pub fn new(linear: f64, angular: f64, timestamp_ns: u64) -> Self {
        Self {
            linear,
            angular,
            timestamp_ns,
        }
    }

    pub fn stop(timestamp_ns: u64) -> Self {
        Self {
            linear: 0.0,
            angular: 0.0,
            timestamp_ns,
        }
    }

    pub fn is_stopped(&self) -> bool {
        self.linear.abs() < 1e-6 && self.angular.abs() < 1e-6
    }
}

impl From<CmdVel> for Twist2D {
    fn from(cmd: CmdVel) -> Self {
        Twist2D::new(cmd.linear, cmd.angular)
    }
}

impl From<Twist2D> for CmdVel {
    fn from(twist: Twist2D) -> Self {
        Self {
            linear: twist.v,
            angular: twist.w,
            timestamp_ns: 0,
        }
    }
}

// =============================================================================
// Odometry
// =============================================================================

/// Odometry reading combining pose and velocity.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Pod, Zeroable)]
#[repr(C)]
pub struct Odometry {
    /// Robot pose in the odometry frame.
    pub x: f64,
    pub y: f64,
    pub theta: f64,
    /// Linear velocity in m/s.
    pub linear_velocity: f64,
    /// Angular velocity in rad/s.
    pub angular_velocity: f64,
    /// Timestamp in nanoseconds since epoch.
    pub timestamp_ns: u64,
}

impl Odometry {
    pub fn new(pose: Pose2D, linear_velocity: f64, angular_velocity: f64, timestamp_ns: u64) -> Self {
        Self {
            x: pose.x,
            y: pose.y,
            theta: pose.theta,
            linear_velocity,
            angular_velocity,
            timestamp_ns,
        }
    }

    pub fn pose(&self) -> Pose2D {
        Pose2D::new(self.x, self.y, self.theta)
    }

    pub fn twist(&self) -> Twist2D {
        Twist2D::new(self.linear_velocity, self.angular_velocity)
    }
}

// =============================================================================
// Joint State
// =============================================================================

/// Maximum number of joints supported in a Pod JointState message.
pub const MAX_JOINTS: usize = 32;

/// Joint state for up to 32 joints.
///
/// Fixed-size for Pod compatibility. Use `num_joints` to know how many are valid.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Pod, Zeroable)]
#[repr(C)]
pub struct JointState {
    /// Joint positions in radians (revolute) or meters (prismatic).
    pub position: [f64; MAX_JOINTS],
    /// Joint velocities in rad/s or m/s.
    pub velocity: [f64; MAX_JOINTS],
    /// Joint efforts in Nm or N.
    pub effort: [f64; MAX_JOINTS],
    /// Number of valid joints (0..MAX_JOINTS).
    pub num_joints: u32,
    /// Padding for alignment.
    pub _pad: [u8; 4],
    /// Timestamp in nanoseconds since epoch.
    pub timestamp_ns: u64,
}

impl JointState {
    pub fn new(num_joints: usize, timestamp_ns: u64) -> Self {
        let mut s = Self::zeroed();
        s.num_joints = num_joints.min(MAX_JOINTS) as u32;
        s.timestamp_ns = timestamp_ns;
        s
    }

    pub fn zeroed() -> Self {
        bytemuck::Zeroable::zeroed()
    }

    pub fn set_position(&mut self, index: usize, value: f64) {
        if index < self.num_joints as usize {
            self.position[index] = value;
        }
    }

    pub fn set_velocity(&mut self, index: usize, value: f64) {
        if index < self.num_joints as usize {
            self.velocity[index] = value;
        }
    }

    pub fn set_effort(&mut self, index: usize, value: f64) {
        if index < self.num_joints as usize {
            self.effort[index] = value;
        }
    }
}

// =============================================================================
// Battery State
// =============================================================================

/// Battery state information.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Pod, Zeroable)]
#[repr(C)]
pub struct BatteryState {
    /// Battery voltage in Volts.
    pub voltage: f64,
    /// Battery current in Amps (negative = discharging).
    pub current: f64,
    /// Charge percentage (0.0 to 100.0).
    pub percentage: f64,
    /// Charging state: 0 = discharging, 1 = charging, 2 = full.
    pub charging: u8,
    /// Padding for alignment.
    pub _pad: [u8; 7],
    /// Timestamp in nanoseconds since epoch.
    pub timestamp_ns: u64,
}

impl BatteryState {
    pub fn new(voltage: f64, current: f64, percentage: f64, charging: u8, timestamp_ns: u64) -> Self {
        Self {
            voltage,
            current,
            percentage,
            charging,
            _pad: [0; 7],
            timestamp_ns,
        }
    }

    pub fn is_low(&self) -> bool {
        self.percentage < 20.0
    }

    pub fn is_critical(&self) -> bool {
        self.percentage < 5.0
    }
}

// =============================================================================
// GPS / NavSatFix
// =============================================================================

/// GPS position fix.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Pod, Zeroable)]
#[repr(C)]
pub struct NavSatFix {
    /// Latitude in degrees.
    pub latitude: f64,
    /// Longitude in degrees.
    pub longitude: f64,
    /// Altitude above WGS84 ellipsoid in meters.
    pub altitude: f64,
    /// Horizontal accuracy (CEP) in meters.
    pub accuracy: f64,
    /// Fix type: 0 = no fix, 1 = 2D, 2 = 3D, 3 = DGPS, 4 = RTK fixed.
    pub fix_type: u8,
    /// Number of satellites used.
    pub num_satellites: u8,
    /// Padding for alignment.
    pub _pad: [u8; 6],
    /// Timestamp in nanoseconds since epoch.
    pub timestamp_ns: u64,
}

impl NavSatFix {
    pub fn new(latitude: f64, longitude: f64, altitude: f64, timestamp_ns: u64) -> Self {
        Self {
            latitude,
            longitude,
            altitude,
            accuracy: 0.0,
            fix_type: 0,
            num_satellites: 0,
            _pad: [0; 6],
            timestamp_ns,
        }
    }

    pub fn has_fix(&self) -> bool {
        self.fix_type > 0
    }
}

// =============================================================================
// Tests
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn imu_reading_pod_size() {
        // 3*8 + 3*8 + 4*8 + 8 = 24+24+32+8 = 88 bytes
        assert_eq!(core::mem::size_of::<ImuReading>(), 88);
    }

    #[test]
    fn imu_reading_imu_data_trait() {
        let imu = ImuReading::new([0.0, 0.0, 9.81], [0.0, 0.0, 0.1], [0.0, 0.0, 0.0, 1.0], 12345);
        assert_eq!(imu.linear_acceleration()[2], 9.81);
        assert_eq!(imu.angular_velocity()[2], 0.1);
        assert_eq!(imu.orientation()[3], 1.0);
    }

    #[test]
    fn cmd_vel_pod_size() {
        // 8 + 8 + 8 = 24 bytes
        assert_eq!(core::mem::size_of::<CmdVel>(), 24);
    }

    #[test]
    fn cmd_vel_twist2d_roundtrip() {
        let cmd = CmdVel::new(1.0, 0.5, 999);
        let twist: Twist2D = cmd.into();
        assert_eq!(twist.v, 1.0);
        assert_eq!(twist.w, 0.5);
        let cmd2: CmdVel = twist.into();
        assert_eq!(cmd2.linear, 1.0);
        assert_eq!(cmd2.angular, 0.5);
    }

    #[test]
    fn cmd_vel_stop() {
        let cmd = CmdVel::stop(0);
        assert!(cmd.is_stopped());
    }

    #[test]
    fn odometry_pod_size() {
        // 5*8 + 8 = 48 bytes
        assert_eq!(core::mem::size_of::<Odometry>(), 48);
    }

    #[test]
    fn odometry_pose_twist() {
        let odom = Odometry::new(Pose2D::new(1.0, 2.0, 0.5), 0.3, 0.1, 42);
        let p = odom.pose();
        assert_eq!(p.x, 1.0);
        assert_eq!(p.y, 2.0);
        let t = odom.twist();
        assert_eq!(t.v, 0.3);
        assert_eq!(t.w, 0.1);
    }

    #[test]
    fn joint_state_pod_size() {
        let size = core::mem::size_of::<JointState>();
        // All f64 fields aligned, no padding issues
        assert!(size > 0);
    }

    #[test]
    fn joint_state_set_get() {
        let mut js = JointState::new(6, 0);
        js.set_position(0, 1.57);
        js.set_velocity(0, 0.5);
        js.set_effort(0, 10.0);
        assert_eq!(js.position[0], 1.57);
        assert_eq!(js.velocity[0], 0.5);
        assert_eq!(js.effort[0], 10.0);
        // Out of bounds should be ignored
        js.set_position(6, 999.0);
        assert_eq!(js.position[6], 0.0);
    }

    #[test]
    fn battery_state_pod_size() {
        // 8*3 + 1 + 7 + 8 = 24 + 16 = 40 bytes
        assert_eq!(core::mem::size_of::<BatteryState>(), 40);
    }

    #[test]
    fn battery_low_critical() {
        let bat = BatteryState::new(11.1, -2.0, 15.0, 0, 0);
        assert!(bat.is_low());
        assert!(!bat.is_critical());
        let bat2 = BatteryState::new(10.5, -2.0, 3.0, 0, 0);
        assert!(bat2.is_critical());
    }

    #[test]
    fn navsat_fix() {
        let fix = NavSatFix::new(37.7749, -122.4194, 10.0, 0);
        assert!(!fix.has_fix());
        let mut fix2 = fix;
        fix2.fix_type = 3;
        assert!(fix2.has_fix());
    }

    #[test]
    fn pod_zero_copy_roundtrip() {
        let imu = ImuReading::new([1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [0.0, 0.0, 0.0, 1.0], 42);
        let bytes = bytemuck::bytes_of(&imu);
        let imu2: &ImuReading = bytemuck::from_bytes(bytes);
        assert_eq!(imu.linear_acceleration, imu2.linear_acceleration);
        assert_eq!(imu.timestamp_ns, imu2.timestamp_ns);
    }

    #[test]
    fn serde_roundtrip() {
        let cmd = CmdVel::new(1.5, -0.3, 12345);
        let json = serde_json::to_string(&cmd).unwrap();
        let cmd2: CmdVel = serde_json::from_str(&json).unwrap();
        assert_eq!(cmd, cmd2);
    }
}
