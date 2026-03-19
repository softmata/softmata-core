//! Canonical names for topics, frames, and encodings.
//!
//! All Softmata projects should use these constants instead of hardcoding
//! strings. This ensures components discover each other automatically.

/// Canonical HORUS topic names. No slashes — plain identifiers.
pub mod topics {
    /// 2D laser scan data.
    pub const SCAN: &str = "scan";
    /// IMU readings (accelerometer + gyroscope + orientation).
    pub const IMU: &str = "imu";
    /// Odometry (pose + velocity from wheel encoders / visual odometry).
    pub const ODOM: &str = "odom";
    /// Velocity command for differential-drive robots.
    pub const CMD_VEL: &str = "cmd_vel";
    /// 3D point cloud.
    pub const POINTS: &str = "points";
    /// Robot pose estimate (from SLAM / localization).
    pub const POSE: &str = "pose";
    /// Emergency stop command/status.
    pub const ESTOP: &str = "emergency_stop";
    /// Component diagnostic status.
    pub const DIAGNOSTICS: &str = "diagnostics";
    /// Joint state (position, velocity, effort).
    pub const JOINT_STATES: &str = "joint_states";
    /// Depth image from depth camera.
    pub const DEPTH: &str = "depth";
    /// RGB camera image.
    pub const IMAGE: &str = "image";
    /// System clock (sim time, replay time).
    pub const CLOCK: &str = "clock";
    /// Occupancy grid map.
    pub const MAP: &str = "map";
    /// Node heartbeat for liveness monitoring.
    pub const HEARTBEAT: &str = "heartbeat";
    /// Navigation goal (target pose).
    pub const NAV_GOAL: &str = "nav_goal";
    /// Navigation status (arrived, blocked, navigating).
    pub const NAV_STATUS: &str = "nav_status";
    /// GPS / satellite fix.
    pub const GPS: &str = "gps";
    /// Battery state.
    pub const BATTERY: &str = "battery";
    /// Joint commands (target positions/velocities for actuators).
    pub const JOINT_CMD: &str = "joint_cmd";
    /// Motion planning request.
    pub const PLAN_REQUEST: &str = "plan_request";
    /// Planned trajectory output.
    pub const TRAJECTORY: &str = "trajectory";
    /// Twist command for servo control.
    pub const TWIST_CMD: &str = "twist_cmd";
}

/// Standard coordinate frame names following REP-105 conventions.
///
/// These define the standard transform tree for mobile robots:
/// `map → odom → base_link → sensor_frames`
pub mod frames {
    /// World-fixed frame for mapping / global localization.
    pub const MAP: &str = "map";
    /// Odometry frame (continuous but drifts over time).
    pub const ODOM: &str = "odom";
    /// Robot body center frame.
    pub const BASE_LINK: &str = "base_link";
    /// Robot footprint projected onto ground plane.
    pub const BASE_FOOTPRINT: &str = "base_footprint";
    /// Primary LiDAR sensor frame.
    pub const LASER: &str = "laser";
    /// RGB camera frame.
    pub const CAMERA_RGB: &str = "camera_rgb";
    /// Depth camera frame.
    pub const CAMERA_DEPTH: &str = "camera_depth";
    /// IMU sensor frame.
    pub const IMU_LINK: &str = "imu_link";
    /// GPS antenna frame.
    pub const GPS_LINK: &str = "gps_link";
    /// End-effector frame (for manipulators).
    pub const END_EFFECTOR: &str = "end_effector";
    /// Tool frame (for tool-center-point).
    pub const TOOL: &str = "tool";
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn topic_names_are_valid_identifiers() {
        // No slashes, no spaces, no special characters
        for name in [
            topics::SCAN, topics::IMU, topics::ODOM, topics::CMD_VEL,
            topics::POINTS, topics::POSE, topics::ESTOP, topics::DIAGNOSTICS,
            topics::JOINT_STATES, topics::DEPTH, topics::IMAGE, topics::CLOCK,
            topics::MAP, topics::HEARTBEAT, topics::NAV_GOAL, topics::NAV_STATUS,
            topics::GPS, topics::BATTERY, topics::JOINT_CMD,
            topics::PLAN_REQUEST, topics::TRAJECTORY, topics::TWIST_CMD,
        ] {
            assert!(!name.is_empty());
            assert!(!name.contains('/'));
            assert!(!name.contains(' '));
        }
    }

    #[test]
    fn frame_names_are_valid() {
        for name in [
            frames::MAP, frames::ODOM, frames::BASE_LINK, frames::BASE_FOOTPRINT,
            frames::LASER, frames::CAMERA_RGB, frames::CAMERA_DEPTH,
            frames::IMU_LINK, frames::GPS_LINK, frames::END_EFFECTOR, frames::TOOL,
        ] {
            assert!(!name.is_empty());
            assert!(!name.contains('/'));
        }
    }

    #[test]
    fn no_duplicate_topic_names() {
        let names = [
            topics::SCAN, topics::IMU, topics::ODOM, topics::CMD_VEL,
            topics::POINTS, topics::POSE, topics::ESTOP, topics::DIAGNOSTICS,
            topics::JOINT_STATES, topics::DEPTH, topics::IMAGE, topics::CLOCK,
            topics::MAP, topics::HEARTBEAT, topics::NAV_GOAL, topics::NAV_STATUS,
            topics::GPS, topics::BATTERY, topics::JOINT_CMD,
            topics::PLAN_REQUEST, topics::TRAJECTORY, topics::TWIST_CMD,
        ];
        for (i, a) in names.iter().enumerate() {
            for (j, b) in names.iter().enumerate() {
                if i != j {
                    assert_ne!(a, b, "Duplicate topic name: {}", a);
                }
            }
        }
    }
}
