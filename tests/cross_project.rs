//! Cross-project integration tests for softmata-core foundation types.
//!
//! Verifies that:
//! 1. Sensor traits work with different backing storages (Pod arrays, Vec, custom)
//! 2. Geometry types are the single canonical source
//! 3. Standalone use without HORUS/Terra is fully functional

use softmata_core::geometry::{Pose2D, Pose3D, Quaternion, Twist2D, Vector3, Wrench};
use softmata_core::sensor::{
    BatteryData, ImuData, MagneticFieldData, OdometryData, PointCloudData, RangeData, ScanData,
    TemperatureData, WrenchData,
};

// =============================================================================
// Mock types simulating different project storage strategies
// =============================================================================

/// Simulates a HORUS-style Pod type: fixed-size array, repr(C), no heap.
#[repr(C)]
struct PodLaserScan {
    angle_min: f32,
    angle_max: f32,
    angle_increment: f32,
    range_min: f32,
    range_max: f32,
    ranges: [f32; 360],
    valid_count: u32,
}

impl ScanData for PodLaserScan {
    fn num_ranges(&self) -> usize {
        self.valid_count as usize
    }
    fn range_at(&self, index: usize) -> f64 {
        f64::from(self.ranges[index])
    }
    fn angle_min(&self) -> f64 {
        f64::from(self.angle_min)
    }
    fn angle_max(&self) -> f64 {
        f64::from(self.angle_max)
    }
    fn angle_increment(&self) -> f64 {
        f64::from(self.angle_increment)
    }
    fn range_min(&self) -> f64 {
        f64::from(self.range_min)
    }
    fn range_max(&self) -> f64 {
        f64::from(self.range_max)
    }
}

/// Simulates a Terra-style Vec-backed type: heap-allocated, variable length.
struct VecLaserScan {
    ranges: Vec<f64>,
    angle_min: f64,
    angle_max: f64,
    angle_increment: f64,
}

impl ScanData for VecLaserScan {
    fn num_ranges(&self) -> usize {
        self.ranges.len()
    }
    fn range_at(&self, index: usize) -> f64 {
        self.ranges[index]
    }
    fn angle_min(&self) -> f64 {
        self.angle_min
    }
    fn angle_max(&self) -> f64 {
        self.angle_max
    }
    fn angle_increment(&self) -> f64 {
        self.angle_increment
    }
    fn range_min(&self) -> f64 {
        0.0
    }
    fn range_max(&self) -> f64 {
        f64::INFINITY
    }
}

/// Simulates a Pod IMU (fixed-size arrays, no heap).
#[repr(C)]
struct PodImu {
    orientation: [f64; 4],
    angular_velocity: [f64; 3],
    linear_acceleration: [f64; 3],
}

impl ImuData for PodImu {
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

/// Simulates a Terra-style f32 IMU (widened to f64 by trait impl).
struct F32Imu {
    accel_x: f32,
    accel_y: f32,
    accel_z: f32,
    gyro_x: f32,
    gyro_y: f32,
    gyro_z: f32,
}

impl ImuData for F32Imu {
    fn orientation(&self) -> [f64; 4] {
        [0.0, 0.0, 0.0, 1.0]
    }
    fn angular_velocity(&self) -> [f64; 3] {
        [
            f64::from(self.gyro_x),
            f64::from(self.gyro_y),
            f64::from(self.gyro_z),
        ]
    }
    fn linear_acceleration(&self) -> [f64; 3] {
        [
            f64::from(self.accel_x),
            f64::from(self.accel_y),
            f64::from(self.accel_z),
        ]
    }
    fn has_orientation(&self) -> bool {
        false
    }
}

// =============================================================================
// Generic processing functions (what algorithm crates would define)
// =============================================================================

/// Generic scan processor — works with ANY ScanData impl.
fn count_valid_ranges(scan: &impl ScanData) -> usize {
    (0..scan.num_ranges())
        .filter(|&i| scan.is_range_valid(i))
        .count()
}

/// Generic scan to cartesian — works with ANY ScanData impl.
fn scan_to_cartesian(scan: &impl ScanData) -> Vec<(f64, f64)> {
    (0..scan.num_ranges())
        .filter(|&i| scan.is_range_valid(i))
        .map(|i| {
            let angle = scan.angle_at(i);
            let range = scan.range_at(i);
            (range * angle.cos(), range * angle.sin())
        })
        .collect()
}

/// Generic IMU gravity vector extraction.
fn gravity_vector(imu: &impl ImuData) -> [f64; 3] {
    imu.linear_acceleration()
}

/// Generic wrench magnitude computation.
fn total_wrench_magnitude(w: &impl WrenchData) -> f64 {
    (w.force_magnitude().powi(2) + w.torque_magnitude().powi(2)).sqrt()
}

// =============================================================================
// TEST 1: Pod (HORUS-style) scan through generic function
// =============================================================================

#[test]
fn test_pod_scan_through_generic_function() {
    let mut scan = PodLaserScan {
        angle_min: -std::f32::consts::PI,
        angle_max: std::f32::consts::PI,
        angle_increment: 2.0 * std::f32::consts::PI / 360.0,
        range_min: 0.1,
        range_max: 10.0,
        ranges: [0.0; 360],
        valid_count: 360,
    };
    // Fill with valid ranges
    for i in 0..360 {
        scan.ranges[i] = 2.0;
    }

    let valid = count_valid_ranges(&scan);
    assert_eq!(valid, 360);

    let points = scan_to_cartesian(&scan);
    assert_eq!(points.len(), 360);
    // First point should be at angle_min with range 2.0
    let (x, y) = points[0];
    let expected_angle = f64::from(scan.angle_min);
    assert!((x - 2.0 * expected_angle.cos()).abs() < 1e-5);
    assert!((y - 2.0 * expected_angle.sin()).abs() < 1e-5);
}

// =============================================================================
// TEST 2: Vec (Terra-style) scan through SAME generic function
// =============================================================================

#[test]
fn test_vec_scan_through_same_generic_function() {
    let scan = VecLaserScan {
        ranges: vec![1.0, 2.0, 3.0, f64::INFINITY, 0.5],
        angle_min: 0.0,
        angle_max: 1.0,
        angle_increment: 0.25,
    };

    // f64::INFINITY is not valid (not finite)
    let valid = count_valid_ranges(&scan);
    assert_eq!(valid, 4);

    let points = scan_to_cartesian(&scan);
    assert_eq!(points.len(), 4);
}

// =============================================================================
// TEST 3: Both Pod and Vec scans through the SAME function — polymorphism works
// =============================================================================

#[test]
fn test_polymorphic_scan_processing() {
    let mut pod_scan = PodLaserScan {
        angle_min: 0.0,
        angle_max: 1.0,
        angle_increment: 0.5,
        range_min: 0.05,
        range_max: 10.0,
        ranges: [0.0; 360],
        valid_count: 2,
    };
    pod_scan.ranges[0] = 1.0;
    pod_scan.ranges[1] = 2.0;

    let vec_scan = VecLaserScan {
        ranges: vec![1.0, 2.0],
        angle_min: 0.0,
        angle_max: 1.0,
        angle_increment: 0.5,
    };

    // Same generic function, different backing types
    let pod_valid = count_valid_ranges(&pod_scan);
    let vec_valid = count_valid_ranges(&vec_scan);
    assert_eq!(pod_valid, vec_valid);

    let pod_points = scan_to_cartesian(&pod_scan);
    let vec_points = scan_to_cartesian(&vec_scan);

    // Results should be identical (within floating point tolerance)
    assert_eq!(pod_points.len(), vec_points.len());
    for (p, v) in pod_points.iter().zip(vec_points.iter()) {
        assert!((p.0 - v.0).abs() < 1e-5, "x mismatch: {} vs {}", p.0, v.0);
        assert!((p.1 - v.1).abs() < 1e-5, "y mismatch: {} vs {}", p.1, v.1);
    }
}

// =============================================================================
// TEST 4: Pod and f32 IMU through same generic function
// =============================================================================

#[test]
fn test_imu_trait_polymorphism() {
    let pod_imu = PodImu {
        orientation: [0.0, 0.0, 0.0, 1.0],
        angular_velocity: [0.1, 0.2, 0.3],
        linear_acceleration: [0.0, 0.0, 9.81],
    };

    let f32_imu = F32Imu {
        accel_x: 0.0,
        accel_y: 0.0,
        accel_z: 9.81,
        gyro_x: 0.1,
        gyro_y: 0.2,
        gyro_z: 0.3,
    };

    let pod_gravity = gravity_vector(&pod_imu);
    let f32_gravity = gravity_vector(&f32_imu);

    assert!((pod_gravity[2] - f32_gravity[2]).abs() < 1e-5);
    assert!((pod_gravity[2] - 9.81).abs() < 1e-5);

    // Both should report same angular velocity
    assert!((pod_imu.angular_velocity()[0] - f32_imu.angular_velocity()[0]).abs() < 1e-5);
}

// =============================================================================
// TEST 5: Pose2D is the same canonical type — no conversion needed
// =============================================================================

#[test]
fn test_pose2d_identity_across_usage() {
    // Create a Pose2D — this is the SAME type everywhere
    let pose = Pose2D::new(1.0, 2.0, std::f64::consts::FRAC_PI_2);

    // Use geometry operations
    let goal = Pose2D::new(5.0, 3.0, 0.0);
    let distance = pose.distance_to(&goal);
    assert!(distance > 4.0);
    assert!(distance < 5.0);

    // Pose2D can be passed to any function expecting it — no From/Into needed
    fn accept_pose(p: &Pose2D) -> f64 {
        p.x + p.y
    }
    assert!((accept_pose(&pose) - 3.0).abs() < 1e-10);
}

// =============================================================================
// TEST 6: Standalone use — no HORUS dependency needed
// =============================================================================

#[test]
fn test_standalone_no_horus_dependency() {
    // Create all types from softmata-core alone
    let pose = Pose2D::new(0.0, 0.0, 0.0);
    let _twist = Twist2D::new(1.0, 0.5);
    let _vec3 = Vector3::new(1.0, 2.0, 3.0);
    let _quat = Quaternion::identity();
    let _pose3d = Pose3D::identity();
    let _wrench = Wrench::new(Vector3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 0.1));

    // Use a Vec-backed scan (standalone, no HORUS Pod types)
    let scan = VecLaserScan {
        ranges: vec![1.0; 100],
        angle_min: -std::f64::consts::PI,
        angle_max: std::f64::consts::PI,
        angle_increment: 2.0 * std::f64::consts::PI / 100.0,
    };

    let valid = count_valid_ranges(&scan);
    assert_eq!(valid, 100);

    let points = scan_to_cartesian(&scan);
    assert_eq!(points.len(), 100);

    // Verify pose operations work standalone
    let goal = Pose2D::new(10.0, 0.0, 0.0);
    assert!((pose.distance_to(&goal) - 10.0).abs() < 1e-10);
}

// =============================================================================
// TEST 7: WrenchData trait with softmata-core Wrench type
// =============================================================================

#[test]
fn test_wrench_trait_with_core_type() {
    let wrench = Wrench::new(Vector3::new(3.0, 4.0, 0.0), Vector3::new(0.0, 0.0, 1.0));

    // Use trait methods
    let force_mag = wrench.force_magnitude();
    assert!((force_mag - 5.0).abs() < 1e-10); // 3-4-5 triangle

    let torque_mag = wrench.torque_magnitude();
    assert!((torque_mag - 1.0).abs() < 1e-10);

    // Generic function works
    let total = total_wrench_magnitude(&wrench);
    assert!(total > 5.0);
}

// =============================================================================
// TEST 8: OdometryData trait with custom impl
// =============================================================================

#[test]
fn test_odometry_trait_polymorphism() {
    struct SimpleOdom {
        x: f64,
        y: f64,
        theta: f64,
        v: f64,
        w: f64,
    }

    impl OdometryData for SimpleOdom {
        fn pose(&self) -> Pose2D {
            Pose2D::new(self.x, self.y, self.theta)
        }
        fn linear_velocity(&self) -> f64 {
            self.v
        }
        fn angular_velocity(&self) -> f64 {
            self.w
        }
    }

    let odom = SimpleOdom {
        x: 1.0,
        y: 2.0,
        theta: 0.5,
        v: 1.0,
        w: 0.1,
    };

    fn compute_stopping_distance(o: &impl OdometryData, decel: f64) -> f64 {
        let v = o.linear_velocity();
        v * v / (2.0 * decel)
    }

    let stop_dist = compute_stopping_distance(&odom, 1.0);
    assert!((stop_dist - 0.5).abs() < 1e-10);
    assert!((odom.pose().x - 1.0).abs() < 1e-10);
}

// =============================================================================
// TEST 9: PointCloudData trait — Pod vs Vec backing
// =============================================================================

#[test]
fn test_point_cloud_trait_polymorphism() {
    struct PodCloud {
        points: [[f32; 3]; 4],
        count: usize,
    }

    impl PointCloudData for PodCloud {
        fn num_points(&self) -> usize {
            self.count
        }
        fn point_at(&self, index: usize) -> [f32; 3] {
            self.points[index]
        }
    }

    struct VecCloud {
        points: Vec<[f32; 3]>,
    }

    impl PointCloudData for VecCloud {
        fn num_points(&self) -> usize {
            self.points.len()
        }
        fn point_at(&self, index: usize) -> [f32; 3] {
            self.points[index]
        }
    }

    fn centroid(cloud: &impl PointCloudData) -> [f32; 3] {
        let n = cloud.num_points() as f32;
        let mut sum = [0.0f32; 3];
        for i in 0..cloud.num_points() {
            let p = cloud.point_at(i);
            sum[0] += p[0];
            sum[1] += p[1];
            sum[2] += p[2];
        }
        [sum[0] / n, sum[1] / n, sum[2] / n]
    }

    let pod = PodCloud {
        points: [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
            [1.0, 1.0, 1.0],
        ],
        count: 4,
    };

    let vec_cloud = VecCloud {
        points: vec![
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
            [1.0, 1.0, 1.0],
        ],
    };

    let pod_centroid = centroid(&pod);
    let vec_centroid = centroid(&vec_cloud);

    for i in 0..3 {
        assert!(
            (pod_centroid[i] - vec_centroid[i]).abs() < 1e-5,
            "centroid[{i}] mismatch"
        );
        assert!((pod_centroid[i] - 0.5).abs() < 1e-5);
    }
}

// =============================================================================
// TEST 10: Geometry type conversion roundtrips
// =============================================================================

#[test]
fn test_geometry_nalgebra_bridge() {
    let v = Vector3::new(1.0, 2.0, 3.0);
    let na_v: nalgebra::Vector3<f64> = v.to_nalgebra();
    let back = Vector3::from_nalgebra(&na_v);
    assert!((back.x - v.x).abs() < 1e-10);
    assert!((back.y - v.y).abs() < 1e-10);
    assert!((back.z - v.z).abs() < 1e-10);

    let q = Quaternion::new(0.0, 0.0, 0.3826834, 0.9238795); // ~45° yaw
    let na_q = q.to_nalgebra();
    let back_q = Quaternion::from_nalgebra(&na_q);
    assert!((back_q.w - q.w).abs() < 1e-5);
    assert!((back_q.z - q.z).abs() < 1e-5);
}

// =============================================================================
// TEST 11: Tier 2 sensor traits work standalone
// =============================================================================

#[test]
fn test_tier2_sensor_traits_standalone() {
    struct MockBattery;
    impl BatteryData for MockBattery {
        fn voltage(&self) -> f32 {
            12.6
        }
        fn current(&self) -> f32 {
            -1.5
        }
        fn percentage(&self) -> f32 {
            85.0
        }
    }

    struct MockRange;
    impl RangeData for MockRange {
        fn range(&self) -> f64 {
            1.5
        }
        fn range_min(&self) -> f64 {
            0.02
        }
        fn range_max(&self) -> f64 {
            4.0
        }
    }

    struct MockTemp;
    impl TemperatureData for MockTemp {
        fn celsius(&self) -> f32 {
            25.0
        }
    }

    struct MockMag;
    impl MagneticFieldData for MockMag {
        fn magnetic_field(&self) -> [f32; 3] {
            [25.0, -10.0, 45.0]
        }
    }

    fn battery_health(b: &impl BatteryData) -> &'static str {
        if b.percentage() > 50.0 {
            "healthy"
        } else {
            "low"
        }
    }

    assert_eq!(battery_health(&MockBattery), "healthy");
    assert!(MockRange.is_valid());
    assert!((MockTemp.fahrenheit() - 77.0).abs() < 1e-2);
    assert!((MockMag.magnetic_field()[2] - 45.0).abs() < 1e-5);
}
