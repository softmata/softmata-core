# Softmata Ecosystem Integration Guide

## Overview

`softmata-core` is the shared type foundation for all Softmata robotics projects. Every project imports canonical Pod message types from here, enabling zero-copy IPC via HORUS and standalone usage without any middleware.

## Canonical Message Types

All types are `#[repr(C)]`, `Copy`, `Pod`, `Zeroable` — safe for zero-copy shared memory transport.

### Sensor Messages (`softmata_core::messages`)

| Type | Fields | Size |
|------|--------|------|
| `ImuReading` | linear_acceleration [f64;3], angular_velocity [f64;3], orientation [f64;4], timestamp_ns u64 | 88B |
| `CmdVel` | linear f64, angular f64, timestamp_ns u64 | 24B |
| `Odometry` | x, y, theta, linear_velocity, angular_velocity (all f64), timestamp_ns u64 | 48B |
| `JointState` | position/velocity/effort [f64;32] each, num_joints u32, timestamp_ns u64 | 784B |
| `BatteryState` | voltage, current, percentage (f64), charging u8, timestamp_ns u64 | 40B |
| `NavSatFix` | latitude, longitude, altitude, accuracy (f64), fix_type u8, timestamp_ns u64 | 48B |

### Diagnostic Messages

| Type | Fields | Size |
|------|--------|------|
| `EmergencyStop` | engaged u8, reason [u8;64], source [u8;32], timestamp_ns u64 | 112B |
| `DiagnosticStatus` | level u8, code u32, message [u8;128], component [u8;32], timestamp_ns u64 | 176B |
| `Heartbeat` | node_name [u8;32], node_id u32, sequence u64, alive u8, timestamp_ns u64 | 64B |

### Descriptors (for large data in shared memory pools)

| Type | Purpose |
|------|---------|
| `LaserScanDesc` | Metadata for laser scan; range data lives in pool |
| `PointCloudDesc` | Metadata for point cloud; point data lives in pool |
| `ImageDesc` | Metadata for camera image; pixel data lives in pool |
| `DepthImageDesc` | Metadata for depth image with camera intrinsics |

## Canonical Topic Names

All topics use plain identifiers — no slashes, no prefixes. Defined in `softmata_core::conventions::topics`.

| Constant | Value | Message Type |
|----------|-------|-------------|
| `SCAN` | `"scan"` | LaserScanDesc |
| `IMU` | `"imu"` | ImuReading |
| `ODOM` | `"odom"` | Odometry |
| `CMD_VEL` | `"cmd_vel"` | CmdVel |
| `POINTS` | `"points"` | PointCloudDesc |
| `POSE` | `"pose"` | Odometry / Pose2D |
| `ESTOP` | `"emergency_stop"` | EmergencyStop |
| `DIAGNOSTICS` | `"diagnostics"` | DiagnosticStatus |
| `JOINT_STATES` | `"joint_states"` | JointState |
| `DEPTH` | `"depth"` | DepthImageDesc |
| `IMAGE` | `"image"` | ImageDesc |
| `MAP` | `"map"` | OccupancyGrid |
| `HEARTBEAT` | `"heartbeat"` | Heartbeat |
| `GPS` | `"gps"` | NavSatFix |

## Standard Frame Names

Defined in `softmata_core::conventions::frames`. Following REP-105:

```
map → odom → base_link → sensor_frames (laser, camera_rgb, camera_depth, imu_link)
```

## Usage Patterns

### Standalone (no HORUS)

```rust
use softmata_core::prelude::*;

let imu = ImuReading::new([0.0, 0.0, 9.81], [0.0, 0.0, 0.1], [0.0, 0.0, 0.0, 1.0], 0);
// Use directly in any project — same struct everywhere
```

### With HORUS IPC

```rust
use softmata_core::prelude::*;
use softmata_core::conventions::topics;
use horus_core::communication::Topic;

// Publish — zero-copy, Pod types detected automatically
let topic: Topic<ImuReading> = Topic::new(topics::IMU)?;
topic.send(&imu);

// Subscribe (different process) — same type, zero-copy
let topic: Topic<ImuReading> = Topic::new(topics::IMU)?;
if let Some(imu) = topic.recv() {
    // imu is &ImuReading — no deserialization
}
```

### Cross-Project (e.g., Terra → Ferroslam)

```rust
// In Terra driver node:
use softmata_core::prelude::*;
use terra_core::compat; // From impls

let terra_imu = read_from_hardware();
let canonical: ImuReading = (&terra_imu).into();  // terra → canonical
topic.send(&canonical);

// In Ferroslam SLAM node (different process):
use ferroslam::compat; // From impls

if let Some(canonical) = topic.recv() {
    let fs_imu: ImuData = canonical.into();  // canonical → ferroslam
    slam.update_imu(&fs_imu);
}
```

## Project Compatibility Modules

Each project has a `compat.rs` (or `compat` module) with `From` impls:

| Project | Module | Feature Flag | Key Conversions |
|---------|--------|-------------|-----------------|
| terra-core | `src/compat.rs` | always | ImuReading, OdometryReading |
| pathfinder_core | `src/compat.rs` | always | LaserScan ↔ LaserScanDesc |
| ferroslam | `src/compat.rs` | `softmata` (default) | ImuData, LidarScan |
| aegis | `src/compat.rs` | `softmata` (optional) | Pose2D generic ↔ concrete |
| horus_library | `messages/compat.rs` | always | All HORUS ↔ canonical types |
