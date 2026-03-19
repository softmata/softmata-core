//! # softmata-core
//!
//! Foundational robotics types and traits for the SOFTMATA ecosystem.
//!
//! Provides two layers:
//! 1. **Concrete types** — geometry primitives (Pose2D, Vector3, etc.) shared across all projects
//! 2. **Sensor traits** — abstractions (ScanData, ImuData, etc.) bridging Pod IPC and standalone use
//!
//! ## Quick Start
//!
//! ```rust
//! use softmata_core::prelude::*;
//!
//! let pose = Pose2D::new(1.0, 2.0, 0.0);
//! let goal = Pose2D::new(5.0, 3.0, 1.57);
//! println!("distance: {:.2}m", pose.distance_to(&goal));
//! ```

#![cfg_attr(not(feature = "std"), no_std)]

#[cfg(not(feature = "std"))]
extern crate alloc;

pub mod conventions;
pub mod geometry;
pub mod messages;
pub mod sensor;
pub mod time;

pub mod prelude;
