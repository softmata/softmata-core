//! Canonical Pod message types for cross-project IPC.
//!
//! All types are `#[repr(C)]`, `Copy`, `Pod`, `Zeroable` — safe for zero-copy
//! shared memory transport via HORUS Topic<T>. Also `Serialize`/`Deserialize`
//! for standalone use without HORUS.
//!
//! These are THE message types for the Softmata ecosystem. Every project
//! imports them from here. No project defines its own sensor messages.

pub mod sensor;
pub mod descriptors;
pub mod diagnostics;

pub use sensor::*;
pub use descriptors::*;
pub use diagnostics::*;
