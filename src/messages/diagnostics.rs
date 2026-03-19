//! Pod diagnostic message types for safety and health monitoring.

use bytemuck::{Pod, Zeroable};
use serde::{Deserialize, Serialize};
use serde_big_array::BigArray;

// =============================================================================
// Emergency Stop
// =============================================================================

/// Emergency stop command/status.
///
/// When `engaged` is non-zero, ALL actuators must halt immediately.
/// Published on canonical topic "emergency_stop". Every bridge must subscribe.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Pod, Zeroable)]
#[repr(C)]
pub struct EmergencyStop {
    /// 0 = released, 1 = engaged.
    pub engaged: u8,
    /// Whether auto-reset is allowed: 0 = manual reset required, 1 = auto-reset ok.
    pub auto_reset: u8,
    /// Padding for alignment.
    pub _pad: [u8; 6],
    /// Human-readable reason (null-terminated UTF-8).
    #[serde(with = "BigArray")]
    pub reason: [u8; 64],
    /// Source identifier (null-terminated UTF-8).
    pub source: [u8; 32],
    /// Timestamp in nanoseconds since epoch.
    pub timestamp_ns: u64,
}

impl EmergencyStop {
    pub fn engage(reason: &str, source: &str, timestamp_ns: u64) -> Self {
        let mut msg = Self {
            engaged: 1,
            auto_reset: 0,
            _pad: [0; 6],
            reason: [0; 64],
            source: [0; 32],
            timestamp_ns,
        };
        let reason_bytes = reason.as_bytes();
        let len = reason_bytes.len().min(63);
        msg.reason[..len].copy_from_slice(&reason_bytes[..len]);
        let source_bytes = source.as_bytes();
        let slen = source_bytes.len().min(31);
        msg.source[..slen].copy_from_slice(&source_bytes[..slen]);
        msg
    }

    pub fn release(source: &str, timestamp_ns: u64) -> Self {
        let mut msg = Self {
            engaged: 0,
            auto_reset: 0,
            _pad: [0; 6],
            reason: [0; 64],
            source: [0; 32],
            timestamp_ns,
        };
        let source_bytes = source.as_bytes();
        let slen = source_bytes.len().min(31);
        msg.source[..slen].copy_from_slice(&source_bytes[..slen]);
        msg
    }

    pub fn is_engaged(&self) -> bool {
        self.engaged != 0
    }

    pub fn reason_str(&self) -> &str {
        let end = self.reason.iter().position(|&b| b == 0).unwrap_or(64);
        core::str::from_utf8(&self.reason[..end]).unwrap_or("")
    }

    pub fn source_str(&self) -> &str {
        let end = self.source.iter().position(|&b| b == 0).unwrap_or(32);
        core::str::from_utf8(&self.source[..end]).unwrap_or("")
    }
}

// =============================================================================
// Diagnostic Status
// =============================================================================

/// Diagnostic severity levels.
pub const DIAG_OK: u8 = 0;
pub const DIAG_WARN: u8 = 1;
pub const DIAG_ERROR: u8 = 2;
pub const DIAG_FATAL: u8 = 3;

/// Component-level diagnostic status.
///
/// Published on canonical topic "diagnostics". Every bridge should publish
/// these when errors occur.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Pod, Zeroable)]
#[repr(C)]
pub struct DiagnosticStatus {
    /// Severity level: 0=OK, 1=WARN, 2=ERROR, 3=FATAL.
    pub level: u8,
    /// Padding for alignment.
    pub _pad1: [u8; 3],
    /// Error/status code (project-specific).
    pub code: u32,
    /// Human-readable message (null-terminated UTF-8).
    #[serde(with = "BigArray")]
    pub message: [u8; 128],
    /// Component name (null-terminated UTF-8).
    pub component: [u8; 32],
    /// Timestamp in nanoseconds since epoch.
    pub timestamp_ns: u64,
}

impl DiagnosticStatus {
    fn with_level(level: u8, component: &str, message: &str, timestamp_ns: u64) -> Self {
        let mut msg = Self {
            level,
            _pad1: [0; 3],
            code: 0,
            message: [0; 128],
            component: [0; 32],
            timestamp_ns,
        };
        let msg_bytes = message.as_bytes();
        let mlen = msg_bytes.len().min(127);
        msg.message[..mlen].copy_from_slice(&msg_bytes[..mlen]);
        let comp_bytes = component.as_bytes();
        let clen = comp_bytes.len().min(31);
        msg.component[..clen].copy_from_slice(&comp_bytes[..clen]);
        msg
    }

    pub fn ok(component: &str, message: &str, timestamp_ns: u64) -> Self {
        Self::with_level(DIAG_OK, component, message, timestamp_ns)
    }

    pub fn warn(component: &str, message: &str, timestamp_ns: u64) -> Self {
        Self::with_level(DIAG_WARN, component, message, timestamp_ns)
    }

    pub fn error(component: &str, message: &str, timestamp_ns: u64) -> Self {
        Self::with_level(DIAG_ERROR, component, message, timestamp_ns)
    }

    pub fn fatal(component: &str, message: &str, timestamp_ns: u64) -> Self {
        Self::with_level(DIAG_FATAL, component, message, timestamp_ns)
    }

    pub fn with_code(mut self, code: u32) -> Self {
        self.code = code;
        self
    }

    pub fn message_str(&self) -> &str {
        let end = self.message.iter().position(|&b| b == 0).unwrap_or(128);
        core::str::from_utf8(&self.message[..end]).unwrap_or("")
    }

    pub fn component_str(&self) -> &str {
        let end = self.component.iter().position(|&b| b == 0).unwrap_or(32);
        core::str::from_utf8(&self.component[..end]).unwrap_or("")
    }

    pub fn is_ok(&self) -> bool {
        self.level == DIAG_OK
    }

    pub fn is_error_or_fatal(&self) -> bool {
        self.level >= DIAG_ERROR
    }
}

// =============================================================================
// Heartbeat
// =============================================================================

/// Node heartbeat for liveness detection.
///
/// Published periodically by every node. Watchdogs detect stale heartbeats.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Pod, Zeroable)]
#[repr(C)]
pub struct Heartbeat {
    /// Node name (null-terminated UTF-8).
    pub node_name: [u8; 32],
    /// Node ID.
    pub node_id: u32,
    /// 0 = shutting down, 1 = alive.
    pub alive: u8,
    /// Padding for alignment.
    pub _pad: [u8; 3],
    /// Monotonically increasing sequence number.
    pub sequence: u64,
    /// Uptime in nanoseconds.
    pub uptime_ns: u64,
    /// Timestamp in nanoseconds since epoch.
    pub timestamp_ns: u64,
}

impl Heartbeat {
    pub fn new(node_name: &str, node_id: u32, sequence: u64, timestamp_ns: u64) -> Self {
        let mut msg = Self {
            node_name: [0; 32],
            node_id,
            sequence,
            alive: 1,
            _pad: [0; 3],
            uptime_ns: 0,
            timestamp_ns,
        };
        let name_bytes = node_name.as_bytes();
        let nlen = name_bytes.len().min(31);
        msg.node_name[..nlen].copy_from_slice(&name_bytes[..nlen]);
        msg
    }

    pub fn node_name_str(&self) -> &str {
        let end = self.node_name.iter().position(|&b| b == 0).unwrap_or(32);
        core::str::from_utf8(&self.node_name[..end]).unwrap_or("")
    }

    pub fn is_alive(&self) -> bool {
        self.alive != 0
    }
}

// =============================================================================
// Tests
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn emergency_stop_engage_release() {
        let stop = EmergencyStop::engage("collision detected", "safety_monitor", 12345);
        assert!(stop.is_engaged());
        assert_eq!(stop.reason_str(), "collision detected");
        assert_eq!(stop.source_str(), "safety_monitor");

        let release = EmergencyStop::release("operator", 12346);
        assert!(!release.is_engaged());
    }

    #[test]
    fn diagnostic_status_levels() {
        let ok = DiagnosticStatus::ok("slam", "tracking stable", 0);
        assert!(ok.is_ok());
        assert!(!ok.is_error_or_fatal());

        let err = DiagnosticStatus::error("slam", "tracking lost", 0).with_code(42);
        assert!(err.is_error_or_fatal());
        assert_eq!(err.code, 42);
        assert_eq!(err.message_str(), "tracking lost");
        assert_eq!(err.component_str(), "slam");
    }

    #[test]
    fn heartbeat_basics() {
        let hb = Heartbeat::new("my_node", 7, 42, 999);
        assert_eq!(hb.node_name_str(), "my_node");
        assert_eq!(hb.node_id, 7);
        assert_eq!(hb.sequence, 42);
        assert!(hb.is_alive());
    }

    #[test]
    fn pod_zero_copy_estop() {
        let stop = EmergencyStop::engage("test", "unit_test", 42);
        let bytes = bytemuck::bytes_of(&stop);
        let stop2: &EmergencyStop = bytemuck::from_bytes(bytes);
        assert_eq!(stop.engaged, stop2.engaged);
        assert_eq!(stop.reason_str(), stop2.reason_str());
    }

    #[test]
    fn serde_diagnostic() {
        let diag = DiagnosticStatus::warn("nav", "path blocked", 123);
        let json = serde_json::to_string(&diag).unwrap();
        let diag2: DiagnosticStatus = serde_json::from_str(&json).unwrap();
        assert_eq!(diag, diag2);
    }
}
