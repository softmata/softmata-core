use super::{Pose3D, Twist3D};
use serde::de::{SeqAccess, Visitor};
use serde::ser::SerializeTuple;
use serde::{Deserialize, Deserializer, Serialize, Serializer};

fn serialize_cov<S: Serializer>(data: &[f64; 36], ser: S) -> Result<S::Ok, S::Error> {
    let mut tup = ser.serialize_tuple(36)?;
    for v in data {
        tup.serialize_element(v)?;
    }
    tup.end()
}

fn deserialize_cov<'de, D: Deserializer<'de>>(de: D) -> Result<[f64; 36], D::Error> {
    struct Cov36Visitor;
    impl<'de> Visitor<'de> for Cov36Visitor {
        type Value = [f64; 36];

        fn expecting(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
            write!(f, "an array of 36 f64 values")
        }

        fn visit_seq<A: SeqAccess<'de>>(self, mut seq: A) -> Result<[f64; 36], A::Error> {
            let mut arr = [0.0f64; 36];
            for (i, slot) in arr.iter_mut().enumerate() {
                *slot = seq
                    .next_element()?
                    .ok_or_else(|| serde::de::Error::invalid_length(i, &self))?;
            }
            Ok(arr)
        }
    }
    de.deserialize_tuple(36, Cov36Visitor)
}

/// 3D pose with 6x6 covariance matrix (row-major).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct PoseWithCovariance {
    pub pose: Pose3D,
    #[serde(serialize_with = "serialize_cov", deserialize_with = "deserialize_cov")]
    pub covariance: [f64; 36],
}

impl Default for PoseWithCovariance {
    fn default() -> Self {
        Self {
            pose: Pose3D::identity(),
            covariance: [0.0; 36],
        }
    }
}

impl PoseWithCovariance {
    pub fn new(pose: Pose3D, covariance: [f64; 36]) -> Self {
        Self { pose, covariance }
    }

    pub fn with_zero_covariance(pose: Pose3D) -> Self {
        Self {
            pose,
            covariance: [0.0; 36],
        }
    }
}

/// 3D twist with 6x6 covariance matrix (row-major).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct TwistWithCovariance {
    pub twist: Twist3D,
    #[serde(serialize_with = "serialize_cov", deserialize_with = "deserialize_cov")]
    pub covariance: [f64; 36],
}

impl Default for TwistWithCovariance {
    fn default() -> Self {
        Self {
            twist: Twist3D::zero(),
            covariance: [0.0; 36],
        }
    }
}

impl TwistWithCovariance {
    pub fn new(twist: Twist3D, covariance: [f64; 36]) -> Self {
        Self { twist, covariance }
    }
}
