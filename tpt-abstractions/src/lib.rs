//! # tpt-abstractions
//!
//! Core trait abstractions defining the contract between the
//! platform-independent flight control core and the platform-specific
//! backends (`spec.txt` §5). Every external dependency — hardware, OS,
//! sensors, actuators, and spatial maps — is accessed exclusively through
//! these traits, which keeps [`tpt_core`](https://docs.rs) portable across
//! the full vehicle spectrum and certification stacks.
//!
//! All traits are `#![no_std]`-compatible and use associated `Error` types so
//! backends can surface platform-specific failures without `alloc`.

#![no_std]
#![forbid(unsafe_code)]

pub mod actuators;
pub mod os;
pub mod sensors;
pub mod spatial;
pub mod types;

pub use actuators::{ControlSurface, Motor};
pub use os::{MemoryPool, PartitionChannel, PowerSystem, Scheduler};
pub use sensors::{Gnss, Imu, LidarSensor, RadarAltimeter, VisualSensor};
pub use spatial::{SpatialMap, TerrainDatabase};
pub use types::{
    BoundingBox, CameraIntrinsics, FixType, FrameMetadata, GeoPosition, Landmark, Point3D, Pose6DOF,
};
