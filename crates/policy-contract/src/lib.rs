//! The `/act` contract `v0`, schema version 3, as `API.md` describes it: what the policy server and
//! the tools that observe or command the robot for it must agree on.

pub mod gripper;
pub mod image;
pub mod kinematics;
mod strict_json;
pub mod wire;

use std::time::Duration;

pub const SCHEMA_VERSION: u32 = 3;
pub const STEP: Duration = Duration::from_millis(50);

/// Joint positions in the contract's order: shoulder pan, shoulder lift, elbow, wrist 1, 2, 3.
pub type Joints = [f64; 6];

/// The arm's joint names, in the contract's order.
pub const ARM_JOINTS: [&str; 6] = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
];
