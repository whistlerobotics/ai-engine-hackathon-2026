use crate::ros::{CAMERA_TOPIC, Ros};
use jiff::ZonedDateTime;
use policy_contract::{
    Joints, gripper,
    image::{FrameError, check_jpeg},
    kinematics,
};
use serde::Serialize;
use std::{fmt, time::Instant};

/// `policy_contract::wire::Proprio`, but with the gripper null where permissive recording had no
/// fresh reading of it.
#[derive(Debug, Clone, Serialize)]
pub struct RecordedProprio {
    pub joint_pos: Joints,
    pub joint_vel: Joints,
    pub gripper: Option<f64>,
    pub ee_pos: [f64; 3],
    pub ee_quat: [f64; 4],
}

#[derive(Debug, Clone)]
pub enum Image {
    Camera(Vec<u8>),
    /// In place of a missing camera frame, when recording permissively.
    Black,
}

#[derive(Debug, Clone)]
pub struct Observation {
    pub proprio: RecordedProprio,
    pub image: Image,
}

#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum Stale {
    #[error("no fresh /joint_states")]
    Arm,
    #[error("no fresh /gripper/joint_states")]
    Gripper,
    #[error("no fresh frame on {CAMERA_TOPIC}")]
    Camera,
    #[error(transparent)]
    Frame(#[from] FrameError),
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum End {
    Release,
    TimeLimit,
    Stale(Stale),
    HandGuidingStopped,
    Signal(&'static str),
}

impl fmt::Display for End {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Release => write!(f, "release"),
            Self::TimeLimit => write!(f, "time limit"),
            Self::Stale(stale) => write!(f, "{stale}"),
            Self::HandGuidingStopped => write!(f, "hand-guiding stopped"),
            Self::Signal(name) => write!(f, "{name}"),
        }
    }
}

#[derive(Debug, Clone)]
pub struct Step {
    /// Seconds since the episode started.
    pub t: f64,
    pub image: Image,
    pub proprio: RecordedProprio,
    /// The gripper opening last commanded.
    pub gripper_command: f64,
}

pub struct Episode {
    pub task_id: String,
    pub id: String,
    /// Local time, with its UTC offset.
    pub started_at: String,
    pub start: Instant,
    pub steps: Vec<Step>,
    pub steps_without_image: u32,
}

impl Episode {
    pub fn new(task_id: &str, start: Instant) -> Self {
        let now = ZonedDateTime::now();
        Self {
            task_id: task_id.to_owned(),
            id: now.strftime("ep_%Y%m%d_%H%M%S").to_string(),
            started_at: now.strftime("%Y-%m-%dT%H:%M:%S%.6f%:z").to_string(),
            start,
            steps: Vec::new(),
            steps_without_image: 0,
        }
    }

    pub fn add_step(&mut self, now: Instant, observation: Observation, commanded_finger: f64) {
        if matches!(observation.image, Image::Black) {
            self.steps_without_image += 1;
        }
        let t = (now - self.start).as_secs_f64();
        self.steps.push(Step {
            t: (t * 10_000.0).round() / 10_000.0,
            image: observation.image,
            proprio: observation.proprio,
            gripper_command: gripper::opening(commanded_finger),
        });
    }
}

/// What the robot looks like now, or why it cannot be recorded. Permissive recording puts up with
/// a missing camera frame or gripper reading, but never with a stale arm.
pub fn observe(ros: &Ros, now: Instant, permissive: bool) -> Result<Observation, Stale> {
    let arm = ros
        .arm()
        .filter(|arm| arm.is_fresh(now))
        .ok_or(Stale::Arm)?
        .value;
    let gripper = ros
        .finger()
        .filter(|finger| finger.is_fresh(now))
        .map(|finger| gripper::opening(finger.value));
    if gripper.is_none() && !permissive {
        return Err(Stale::Gripper);
    }
    let image = match camera_frame(ros, now) {
        Ok(jpeg) => Image::Camera(jpeg),
        Err(_) if permissive => Image::Black,
        Err(stale) => return Err(stale),
    };
    let palm = kinematics::palm_pose(&arm.position);
    Ok(Observation {
        proprio: RecordedProprio {
            joint_pos: arm.position,
            joint_vel: arm.velocity,
            gripper,
            ee_pos: palm.position,
            ee_quat: palm.orientation,
        },
        image,
    })
}

fn camera_frame(ros: &Ros, now: Instant) -> Result<Vec<u8>, Stale> {
    let frame = ros
        .camera()
        .filter(|frame| frame.is_fresh(now))
        .ok_or(Stale::Camera)?;
    check_jpeg(&frame.value)?;
    Ok(frame.value)
}

#[cfg(test)]
mod tests {
    use super::*;
    use policy_contract::wire::Proprio;

    #[test]
    fn recorded_proprio_matches_the_contract() -> serde_json::Result<()> {
        let joint_pos = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6];
        let palm = kinematics::palm_pose(&joint_pos);
        let recorded = RecordedProprio {
            joint_pos,
            joint_vel: [0.0; 6],
            gripper: Some(0.5),
            ee_pos: palm.position,
            ee_quat: palm.orientation,
        };
        let contract = Proprio {
            joint_pos,
            joint_vel: [0.0; 6],
            gripper: 0.5,
            ee_pos: palm.position,
            ee_quat: palm.orientation,
        };
        assert_eq!(
            serde_json::to_value(&recorded)?,
            serde_json::to_value(&contract)?
        );
        Ok(())
    }

    #[test]
    fn ids_and_times() {
        let episode = Episode::new("task", Instant::now());
        assert_eq!(episode.id.len(), "ep_20261125_140312".len());
        assert!(episode.id.starts_with("ep_"));
        // Such as 2026-11-25T14:03:12.123456+00:00.
        assert_eq!(episode.started_at.len(), 32, "{}", episode.started_at);
    }
}
