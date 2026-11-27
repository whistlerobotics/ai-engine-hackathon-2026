use crate::{
    app::{Command, Phase, Shared},
    episode::Episode,
    policy::{Health, Policy, Readiness},
    ros::{ArmState, Received, Ros},
    task::TaskPack,
};
use color_eyre::{
    Result,
    eyre::{bail, ensure, eyre},
};
use policy_contract::Joints;
use std::{
    sync::{atomic::Ordering, mpsc::Receiver},
    thread,
    time::{Duration, Instant},
};

pub const TOLERANCE: f64 = 0.02;
const START_SPEED: f64 = std::f64::consts::PI / 10.0;
const POLL: Duration = Duration::from_millis(10);
const SETTLE: Duration = Duration::from_millis(500);

pub fn serve(
    ros: &Ros,
    policy: &Policy,
    shared: &Shared,
    commands: &Receiver<Command>,
    pack: &TaskPack,
) -> Result<()> {
    for command in commands {
        let phase = match command {
            Command::MoveToStart => match move_to_start(ros, shared, pack.start) {
                Ok(()) => Phase::AtStart,
                Err(reason) => {
                    shared
                        .log
                        .push(format!("Move to start stopped: {reason:#}"));
                    Phase::Idle
                }
            },
            Command::RunEpisode { task_id } => {
                let setup = (|| -> Result<_> {
                    let task = pack.select(&task_id)?;
                    let status = policy.status();
                    ensure!(
                        status.health == Health::Up && status.readiness == Readiness::Ready,
                        "policy health {}, readiness {}",
                        status.health,
                        status.readiness
                    );
                    ensure!(
                        ros.controller_ready() && ros.gripper_ready(),
                        "mock controllers are not ready"
                    );
                    ensure!(
                        ros.arm()
                            .filter(Received::is_fresh)
                            .is_some_and(|a| max_error(a.value.position, pack.start) <= TOLERANCE),
                        "arm moved away from the selected start pose"
                    );
                    ensure!(!shared.cancel.load(Ordering::Relaxed), "operator cancelled");
                    ros.check_cameras()?;
                    Ok(task)
                })();
                match setup {
                    Ok(task) => {
                        shared.log.push(format!(
                            "Starting task={} revision={} variant={} seed={}",
                            task.id, pack.revision, task.variant, task.seed
                        ));
                        let report =
                            Episode::run(ros, policy, &task, &shared.cancel, &shared.progress);
                        let line = report.to_string();
                        shared.log.push(line.clone());
                        shared.reports.lock().push(line);
                    }
                    Err(reason) => shared.log.push(format!("Episode refused: {reason:#}")),
                }
                Phase::Idle
            }
        };
        shared.log.end_request();
        *shared.phase.lock() = phase;
    }
    Ok(())
}

pub fn max_error(a: Joints, b: Joints) -> f64 {
    a.into_iter()
        .zip(b)
        .map(|(x, y)| (x - y).abs())
        .fold(0.0, f64::max)
}

struct MovePlan {
    duration: Duration,
    target: Joints,
}

impl MovePlan {
    fn start(from: Joints, target: Joints) -> Self {
        let duration = Duration::from_secs_f64(max_error(from, target) / START_SPEED)
            .max(Duration::from_secs(1));
        Self { duration, target }
    }

    fn arrived(
        &self,
        now: Instant,
        sent: Instant,
        arm: Option<&Received<ArmState>>,
        reference: Option<&Received<Joints>>,
        cancelled: bool,
        controller: bool,
    ) -> Result<bool> {
        ensure!(!cancelled, "operator cancelled");
        ensure!(controller, "mock controller feedback or subscriber lost");
        let arm = arm
            .filter(|a| a.is_fresh())
            .ok_or_else(|| eyre!("missing or stale arm feedback during move"))?;
        let reference = reference
            .filter(|r| r.is_fresh())
            .ok_or_else(|| eyre!("missing or stale controller setpoint"))?;
        let error = max_error(arm.value.position, self.target);
        if now >= sent + self.duration
            && arm.at > sent
            && reference.at > sent
            && error <= TOLERANCE
            && max_error(reference.value, self.target) <= TOLERANCE
        {
            return Ok(true);
        }
        ensure!(
            now < sent + self.duration + SETTLE,
            "arrival timed out, target={:?} measured={:?} controller_reference={:?} error={error:.4} rad",
            self.target,
            arm.value.position,
            reference.value
        );
        Ok(false)
    }
}

fn move_to_start(ros: &Ros, shared: &Shared, target: Joints) -> Result<()> {
    ensure!(
        ros.controller_ready(),
        "mock description, active controller, isolated subscriber and fresh controller feedback required"
    );
    let arm = ros
        .arm()
        .filter(Received::is_fresh)
        .ok_or_else(|| eyre!("missing fresh arm state"))?;
    let movement = MovePlan::start(arm.value.position, target);
    let duration = movement.duration;
    ensure!(!shared.cancel.load(Ordering::Relaxed), "operator cancelled");
    let sent = Instant::now();
    ros.send_trajectory(&[target], duration)?;
    shared.log.push(format!(
        "Move published target={target:?} from={:?} duration={duration:?}",
        arm.value.position
    ));
    let result = wait_for_start(ros, shared, &movement, sent);
    if result.is_err()
        && let Err(error) = hold_verified(ros, false)
    {
        shared.log.push(format!("Hold unconfirmed: {error:#}"));
    }
    result
}

fn wait_for_start(ros: &Ros, shared: &Shared, movement: &MovePlan, sent: Instant) -> Result<()> {
    let target = movement.target;
    let mut last = sent;
    loop {
        let arm = ros.arm();
        let reference = ros.reference();
        let arrived = movement.arrived(
            Instant::now(),
            sent,
            arm.as_ref(),
            reference.as_ref(),
            shared.cancel.load(Ordering::Relaxed),
            ros.controller_ready(),
        )?;
        if (arrived || last.elapsed() >= Duration::from_millis(200))
            && let (Some(arm), Some(reference)) = (arm, reference)
        {
            let phase = if arrived { "At start" } else { "Moving" };
            shared.log.push(format!(
                "{phase} measured={:?} controller_reference={:?} target_error={:.4} rad",
                arm.value.position,
                reference.value,
                max_error(arm.value.position, target)
            ));
            last = Instant::now();
        }
        if arrived {
            return Ok(());
        }
        thread::sleep(POLL);
    }
}

/// Replaces the command even when its last measured position has just gone stale. Confirmation
/// requires newer measured position, low velocity, and a fresh controller setpoint near the hold.
pub fn hold_verified(ros: &Ros, with_gripper: bool) -> Result<()> {
    let target = ros
        .arm()
        .ok_or_else(|| eyre!("no measured position for a hold"))?
        .value
        .position;
    let sent = Instant::now();
    ros.hold(&target)?;
    let finger = if with_gripper {
        let sample = ros
            .finger()
            .ok_or_else(|| eyre!("gripper hold unconfirmed: no measured position"))?;
        ros.command_finger(sample.value)?;
        Some(sample.value)
    } else {
        None
    };
    while sent.elapsed() < SETTLE {
        let finger_stopped = finger.is_none_or(|target| {
            ros.finger()
                .filter(Received::is_fresh)
                .is_some_and(|s| s.at > sent && (s.value - target).abs() <= 0.01)
        });
        if finger_stopped
            && let Some(arm) = ros.arm().filter(Received::is_fresh)
            && arm.at > sent
            && max_error(arm.value.position, target) <= TOLERANCE
            && arm.value.velocity.iter().all(|v| v.abs() <= 0.02)
            && ros
                .reference()
                .is_some_and(|r| r.at > sent && max_error(r.value, target) <= TOLERANCE)
        {
            ros.log(format!(
                "Hold verified target={target:?} measured={:?}",
                arm.value.position
            ));
            return Ok(());
        }
        thread::sleep(POLL);
    }
    bail!("hold published for {target:?}; no matching stationary feedback")
}

#[cfg(test)]
mod tests {
    use super::*;
    use test_case::test_case;

    #[test_case(true,true,0,Some(0.0),Some(0.0),false => Ok(true); "measured arrival")]
    #[test_case(false,true,0,Some(0.0),Some(0.0),false => Ok(false); "before scheduled arrival")]
    #[test_case(true,false,0,Some(0.0),Some(0.0),false => matches Err(_); "offline controller")]
    #[test_case(true,true,0,None,Some(0.0),false => matches Err(_); "missing arm")]
    #[test_case(true,true,0,Some(0.0),None,false => matches Err(_); "missing reference")]
    #[test_case(true,true,200,Some(0.0),Some(0.0),false => matches Err(_); "stale arm")]
    #[test_case(true,true,0,Some(0.1),Some(0.0),false => matches Err(_); "partial motion timeout")]
    #[test_case(true,true,0,Some(0.0),Some(0.1),false => matches Err(_); "unaccepted target timeout")]
    #[test_case(true,true,0,Some(0.0),Some(0.0),true => matches Err(_); "operator cancellation")]
    fn arrival(
        due: bool,
        active: bool,
        age: u64,
        measured: Option<f64>,
        commanded: Option<f64>,
        cancel: bool,
    ) -> Result<bool, String> {
        let now = Instant::now();
        let sent = now - Duration::from_secs(2);
        let movement = MovePlan {
            duration: Duration::from_secs(if due { 1 } else { 3 }),
            target: [0.0; 6],
        };
        let arm = measured.map(|q| Received {
            at: now - Duration::from_millis(age),
            value: ArmState {
                position: [q; 6],
                velocity: [0.0; 6],
            },
        });
        let reference = commanded.map(|q| Received {
            at: now,
            value: [q; 6],
        });
        movement
            .arrived(now, sent, arm.as_ref(), reference.as_ref(), cancel, active)
            .map_err(|e| e.to_string())
    }
}
