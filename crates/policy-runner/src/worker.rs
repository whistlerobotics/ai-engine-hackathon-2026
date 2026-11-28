use crate::{
    app::{Command, Phase, Shared},
    episode::{Episode, STEPS},
    policy::{Health, Policy, Readiness},
    ros::{Mode, Received, Ros},
    task::{Task, TaskPack},
};
use color_eyre::{
    Result,
    eyre::{ensure, eyre},
};
use policy_contract::Joints;
use std::{
    sync::{atomic::Ordering, mpsc::Receiver},
    thread,
    time::{Duration, Instant},
};

/// Captured 2026-11-28 at 07:14 UTC; J6 adjusted by +360 degrees.
pub const START_POSE: Joints = [
    -2.932028,
    -1.278959,
    -2.076159,
    -1.365526,
    1.570474,
    0.615173307,
];
pub const TOLERANCE: f64 = 0.02;
const START_SPEED: f64 = std::f64::consts::PI / 10.0;
const MIN_START_MOVE: Duration = Duration::from_secs(1);
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
            Command::MoveToStart => {
                if move_to_start(ros, shared, &START_POSE)? {
                    Phase::AtStart
                } else {
                    Phase::Idle
                }
            }
            Command::RunEpisode { task_id } => {
                let setup = prepare_episode(ros, policy, shared, pack, &task_id, &START_POSE);
                match setup {
                    Ok(task) => {
                        let report = Episode::run(
                            ros,
                            policy,
                            &task,
                            &shared.cancel,
                            &shared.progress,
                            STEPS,
                        );
                        let line = report.to_string();
                        shared.log.push(line.clone());
                        shared.reports.lock().push(line);
                        ensure!(
                            report.hold_failed.is_none(),
                            "stopping, as the arm may not be holding"
                        );
                    }
                    Err(reason) => shared.log.push(format!("Episode refused: {reason:#}")),
                }
                Phase::Idle
            }
        };
        *shared.phase.lock() = phase;
    }
    Ok(())
}

pub fn prepare_episode(
    ros: &Ros,
    policy: &Policy,
    shared: &Shared,
    pack: &TaskPack,
    task_id: &str,
    start_pose: &Joints,
) -> Result<Task> {
    let task = pack.select(task_id)?;
    let status = policy.status();
    ensure!(
        status.health == Health::Up && status.readiness == Readiness::Ready,
        "policy health {}, readiness {}",
        status.health,
        status.readiness
    );
    if let Some(issue) = ros.controller_issue().or_else(|| ros.gripper_issue()) {
        return Err(eyre!("{issue}"));
    }
    ensure!(
        ros.arm()
            .filter(Received::is_fresh)
            .is_some_and(|arm| distance_to_pose(&arm.value.position, start_pose) <= TOLERANCE),
        "arm moved away from the selected start pose"
    );
    ensure!(!shared.cancel.load(Ordering::Relaxed), "operator cancelled");
    ros.check_cameras()?;
    Ok(task)
}

pub fn distance_to_start(position: &Joints) -> f64 {
    distance_to_pose(position, &START_POSE)
}

fn distance_to_pose(position: &Joints, target: &Joints) -> f64 {
    position
        .iter()
        .zip(target)
        .map(|(q, s)| {
            if q.is_finite() {
                (q - s).abs()
            } else {
                f64::INFINITY
            }
        })
        .fold(0.0, f64::max)
}

fn start_move(from: &Joints, target: &Joints) -> Result<Duration> {
    let duration = Duration::try_from_secs_f64(distance_to_pose(from, target) / START_SPEED)
        .map_err(|_| eyre!("invalid measured joint positions"))?;
    Ok(duration.max(MIN_START_MOVE))
}

pub fn move_to_start(ros: &Ros, shared: &Shared, target: &Joints) -> Result<bool> {
    if let Some(issue) = ros.controller_issue() {
        return Err(eyre!("{issue}"));
    }
    let arm = ros
        .arm()
        .filter(Received::is_fresh)
        .ok_or_else(|| eyre!("missing fresh arm state"))?;
    let duration = start_move(&arm.value.position, target)?;
    ensure!(!shared.cancel.load(Ordering::Relaxed), "operator cancelled");
    let sent = Instant::now();
    ros.send_trajectory(std::slice::from_ref(target), duration)?;
    shared.log.push(format!(
        "Moving to the start pose over {:.1} s.",
        duration.as_secs_f64()
    ));
    loop {
        let measured = ros
            .arm()
            .ok_or_else(|| eyre!("no arm state available to hold"))?;
        let stop = if shared.cancel.load(Ordering::Relaxed) {
            "operator cancelled"
        } else if !ros.controller_ready() || !measured.is_fresh() {
            "arm controller or feedback lost"
        } else if sent.elapsed() < duration {
            thread::sleep(POLL);
            continue;
        } else if measured.at > sent
            && distance_to_pose(&measured.value.position, target) <= TOLERANCE
            && (ros.mode() == Mode::Real
                || ros.reference().is_some_and(|r| {
                    r.at > sent && distance_to_pose(&r.value, target) <= TOLERANCE
                }))
        {
            shared.log.push("At the start pose.");
            return Ok(true);
        } else if sent.elapsed() < duration + SETTLE {
            thread::sleep(POLL);
            continue;
        } else {
            "start pose not reached"
        };
        ros.hold(&measured.value.position)?;
        shared
            .log
            .push(format!("Move stopped: {stop}. Hold requested."));
        return Ok(false);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;
    use test_case::test_case;

    fn offset(joint: usize, by: f64) -> Joints {
        let mut position = START_POSE;
        position[joint] += by;
        position
    }

    #[test_case(START_POSE => MIN_START_MOVE; "already there")]
    #[test_case(offset(5, -2.0 * PI) => Duration::from_secs(20); "full turn backwards")]
    fn start_move_duration(from: Joints) -> Duration {
        start_move(&from, &START_POSE).unwrap()
    }
}
