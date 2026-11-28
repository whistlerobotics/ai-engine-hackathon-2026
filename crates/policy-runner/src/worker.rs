use crate::{
    app::{Command, Phase, Shared},
    episode::Episode,
    policy::{Health, Policy, Readiness},
    ros::{Mode, Received, Ros},
    task::TaskPack,
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

/// Captured from the arm on 2026-11-25.
pub const START_POSE: Joints = [-3.1477, -1.6507, -2.4943, -0.6170, 1.4267, -4.5194];
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
                if move_to_start(ros, shared)? {
                    Phase::AtStart
                } else {
                    Phase::Idle
                }
            }
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
                    if let Some(issue) = ros.controller_issue().or_else(|| ros.gripper_issue()) {
                        return Err(eyre!("{issue}"));
                    }
                    ensure!(
                        ros.arm()
                            .filter(Received::is_fresh)
                            .is_some_and(|a| distance_to_start(&a.value.position) <= TOLERANCE),
                        "arm moved away from the selected start pose"
                    );
                    ensure!(!shared.cancel.load(Ordering::Relaxed), "operator cancelled");
                    ros.check_cameras()?;
                    Ok(task)
                })();
                match setup {
                    Ok(task) => {
                        let report =
                            Episode::run(ros, policy, &task, &shared.cancel, &shared.progress);
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

pub fn distance_to_start(position: &Joints) -> f64 {
    position
        .iter()
        .zip(START_POSE)
        .map(|(q, s)| (q - s).abs())
        .fold(0.0, f64::max)
}

fn start_move(from: &Joints) -> Duration {
    Duration::from_secs_f64(distance_to_start(from) / START_SPEED).max(MIN_START_MOVE)
}

fn move_to_start(ros: &Ros, shared: &Shared) -> Result<bool> {
    if let Some(issue) = ros.controller_issue() {
        return Err(eyre!("{issue}"));
    }
    let arm = ros
        .arm()
        .filter(Received::is_fresh)
        .ok_or_else(|| eyre!("missing fresh arm state"))?;
    let duration = start_move(&arm.value.position);
    ensure!(!shared.cancel.load(Ordering::Relaxed), "operator cancelled");
    let sent = Instant::now();
    ros.send_trajectory(&[START_POSE], duration)?;
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
            && distance_to_start(&measured.value.position) <= TOLERANCE
            && (ros.mode() == Mode::Real
                || ros
                    .reference()
                    .is_some_and(|r| r.at > sent && distance_to_start(&r.value) <= TOLERANCE))
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
        start_move(&from)
    }
}
