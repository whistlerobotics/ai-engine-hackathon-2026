use crate::{
    app::{Command, Phase, Shared},
    episode::Episode,
    policy::Policy,
    ros::Ros,
};
use color_eyre::{Result, eyre::bail};
use policy_contract::Joints;
use std::{
    sync::{atomic::Ordering, mpsc::Receiver},
    thread,
    time::{Duration, Instant},
};

/// Captured from the real arm on 2026-11-25; not the contract's example pose.
pub const START_POSE: Joints = [-3.1477, -1.6507, -2.4943, -0.6170, 1.4267, -4.5194];
/// Half a turn takes 10 s, as every move to the start pose used to.
const START_SPEED: f64 = std::f64::consts::PI / 10.0;
const MIN_START_MOVE: Duration = Duration::from_secs(1);

/// Carries out the operator's commands until the sender is dropped. Returns an error, ending
/// the runner, when the arm may not be holding.
pub fn serve(
    ros: &Ros,
    policy: &Policy,
    shared: &Shared,
    commands: &Receiver<Command>,
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
                let report = Episode::run(ros, policy, &task_id, &shared.cancel, &shared.progress);
                let line = report.to_string();
                shared.log.push(line.clone());
                shared.reports.lock().push(line);
                if report.hold_failed.is_some() {
                    // Commands to the arm are likely failing; someone should look before it
                    // moves again.
                    bail!("stopping, as the arm may not be holding");
                }
                Phase::Idle
            }
        };
        *shared.phase.lock() = phase;
    }
    Ok(())
}

/// The largest joint error from `position` to the start pose, in radians.
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

/// Moves the arm slowly to the start pose. Returns false if the operator aborted, after making
/// the arm hold.
fn move_to_start(ros: &Ros, shared: &Shared) -> Result<bool> {
    let Some(arm) = ros.arm() else {
        bail!("no joint state from the arm")
    };
    let duration = start_move(&arm.value.position);
    shared.log.push(format!(
        "Moving to the start pose over {:.1} s.",
        duration.as_secs_f64()
    ));
    ros.send_trajectory(&[START_POSE], duration)?;

    let arrival = Instant::now() + duration + Duration::from_millis(500);
    while Instant::now() < arrival {
        if shared.cancel.load(Ordering::Relaxed) {
            if let Some(arm) = ros.arm() {
                ros.hold(&arm.value.position)?;
            }
            shared.log.push("Move to the start pose aborted; holding.");
            return Ok(false);
        }
        thread::sleep(Duration::from_millis(10));
    }

    let Some(arm) = ros.arm() else {
        bail!("no joint state from the arm")
    };
    let error = distance_to_start(&arm.value.position);
    shared
        .log
        .push(format!("At the start pose, within {error:.4} rad."));
    Ok(true)
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
    fn start_move_scales_with_distance(from: Joints) -> Duration {
        start_move(&from)
    }
}
