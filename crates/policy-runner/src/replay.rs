use crate::{
    app::Shared,
    episode::{Episode, Outcome},
    policy::Policy,
    ros::{Mode, Received, Ros},
    task::TaskPack,
    worker::{START_POSE, move_to_start, prepare_episode},
};
use color_eyre::{
    Result,
    eyre::{bail, ensure, eyre},
};
use policy_contract::{ARM_JOINTS, Joints};
use std::{
    f64::consts::{PI, TAU},
    io::{self, BufRead, Write},
    sync::{
        Arc,
        atomic::Ordering,
        mpsc::{self, RecvTimeoutError},
    },
    thread,
    time::{Duration, Instant},
};

const DISCOVERY_TIMEOUT: Duration = Duration::from_secs(15);
const POLL: Duration = Duration::from_millis(50);

pub fn start_pose(values: Option<Vec<f64>>) -> Result<Joints> {
    let Some(values) = values else {
        return Ok(START_POSE);
    };
    let joints: Joints = values
        .try_into()
        .map_err(|_| eyre!("expected six mock replay start angles"))?;
    let limits = [TAU, TAU, PI, TAU, TAU, TAU];
    for ((name, angle), limit) in ARM_JOINTS.into_iter().zip(joints).zip(limits) {
        ensure!(
            angle.is_finite() && angle.abs() <= limit,
            "{name}: expected a mock replay start angle within ±{limit:.3} rad"
        );
    }
    Ok(joints)
}

pub fn status(line: &str) -> Result<()> {
    let mut output = io::stdout().lock();
    writeln!(output, "{line}")?;
    output.flush()?;
    Ok(())
}

#[derive(PartialEq, Eq)]
enum Phase {
    AwaitingHome,
    AtStart,
    Replayed,
}

pub fn serve(
    ros: &Ros,
    policy: &Policy,
    pack: &TaskPack,
    task_id: &str,
    shared: Shared,
    target: &Joints,
    steps: u32,
) -> Result<()> {
    ensure!(
        ros.mode() == Mode::Mock && std::env::var("ROS_DOMAIN_ID").as_deref() == Ok("77"),
        "mock replay requires --mode mock and ROS_DOMAIN_ID=77"
    );
    let shared = Arc::new(shared);
    let cancel = Arc::clone(&shared);
    ctrlc::set_handler(move || cancel.cancel.store(true, Ordering::Relaxed))?;
    let (send, commands) = mpsc::channel();
    let cancel = Arc::clone(&shared);
    thread::spawn(move || {
        for line in io::stdin().lock().lines().map_while(|line| line.ok()) {
            let command = line.trim();
            let stop = matches!(command, "cancel" | "quit");
            if stop {
                cancel.cancel.store(true, Ordering::Relaxed);
            }
            if send.send(command.to_owned()).is_err() || stop {
                return;
            }
        }
        cancel.cancel.store(true, Ordering::Relaxed);
    });

    wait_for_arm(ros, &shared)?;
    status("READY")?;
    let mut phase = Phase::AwaitingHome;
    loop {
        let command = match commands.recv_timeout(POLL) {
            Ok(command) => command,
            Err(RecvTimeoutError::Timeout) => {
                ensure!(!shared.cancel.load(Ordering::Relaxed), "replay cancelled");
                continue;
            }
            Err(RecvTimeoutError::Disconnected) => bail!("replay input closed"),
        };
        match command.as_str() {
            "quit" => return Ok(()),
            "cancel" => bail!("replay cancelled"),
            "home" if phase == Phase::AwaitingHome => {
                status("HOMING")?;
                if !move_to_start(ros, &shared, target)? {
                    let last = shared.log.latest(1);
                    let message = last
                        .first()
                        .map_or("mock homing stopped", |line| line.text.as_str());
                    bail!("{message}");
                }
                status("HOMED")?;
                phase = Phase::AtStart;
            }
            "run" if phase == Phase::AtStart => {
                let task = prepare_episode(ros, policy, &shared, pack, task_id, target)?;
                status("REPLAYING")?;
                let report =
                    Episode::run(ros, policy, &task, &shared.cancel, &shared.progress, steps);
                ensure!(
                    matches!(report.outcome, Outcome::Completed) && report.hold_failed.is_none(),
                    "{report}"
                );
                status("REPLAYED")?;
                phase = Phase::Replayed;
            }
            _ => bail!("unexpected replay command: {command:?}"),
        }
    }
}

fn wait_for_arm(ros: &Ros, shared: &Shared) -> Result<()> {
    let started = Instant::now();
    loop {
        ensure!(!shared.cancel.load(Ordering::Relaxed), "replay cancelled");
        let arm_live = ros
            .arm()
            .filter(Received::is_fresh)
            .is_some_and(|sample| sample.value.position.iter().all(|joint| joint.is_finite()));
        if arm_live && ros.controller_ready() {
            return Ok(());
        }
        ensure!(
            started.elapsed() < DISCOVERY_TIMEOUT,
            "{}",
            ros.controller_issue()
                .unwrap_or("fresh, finite mock joint feedback did not arrive")
        );
        thread::sleep(POLL);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use test_case::test_case;

    #[test_case(None => matches Ok(START_POSE))]
    #[test_case(Some(vec![0.0; 6]) => matches Ok(_))]
    #[test_case(Some(vec![0.0; 5]) => matches Err(_))]
    #[test_case(Some(vec![f64::NAN; 6]) => matches Err(_))]
    #[test_case(Some(vec![0.0, 0.0, 3.5, 0.0, 0.0, 0.0]) => matches Err(_))]
    fn mock_start_pose(values: Option<Vec<f64>>) -> Result<Joints> {
        start_pose(values)
    }
}
