mod episode;
mod file;
mod ros;

use crate::{
    episode::{End, Episode, observe},
    ros::{ACTIVE_TOPIC, HandGuiding, Ros},
};
use clap::Parser;
use color_eyre::{Result, eyre::eyre};
use policy_contract::STEP;
use signal_hook::consts::{SIGHUP, SIGINT, SIGTERM};
use std::{
    path::PathBuf,
    sync::{
        Arc,
        atomic::{AtomicUsize, Ordering},
    },
    thread,
    time::{Duration, Instant},
};

/// How long after the first press without a gripper command to say what is missing.
const COMMAND_WAIT: Duration = Duration::from_secs(1);
/// How long hand-guiding may stay silent at startup before saying it is not running.
const STARTUP_GRACE: Duration = Duration::from_secs(1);

/// Records demonstrations, one JSON file per episode. An episode runs for as long as
/// hand-guiding has the arm in freedrive, which is while its button is held.
#[derive(Parser)]
struct Args {
    /// The episodes' `task_id`, and the folder they are saved in.
    #[arg(long, value_parser = task_id)]
    task: String,
    /// Where episodes are saved, each in a folder named after its task.
    #[arg(long, default_value = "recording")]
    out_dir: PathBuf,
    /// The longest an episode runs, in seconds, at most 60. It is then saved; the next one starts
    /// with the next press.
    #[arg(long, default_value_t = 60, value_parser = clap::value_parser!(u64).range(1..=60))]
    max_duration: u64,
    /// Record on without a fresh camera frame or gripper reading, with a black frame or a null
    /// gripper in its place, instead of ending the episode. A stale arm still ends it.
    #[arg(long)]
    permissive: bool,
}

enum State {
    /// Until hand-guiding reports freedrive off, so that a press under way at startup, or one that
    /// outlasted its episode, is not taken for a new one.
    WaitingForRelease,
    Idle,
    /// Freedrive is on; step 0 waits for the gripper's first command since `since`, so that it
    /// records the one the throttle gives now and not one left from earlier.
    Starting {
        since: Instant,
        reported: bool,
    },
    Recording(Recording),
}

struct Recording {
    episode: Episode,
    next_step_at: Instant,
}

/// What lasts the whole session.
struct Recorder {
    ros: Ros,
    args: Args,
    max_steps: usize,
    saved: usize,
}

fn main() -> Result<()> {
    color_eyre::install()?;
    let args = Args::parse();
    let max_steps =
        usize::try_from(Duration::from_secs(args.max_duration).as_millis() / STEP.as_millis())?;

    let signal = Arc::new(AtomicUsize::new(0));
    for number in [SIGINT, SIGTERM, SIGHUP] {
        signal_hook::flag::register_usize(number, Arc::clone(&signal), usize::try_from(number)?)?;
    }

    println!(
        "Hold the hand-guiding button to record an episode of {}; release it to save. Ctrl-C to quit.",
        args.task
    );
    let mut recorder = Recorder {
        ros: Ros::new()?,
        args,
        max_steps,
        saved: 0,
    };
    let started = Instant::now();
    let mut reported_silent = false;
    let mut state = State::WaitingForRelease;

    let stopped_by = loop {
        let now = Instant::now();
        if let Some(name) = signal_name(signal.load(Ordering::Relaxed)) {
            if let State::Recording(recording) = state {
                recorder.finish(&recording.episode, &End::Signal(name));
            }
            break name;
        }

        let hand_guiding = recorder.ros.hand_guiding(now);
        let silent = hand_guiding.is_none();
        if now - started > STARTUP_GRACE && silent != reported_silent {
            reported_silent = silent;
            if silent {
                println!("Nothing on {ACTIVE_TOPIC}: is hand-guiding running?");
            } else {
                println!("hand-guiding is running.");
            }
        }

        state = match state {
            State::WaitingForRelease if hand_guiding == Some(HandGuiding::Idle) => State::Idle,
            State::Idle if hand_guiding == Some(HandGuiding::Guiding) => State::Starting {
                since: now,
                reported: false,
            },
            State::Starting { .. } if hand_guiding != Some(HandGuiding::Guiding) => {
                println!("Released before the gripper was commanded: no episode.");
                State::WaitingForRelease
            }
            State::Starting { since, reported } => recorder.start(since, reported, now),
            State::Recording(mut recording) => {
                match recorder.record(&mut recording, hand_guiding, now) {
                    None => State::Recording(recording),
                    Some(end) => {
                        recorder.finish(&recording.episode, &end);
                        if end == End::Release {
                            State::Idle
                        } else {
                            State::WaitingForRelease
                        }
                    }
                }
            }
            unchanged @ (State::WaitingForRelease | State::Idle) => unchanged,
        };

        let pause = if matches!(state, State::Recording(_)) {
            Duration::from_millis(2)
        } else {
            Duration::from_millis(10)
        };
        thread::sleep(pause);
    };

    println!(
        "\nStopped: {stopped_by}. {} episode(s) saved this session.",
        recorder.saved
    );
    Ok(())
}

impl Recorder {
    fn start(&self, since: Instant, reported: bool, now: Instant) -> State {
        let Some(command) = self
            .ros
            .finger_command()
            .filter(|command| command.at >= since)
        else {
            if !reported && now - since > COMMAND_WAIT {
                println!(
                    "Waiting for hand-guiding to command the gripper: is the throttle all right?"
                );
            }
            return State::Starting {
                since,
                reported: reported || now - since > COMMAND_WAIT,
            };
        };
        match observe(&self.ros, now, self.args.permissive) {
            Ok(observation) => {
                let mut episode = Episode::new(&self.args.task, now);
                episode.add_step(now, observation, command.value);
                println!("Recording {}. Release to stop.", episode.id);
                State::Recording(Recording {
                    episode,
                    next_step_at: now + STEP,
                })
            }
            Err(stale) => {
                println!("Not starting: {stale}. Release and press again.");
                State::WaitingForRelease
            }
        }
    }

    /// Records the next step when it is due; returns why the episode ended, if it has.
    fn record(
        &self,
        recording: &mut Recording,
        hand_guiding: Option<HandGuiding>,
        now: Instant,
    ) -> Option<End> {
        match hand_guiding {
            Some(HandGuiding::Guiding) => {}
            Some(HandGuiding::Idle) => return Some(End::Release),
            None => return Some(End::HandGuidingStopped),
        }
        if now < recording.next_step_at {
            return None;
        }
        let observation = match observe(&self.ros, now, self.args.permissive) {
            Ok(observation) => observation,
            Err(stale) => return Some(End::Stale(stale)),
        };
        // Step 0 waited for a command, so there is one, if perhaps from before the throttle started
        // reading out of range: the gripper holds it meanwhile.
        let Some(command) = self.ros.finger_command() else {
            return Some(End::HandGuidingStopped);
        };
        recording.episode.add_step(now, observation, command.value);

        recording.next_step_at += STEP;
        if now - recording.next_step_at > STEP {
            recording.next_step_at = now + STEP;
        }
        let steps = recording.episode.steps.len();
        if steps.is_multiple_of(20) {
            println!("  step {steps} ({:.0} s)", duration(steps).as_secs_f64());
        }
        (steps >= self.max_steps).then_some(End::TimeLimit)
    }

    /// Saves `episode` and says so.
    fn finish(&mut self, episode: &Episode, end: &End) {
        let ended_by = end.to_string();
        match file::save(episode, &self.args.out_dir, &ended_by) {
            Ok(path) => {
                let steps = episode.steps.len();
                let mut details = vec![format!("{:.2} s", duration(steps).as_secs_f64())];
                if episode.steps_without_image > 0 {
                    details.push(format!(
                        "{} without a camera frame",
                        episode.steps_without_image
                    ));
                }
                if *end != End::Release {
                    details.push(format!("ended by {ended_by}"));
                }
                println!(
                    "Saved {steps} steps ({}) to {}",
                    details.join(", "),
                    path.display()
                );
                self.saved += 1;
            }
            Err(error) => {
                eprintln!("Saving {} failed: {error:#}", episode.id);
            }
        }
    }
}

fn duration(steps: usize) -> Duration {
    STEP * u32::try_from(steps).unwrap_or(u32::MAX)
}

fn signal_name(number: usize) -> Option<&'static str> {
    let number = i32::try_from(number).ok()?;
    match number {
        SIGINT => Some("Ctrl-C"),
        SIGTERM => Some("SIGTERM"),
        SIGHUP => Some("SIGHUP"),
        _ => None,
    }
}

/// A `task_id` that names a single folder.
fn task_id(value: &str) -> Result<String> {
    if value.is_empty() || value == "." || value == ".." || value.contains('/') {
        return Err(eyre!("{value:?} is not a folder name"));
    }
    Ok(value.to_owned())
}

#[cfg(test)]
mod tests {
    use super::*;
    use test_case::test_case;

    #[test_case("bench_lift_can_v0" => matches Ok(_))]
    #[test_case("" => matches Err(_); "empty")]
    #[test_case(".." => matches Err(_); "parent")]
    #[test_case("a/b" => matches Err(_))]
    fn task_ids(value: &str) -> Result<String> {
        task_id(value)
    }
}
