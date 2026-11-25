use crate::{
    policy::{ActError, Policy},
    ros::{Received, Ros},
};
use color_eyre::{
    Result,
    eyre::{self, bail, eyre},
};
use parking_lot::Mutex;
use policy_contract::{
    Joints, SCHEMA_VERSION, STEP, gripper, kinematics,
    wire::{ActRequest, Action, Images, Proprio},
};
use std::{
    fmt,
    sync::atomic::{AtomicBool, Ordering},
    thread,
    time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};

pub const STEPS: u32 = 300;
pub const BUDGET: Duration = Duration::from_secs(30);
pub const MAX_TRACKING_ERROR: f64 = 0.25;
/// How often a sleep checks for Ctrl-C and the budget.
const SLEEP_SLICE: Duration = Duration::from_millis(5);
/// Connection errors return at once; this keeps a policy that is down from being called in a
/// tight loop.
const MIN_RETRY_INTERVAL: Duration = Duration::from_millis(20);

pub enum Outcome {
    Completed,
    Failed(eyre::Report),
    Aborted,
}

pub struct Report {
    pub task_id: String,
    pub outcome: Outcome,
    /// Why the arm may not be holding, however the episode ended.
    pub hold_failed: Option<eyre::Report>,
    pub counts: Counts,
    pub duration: Duration,
}

#[derive(Debug, Default, Clone, Copy)]
pub struct Counts {
    pub steps: u32,
    pub late: u32,
    pub retries: u32,
}

impl fmt::Display for Report {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}: ", self.task_id)?;
        match &self.outcome {
            Outcome::Completed => write!(f, "Completed")?,
            Outcome::Failed(error) => write!(f, "Failed: {error:#}")?,
            Outcome::Aborted => write!(f, "Aborted")?,
        }
        if let Some(error) = &self.hold_failed {
            write!(f, "; holding the arm failed: {error:#}")?;
        }
        write!(
            f,
            "; {} steps in {:.2} s, {} late replies, {} retries",
            self.counts.steps,
            self.duration.as_secs_f64(),
            self.counts.late,
            self.counts.retries
        )
    }
}

/// An episode as it runs, for the operator's display.
#[derive(Debug, Clone)]
pub struct Progress {
    pub started: Instant,
    /// Set when the episode has ended.
    pub duration: Option<Duration>,
    pub counts: Counts,
    /// Of every `/act` call that got a valid reply, in order.
    pub latencies: Vec<Duration>,
    /// The largest joint tracking error at the latest observation.
    pub tracking_error: f64,
}

impl Progress {
    fn new() -> Self {
        Self {
            started: Instant::now(),
            duration: None,
            counts: Counts::default(),
            latencies: Vec::new(),
            tracking_error: 0.0,
        }
    }

    pub fn elapsed(&self) -> Duration {
        self.duration.unwrap_or_else(|| self.started.elapsed())
    }
}

/// Why the step loop stopped early.
enum Stop {
    Failed(eyre::Report),
    Aborted,
}

impl From<eyre::Report> for Stop {
    fn from(error: eyre::Report) -> Self {
        Self::Failed(error)
    }
}

pub struct Episode<'a> {
    ros: &'a Ros,
    policy: &'a Policy,
    task_id: &'a str,
    cancel: &'a AtomicBool,
    progress: &'a Mutex<Option<Progress>>,
    id: String,
    started: Instant,
    q_cmd: Joints,
    counts: Counts,
}

impl<'a> Episode<'a> {
    /// Runs one episode and makes the arm hold, however it ends. `progress` follows it from
    /// start to end.
    pub fn run(
        ros: &'a Ros,
        policy: &'a Policy,
        task_id: &'a str,
        cancel: &'a AtomicBool,
        progress: &'a Mutex<Option<Progress>>,
    ) -> Report {
        *progress.lock() = Some(Progress::new());
        let mut episode = Self {
            ros,
            policy,
            task_id,
            cancel,
            progress,
            id: episode_id(),
            started: Instant::now(),
            q_cmd: [0.0; 6],
            counts: Counts::default(),
        };
        let outcome = match episode.step_loop() {
            Ok(()) => Outcome::Completed,
            Err(Stop::Failed(error)) => Outcome::Failed(error),
            Err(Stop::Aborted) => Outcome::Aborted,
        };
        let report = Report {
            task_id: task_id.to_owned(),
            outcome,
            hold_failed: episode.stop().err(),
            counts: episode.counts,
            duration: episode.started.elapsed(),
        };
        episode.publish(|progress| progress.duration = Some(report.duration));
        report
    }

    fn publish(&self, update: impl FnOnce(&mut Progress)) {
        if let Some(progress) = self.progress.lock().as_mut() {
            update(progress);
        }
    }

    fn step_loop(&mut self) -> Result<(), Stop> {
        self.q_cmd = self.measured_arm()?;
        // The first action, like every other, starts one step after its observation.
        let mut next_start = Instant::now() + STEP;

        while self.counts.steps < STEPS {
            let actions = self.act()?;
            let now = Instant::now();
            let start = if now > next_start {
                self.counts.late += 1;
                now
            } else {
                next_start
            };

            let remaining = usize::try_from(STEPS - self.counts.steps).unwrap_or(usize::MAX);
            let actions = &actions[..actions.len().min(remaining)];
            let mut targets = Vec::with_capacity(actions.len());
            let mut q = self.q_cmd;
            for action in actions {
                for (joint, delta) in q.iter_mut().zip(action.delta) {
                    *joint += delta;
                }
                targets.push(q);
            }

            for (i, action) in (0_u32..).zip(actions) {
                self.sleep_until(start + STEP * i)?;
                if i == 0 {
                    self.ros.send_steps(&self.q_cmd, &targets, STEP)?;
                    self.q_cmd = q;
                }
                self.ros.command_finger(gripper::finger(action.gripper))?;
                self.counts.steps += 1;
                self.publish(|progress| progress.counts = self.counts);
            }
            next_start = start + STEP * u32::try_from(actions.len()).unwrap_or(u32::MAX);
        }
        // Lets the last action run its 50 ms before holding.
        self.sleep_until(next_start)
    }

    /// Observes and calls the policy for the current step, again on timeouts and connection
    /// errors, until a reply arrives or the budget runs out.
    fn act(&mut self) -> Result<Vec<Action>, Stop> {
        loop {
            self.check_cancel_and_budget()?;
            let attempt = Instant::now();
            let proprio = self.observe()?;
            let request = ActRequest {
                episode_id: &self.id,
                step: self.counts.steps,
                task_id: self.task_id,
                schema_version: SCHEMA_VERSION,
                images: Images {
                    top: self.ros.image()?,
                },
                proprio,
            };
            let sent = Instant::now();
            match self.policy.act(&request) {
                Ok(actions) => {
                    self.publish(|progress| progress.latencies.push(sent.elapsed()));
                    return Ok(actions);
                }
                Err(ActError::Transient(_)) => {
                    self.counts.retries += 1;
                    self.publish(|progress| progress.counts = self.counts);
                    self.sleep_until(attempt + MIN_RETRY_INTERVAL)?;
                }
                Err(error @ ActError::Invalid(_)) => {
                    return Err(eyre::Report::new(error)
                        .wrap_err(format!("step {}", self.counts.steps))
                        .into());
                }
            }
        }
    }

    fn observe(&self) -> Result<Proprio> {
        let Some(arm) = self.ros.arm().filter(Received::is_fresh) else {
            bail!("the arm's joint state is stale");
        };
        let Some(finger) = self.ros.finger().filter(Received::is_fresh) else {
            bail!("the gripper's joint state is stale");
        };

        let position = arm.value.position;
        let (joint, error) = self
            .q_cmd
            .iter()
            .zip(position)
            .map(|(command, measured)| (command - measured).abs())
            .enumerate()
            .fold((0, 0.0), |worst, (joint, error)| {
                if error > worst.1 {
                    (joint, error)
                } else {
                    worst
                }
            });
        self.publish(|progress| progress.tracking_error = error);
        if error > MAX_TRACKING_ERROR {
            bail!("joint {joint} is {error:.3} rad from its command");
        }

        let palm = kinematics::palm_pose(&position);
        Ok(Proprio {
            joint_pos: position,
            joint_vel: arm.value.velocity,
            gripper: gripper::opening(finger.value),
            ee_pos: palm.position,
            ee_quat: palm.orientation,
        })
    }

    fn sleep_until(&self, deadline: Instant) -> Result<(), Stop> {
        loop {
            self.check_cancel_and_budget()?;
            let now = Instant::now();
            if now >= deadline {
                return Ok(());
            }
            thread::sleep((deadline - now).min(SLEEP_SLICE));
        }
    }

    fn check_cancel_and_budget(&self) -> Result<(), Stop> {
        if self.cancel.load(Ordering::Relaxed) {
            return Err(Stop::Aborted);
        }
        if self.started.elapsed() > BUDGET {
            return Err(eyre!("not completed within {} s", BUDGET.as_secs()).into());
        }
        Ok(())
    }

    fn measured_arm(&self) -> Result<Joints> {
        match self.ros.arm() {
            Some(arm) if arm.is_fresh() => Ok(arm.value.position),
            _ => bail!("the arm's joint state is stale"),
        }
    }

    /// Holds the arm where it is; where it was last commanded, without a joint state.
    fn stop(&self) -> Result<()> {
        let position = self.ros.arm().map_or(self.q_cmd, |arm| arm.value.position);
        self.ros.hold(&position)
    }
}

/// Milliseconds since the Unix epoch, which is new for each episode, as the contract asks.
fn episode_id() -> String {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis()
        .to_string()
}
