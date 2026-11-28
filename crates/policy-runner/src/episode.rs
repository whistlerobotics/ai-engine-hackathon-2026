use crate::{
    policy::Policy,
    ros::{Received, Ros},
    task::Task,
};
use color_eyre::{
    Result,
    eyre::{self, bail, eyre},
};
use parking_lot::Mutex;
use policy_contract::{
    Joints, SCHEMA_VERSION, STEP, gripper, kinematics,
    wire::{ActRequest, Action, Proprio},
};
use std::{
    fmt,
    sync::atomic::{AtomicBool, Ordering},
    thread,
    time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};

pub const STEPS: u32 = 1200;
pub const BUDGET: Duration = Duration::from_secs(90);
pub const MAX_TRACKING_ERROR: f64 = 0.25;
/// How often a wait checks for cancellation and lost feedback.
const SLEEP_SLICE: Duration = Duration::from_millis(5);
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
    pub http: Duration,
}

impl fmt::Display for Report {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}: ", self.task_id)?;
        match &self.outcome {
            Outcome::Completed => write!(f, "Step limit reached")?,
            Outcome::Failed(error) => write!(f, "Failed: {error:#}")?,
            Outcome::Aborted => write!(f, "Aborted")?,
        }
        if let Some(error) = &self.hold_failed {
            write!(f, "; holding the arm failed: {error:#}")?;
        }
        write!(
            f,
            "; {} steps, {:.2} s wall, {:.2} s action, {:.2} s HTTP",
            self.counts.steps,
            self.duration.as_secs_f64(),
            (STEP * self.counts.steps).as_secs_f64(),
            self.counts.http.as_secs_f64(),
        )
    }
}

/// An episode as it runs, for the operator's display.
#[derive(Debug, Clone)]
pub struct Progress {
    pub counts: Counts,
    /// Of every `/act` call that got a valid reply, in order.
    pub latencies: Vec<Duration>,
    /// The largest joint tracking error at the latest observation.
    pub tracking_error: f64,
}

impl Progress {
    fn new() -> Self {
        Self {
            counts: Counts::default(),
            latencies: Vec::new(),
            tracking_error: 0.0,
        }
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
    task: &'a Task,
    cancel: &'a AtomicBool,
    progress: &'a Mutex<Option<Progress>>,
    id: String,
    started: Instant,
    q_cmd: Joints,
    commanded: bool,
    counts: Counts,
    step_limit: u32,
}

impl<'a> Episode<'a> {
    /// Runs one episode and makes the arm hold, however it ends. `progress` follows it from
    /// start to end.
    pub fn run(
        ros: &'a Ros,
        policy: &'a Policy,
        task: &'a Task,
        cancel: &'a AtomicBool,
        progress: &'a Mutex<Option<Progress>>,
        step_limit: u32,
    ) -> Report {
        *progress.lock() = Some(Progress::new());
        let mut episode = Self {
            ros,
            policy,
            task,
            cancel,
            progress,
            id: episode_id(),
            started: Instant::now(),
            q_cmd: [0.0; 6],
            commanded: false,
            counts: Counts::default(),
            step_limit,
        };
        let outcome = match episode.step_loop() {
            Ok(()) => Outcome::Completed,
            Err(Stop::Failed(error)) => Outcome::Failed(error),
            Err(Stop::Aborted) => Outcome::Aborted,
        };
        Report {
            task_id: task.id.clone(),
            outcome,
            hold_failed: episode.stop().err(),
            counts: episode.counts,
            duration: episode.started.elapsed(),
        }
    }

    fn publish(&self, update: impl FnOnce(&mut Progress)) {
        if let Some(progress) = self.progress.lock().as_mut() {
            update(progress);
        }
    }

    fn step_loop(&mut self) -> Result<(), Stop> {
        self.q_cmd = self.measured_arm()?;
        let mut next_start = Instant::now() + STEP;
        while self.counts.steps < self.step_limit {
            let actions = self.act()?;
            let now = Instant::now();
            if now > next_start {
                self.counts.late += 1;
            }
            let start = now.max(next_start);
            let remaining = (self.step_limit - self.counts.steps) as usize;
            let actions = &actions[..actions.len().min(remaining)];
            let mut target = self.q_cmd;
            let mut targets = Vec::with_capacity(actions.len());
            for action in actions {
                for (joint, delta) in target.iter_mut().zip(action.delta) {
                    *joint += delta;
                }
                targets.push(target);
            }

            for (index, action) in (0_u32..).zip(actions) {
                self.sleep_until(start + STEP * index)?;
                if index == 0 {
                    self.ros.send_steps(&self.q_cmd, &targets, STEP)?;
                    self.commanded = true;
                    self.q_cmd = target;
                }
                self.ros.command_finger(gripper::finger(action.gripper))?;
                self.counts.steps += 1;
                self.publish(|p| p.counts = self.counts);
            }
            next_start = start + STEP * u32::try_from(actions.len()).unwrap_or(u32::MAX);
        }
        self.sleep_until(next_start)
    }

    fn act(&mut self) -> Result<Vec<Action>, Stop> {
        let remaining = BUDGET.saturating_sub(self.counts.http);
        if remaining.is_zero() {
            return Err(eyre!("cumulative policy HTTP budget exhausted").into());
        }
        let request = ActRequest {
            episode_id: &self.id,
            step: self.counts.steps,
            task_id: &self.task.id,
            instruction: &self.task.instruction,
            schema_version: SCHEMA_VERSION,
            images: self.ros.images()?,
            goal: &self.task.goal,
            proprio: self.observe()?,
        };
        self.check()?;
        let sent = Instant::now();
        let result = self.policy.runtime.block_on(async {
            let call = self.policy.act(&request, remaining);
            tokio::pin!(call);
            loop {
                tokio::select! {
                    biased;
                    () = tokio::time::sleep(SLEEP_SLICE) => self.check()?,
                    result=&mut call => break Ok::<_,Stop>(result),
                }
            }
        });
        let duration = sent.elapsed();
        self.counts.http += duration;
        self.publish(|p| {
            p.latencies.push(duration);
            p.counts = self.counts;
        });
        let result = result?;
        self.check()?;
        if self.counts.http >= BUDGET {
            return Err(eyre!("cumulative policy HTTP budget exhausted").into());
        }
        result.map_err(|error| eyre::Report::new(error).into())
    }

    fn stop(&self) -> Result<()> {
        if self.commanded {
            let position = self.ros.arm().map_or(self.q_cmd, |arm| arm.value.position);
            self.ros.hold(&position)?;
        }
        Ok(())
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
            self.check()?;
            let now = Instant::now();
            if now >= deadline {
                return Ok(());
            }
            thread::sleep((deadline - now).min(SLEEP_SLICE));
        }
    }

    fn check(&self) -> Result<(), Stop> {
        if self.cancel.load(Ordering::Relaxed) {
            return Err(Stop::Aborted);
        }
        if let Some(issue) = self
            .ros
            .controller_issue()
            .or_else(|| self.ros.gripper_issue())
        {
            return Err(eyre!("{issue}").into());
        }
        self.ros.check_cameras()?;
        self.measured_arm()?;
        if !self.ros.finger().is_some_and(|sample| sample.is_fresh()) {
            return Err(eyre!("the gripper's joint state is stale").into());
        }
        Ok(())
    }

    fn measured_arm(&self) -> Result<Joints> {
        match self.ros.arm() {
            Some(arm) if arm.is_fresh() => Ok(arm.value.position),
            _ => bail!("the arm's joint state is stale"),
        }
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
