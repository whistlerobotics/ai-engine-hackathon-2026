use crate::{
    motion,
    policy::{ActError, Policy},
    ros::{Received, Ros},
    task::{Task, joint_limit},
    worker::hold_verified,
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
/// How often a sleep checks for Ctrl-C and the budget.
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
    pub holds: u32,
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
            "; {} steps, {:.2} s wall, {:.2} s action, {:.2} s HTTP, {} held steps",
            self.counts.steps,
            self.duration.as_secs_f64(),
            (STEP * self.counts.steps).as_secs_f64(),
            self.counts.http.as_secs_f64(),
            self.counts.holds
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
    task: &'a Task,
    cancel: &'a AtomicBool,
    progress: &'a Mutex<Option<Progress>>,
    id: String,
    started: Instant,
    q_cmd: Joints,
    gripper_target: f64,
    counts: Counts,
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
            gripper_target: 0.0,
            counts: Counts::default(),
        };
        let outcome = match episode.step_loop() {
            Ok(()) => Outcome::Completed,
            Err(Stop::Failed(error)) => Outcome::Failed(error),
            Err(Stop::Aborted) => Outcome::Aborted,
        };
        let report = Report {
            task_id: task.id.clone(),
            outcome,
            hold_failed: hold_verified(ros, true).err(),
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
        if !self
            .q_cmd
            .iter()
            .enumerate()
            .all(|(j, q)| q.abs() <= joint_limit(j))
        {
            return Err(eyre!("initial measured target exceeds mock joint limits").into());
        }
        self.gripper_target = self.observe()?.gripper;
        while self.counts.steps < STEPS {
            self.check()?;
            let mut actions = self.act()?;
            motion::clip_chunk(self.q_cmd, &mut actions);
            let remaining = (STEPS - self.counts.steps) as usize;
            for action in actions.into_iter().take(remaining) {
                self.check()?;
                let motion = motion::integrate(self.q_cmd, action.delta);
                let target = motion.target;
                let issued = Instant::now();
                self.ros.send_trajectory(&[target], STEP)?;
                self.ros.command_finger(gripper::finger(action.gripper))?;
                self.q_cmd = target;
                self.gripper_target = action.gripper;
                self.ros.log(format!(
                    "episode={} step={} target={target:?} gripper={} requested_tcp_m_s={:.5} requested_elbow_m_s={:.5} speed_scale={:.6}",
                    self.id, self.counts.steps, action.gripper, motion.tcp_speed, motion.elbow_speed, motion.scale
                ));
                self.counts.steps += 1;
                self.publish(|p| p.counts = self.counts);
                self.sleep_until(issued + STEP)?;
                let measured = self.measured_arm()?;
                let reference = self
                    .ros
                    .reference()
                    .ok_or_else(|| eyre!("controller reference lost after command"))?;
                self.ros.log(format!("episode={} step={} measured={measured:?} controller_reference={:?} error={:.5}",self.id,self.counts.steps-1,reference.value,crate::worker::max_error(measured,self.q_cmd)));
            }
        }
        Ok(())
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
            images: self.ros.images(&self.id, self.counts.steps)?,
            goal: &self.task.goal,
            proprio: self.observe()?,
        };
        // Single-point trajectories end at zero velocity. Keep that absolute target while the
        // HTTP future is pending; dropping the future cancels transport on stop or feedback loss.
        self.check()?;
        self.ros.hold(&self.q_cmd)?;
        self.ros.log(format!(
            "episode={} step={} holding while awaiting policy",
            self.id, self.counts.steps
        ));
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
        if duration > STEP {
            self.counts.late += 1;
        }
        self.publish(|p| {
            p.latencies.push(duration);
            p.counts = self.counts;
        });
        let result = result?;
        self.check()?;
        if self.counts.http >= BUDGET {
            return Err(eyre!("cumulative policy HTTP budget exhausted").into());
        }
        match result {
            Ok(actions) => Ok(actions),
            Err(ActError::Invalid(_)) => {
                self.counts.holds += 1;
                self.ros.log(format!(
                    "episode={} step={} invalid reply; one hold step",
                    self.id, self.counts.steps
                ));
                Ok(vec![Action {
                    delta: [0.0; 6],
                    gripper: self.gripper_target,
                }])
            }
            Err(ActError::Transport(reason)) => Err(reason.into()),
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
        if !self.ros.controller_ready() || !self.ros.gripper_ready() {
            return Err(eyre!(
                "mock controller inactive, feedback stale or command subscribers changed"
            )
            .into());
        }
        self.ros.check_cameras()?;
        self.observe()?;
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
