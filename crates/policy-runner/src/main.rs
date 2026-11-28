mod app;
mod episode;
mod gripper;
mod log;
mod policy;
mod replay;
mod ros;
mod task;
mod ui;
mod worker;

use app::{App, Phase, Robot, Shared};
use clap::Parser;
use color_eyre::{Result, eyre::bail};
use log::Log;
use policy::Policy;
use ratatui::{
    DefaultTerminal,
    crossterm::event::{self, Event, KeyCode, KeyEvent, KeyModifiers},
};
use ros::{Mode, Ros};
use std::path::PathBuf;
use std::{
    io::{self, IsTerminal},
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
        mpsc,
    },
    thread,
    time::{Duration, Instant},
};
use task::TaskPack;

/// How long the display waits for a key before drawing again.
const FRAME: Duration = Duration::from_millis(50);
const POLICY_POLL_PERIOD: Duration = Duration::from_secs(1);

/// A `SIGINT` from outside the terminal, which in raw mode sends Ctrl-C as a key instead.
static INTERRUPTED: AtomicBool = AtomicBool::new(false);

/// Runs episodes of a policy over the `/act` contract `v0` on the `UR12e` and its gripper.
#[derive(Parser)]
struct Args {
    /// The policy server's base URL.
    #[arg(long, env = "WHISTLE_POLICY_URL")]
    policy: String,
    /// The `task_id` sent to the policy; `t` changes it between episodes.
    #[arg(long)]
    task: String,
    #[arg(long, env = "WHISTLE_MODE", value_enum, default_value = "real")]
    mode: Mode,
    #[arg(long, env = "WHISTLE_TASK_PACK")]
    task_pack: PathBuf,
    #[arg(
        long,
        env = "WHISTLE_WRIST_TOPIC",
        default_value = "/wrist_camera/color/image_raw/compressed"
    )]
    wrist_topic: String,
    #[arg(
        long,
        env = "WHISTLE_LEFT_TOPIC",
        default_value = "/left_camera/color/image_raw/compressed"
    )]
    left_topic: String,
    #[arg(
        long,
        env = "WHISTLE_RIGHT_TOPIC",
        default_value = "/right_camera/color/image_raw/compressed"
    )]
    right_topic: String,
    /// Accept home, run, and quit on stdin for one isolated mock replay.
    #[arg(long, requires = "mock_replay_steps")]
    mock_replay_stdio: bool,
    /// Run this number of replay steps, including any catch-up steps.
    #[arg(
        long,
        requires = "mock_replay_stdio",
        value_parser = clap::value_parser!(u32).range(1..=i64::from(episode::STEPS))
    )]
    mock_replay_steps: Option<u32>,
    /// Six recorded start joint angles in radians, in arm joint order.
    #[arg(
        long,
        requires = "mock_replay_stdio",
        num_args = 6,
        allow_negative_numbers = true
    )]
    mock_replay_start: Option<Vec<f64>>,
}

fn require_domain(mode: Mode, domain: Option<&str>, mock_replay: bool) -> Result<()> {
    if mock_replay && mode != Mode::Mock {
        bail!("mock replay requires --mode mock and ROS_DOMAIN_ID=77");
    }
    if mode == Mode::Mock && domain != Some("77") {
        bail!("the isolated Spark mock requires ROS_DOMAIN_ID=77");
    }
    Ok(())
}

fn main() -> Result<()> {
    color_eyre::install()?;
    let args = Args::parse();
    require_domain(
        args.mode,
        std::env::var("ROS_DOMAIN_ID").ok().as_deref(),
        args.mock_replay_stdio,
    )?;
    if args.wrist_topic == args.left_topic
        || args.wrist_topic == args.right_topic
        || args.left_topic == args.right_topic
    {
        bail!("each camera role must have a distinct ROS topic");
    }
    if !args.mock_replay_stdio && (!io::stdin().is_terminal() || !io::stdout().is_terminal()) {
        bail!("the policy runner needs a terminal: run it from an interactive shell");
    }
    let start_pose = replay::start_pose(args.mock_replay_start)?;
    let pack = TaskPack::open(&args.task_pack)?;
    let log = Arc::new(Log::new());
    let policy = Policy::new(&args.policy)?;
    let ros = Ros::new(
        args.mode,
        [&args.wrist_topic, &args.left_topic, &args.right_topic],
        Arc::clone(&log),
    )?;
    let shared = Shared::new(log);
    if let Some(steps) = args.mock_replay_steps {
        let result = replay::serve(&ros, &policy, &pack, &args.task, shared, &start_pose, steps);
        if let Err(error) = &result {
            let _ = replay::status(&format!("ERROR {error:#}"));
        }
        return result;
    }

    ctrlc::set_handler(|| INTERRUPTED.store(true, Ordering::Relaxed))?;
    let monitored_policy = Policy::new(&args.policy)?;

    // After color-eyre's panic hook, so the terminal is restored before a panic is printed.
    let terminal = ratatui::try_init()?;
    let result = thread::scope(|scope| {
        let (commands, received) = mpsc::channel();
        let worker = scope.spawn({
            let (ros, policy, shared, pack) = (&ros, &policy, &shared, &pack);
            move || worker::serve(ros, policy, shared, &received, pack)
        });
        scope.spawn(|| monitor(&monitored_policy, &shared));

        let app = App::new(&shared, commands, args.policy.clone(), args.task.clone());
        let stop = StopOnDrop(&shared);
        let ui = run_ui(terminal, app, &ros, || worker.is_finished());
        drop(stop);
        let worker = worker
            .join()
            .unwrap_or_else(|panic| std::panic::resume_unwind(panic));
        ui.and(worker)
    });
    ratatui::restore();

    for report in shared.reports.lock().iter() {
        println!("{report}");
    }
    result
}

/// Holds the arm if the display ends while it moves, and ends the policy monitor, also when the
/// display panics: the scope cannot end until both threads have.
struct StopOnDrop<'a>(&'a Shared);

impl Drop for StopOnDrop<'_> {
    fn drop(&mut self) {
        self.0.cancel.store(true, Ordering::Relaxed);
        self.0.quit.store(true, Ordering::Relaxed);
    }
}

/// Draws and handles keys until the operator quits or the arm worker ends. Dropping `app`
/// drops the command sender, which ends the worker.
fn run_ui(
    mut terminal: DefaultTerminal,
    mut app: App<'_>,
    ros: &Ros,
    worker_finished: impl Fn() -> bool,
) -> Result<()> {
    let ctrl_c = KeyEvent::new(KeyCode::Char('c'), KeyModifiers::CONTROL);
    loop {
        let robot = Robot::read(ros);
        terminal.draw(|frame| ui::draw(frame, &app, robot, ros.mode()))?;
        if app.quit || worker_finished() {
            return Ok(());
        }
        if INTERRUPTED.swap(false, Ordering::Relaxed) {
            app.handle(ctrl_c, robot);
        }
        if event::poll(FRAME)?
            && let Event::Key(key) = event::read()?
        {
            app.handle(key, robot);
        }
    }
}

/// Polls the policy about once a second, except during episodes, and logs what changes.
fn monitor(policy: &Policy, shared: &Shared) {
    while !shared.quit.load(Ordering::Relaxed) {
        let next_poll = Instant::now() + POLICY_POLL_PERIOD;
        if *shared.phase.lock() != Phase::Running {
            let status = policy.status();
            let mut latest = shared.policy.lock();
            let previous = latest.as_ref();
            if previous.is_none_or(|previous| previous.health != status.health) {
                shared.log.push(format!("Policy health: {}", status.health));
            }
            if previous.is_none_or(|previous| previous.readiness != status.readiness) {
                shared
                    .log
                    .push(format!("Policy readiness: {}", status.readiness));
            }
            *latest = Some(status);
        }
        while Instant::now() < next_poll && !shared.quit.load(Ordering::Relaxed) {
            thread::sleep(Duration::from_millis(50));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use test_case::test_case;

    #[test_case(Mode::Mock, Some("77"), false => matches Ok(()))]
    #[test_case(Mode::Real, Some("42"), false => matches Ok(()))]
    #[test_case(Mode::Mock, Some("42"), false => matches Err(_))]
    #[test_case(Mode::Mock, None, false => matches Err(_))]
    #[test_case(Mode::Real, None, false => matches Ok(()))]
    #[test_case(Mode::Mock, Some("77"), true => matches Ok(()))]
    #[test_case(Mode::Real, Some("77"), true => matches Err(_))]
    #[test_case(Mode::Mock, Some("42"), true => matches Err(_))]
    fn mode_and_ros_domain(mode: Mode, domain: Option<&str>, mock_replay: bool) -> Result<()> {
        require_domain(mode, domain, mock_replay)
    }

    #[test]
    fn mock_replay_accepts_negative_start_angles() {
        let args = Args::try_parse_from([
            "policy-runner",
            "--policy",
            "http://127.0.0.1:18080",
            "--task",
            "bolt_to_tray",
            "--task-pack",
            "manifest.json",
            "--mode",
            "mock",
            "--mock-replay-stdio",
            "--mock-replay-steps",
            "12",
            "--mock-replay-start",
            "-3.1477",
            "-1.6507",
            "-2.4943",
            "-0.6170",
            "1.4267",
            "-4.5194",
        ])
        .unwrap();
        assert_eq!(args.mode, Mode::Mock);
        assert_eq!(args.mock_replay_steps, Some(12));
        assert_eq!(
            replay::start_pose(args.mock_replay_start).unwrap(),
            worker::START_POSE
        );
    }
}
