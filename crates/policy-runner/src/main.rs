mod app;
mod episode;
mod gripper;
mod log;
mod policy;
mod ros;
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
use ros::{CAMERA_TOPIC, CameraSource, Ros};
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

/// How long the display waits for a key before drawing again.
const FRAME: Duration = Duration::from_millis(50);
const POLICY_POLL_PERIOD: Duration = Duration::from_secs(1);

/// A `SIGINT` from outside the terminal, which in raw mode sends Ctrl-C as a key instead.
static INTERRUPTED: AtomicBool = AtomicBool::new(false);

/// Runs episodes of a policy over the `/act` contract `v0` on the `UR12e` and its gripper.
#[derive(Parser)]
struct Args {
    /// The policy server's base URL.
    #[arg(long, default_value = "http://127.0.0.1:18081")]
    policy: String,
    /// The `task_id` sent to the policy; `t` changes it between episodes.
    #[arg(long)]
    task: String,
    /// Compressed-image topic used for camera frames and the live indicator.
    #[arg(long, default_value = CAMERA_TOPIC, conflicts_with = "no_camera")]
    camera_topic: String,
    /// Native compressed-image dimensions, WIDTHxHEIGHT; frames are not resized.
    #[arg(long, default_value = "640x360", value_parser = camera_size, conflicts_with = "no_camera")]
    camera_size: (usize, usize),
    /// Send a black frame instead of the camera's.
    #[arg(long)]
    no_camera: bool,
}

fn camera_size(value: &str) -> std::result::Result<(usize, usize), String> {
    let (width, height) = value.split_once('x').ok_or("expected WIDTHxHEIGHT")?;
    let width: usize = width.parse().map_err(|_| "invalid camera width")?;
    let height: usize = height.parse().map_err(|_| "invalid camera height")?;
    if !(1..=8192).contains(&width) || !(1..=8192).contains(&height) {
        return Err("camera dimensions must be between 1 and 8192".to_owned());
    }
    Ok((width, height))
}

fn main() -> Result<()> {
    color_eyre::install()?;
    let args = Args::parse();
    if !io::stdin().is_terminal() || !io::stdout().is_terminal() {
        bail!("the policy runner needs a terminal: run it from an interactive shell");
    }
    ctrlc::set_handler(|| INTERRUPTED.store(true, Ordering::Relaxed))?;

    let log = Arc::new(Log::new());
    let policy = Policy::new(&args.policy);
    let monitored_policy = Policy::new(&args.policy);
    let camera = if args.no_camera {
        CameraSource::BlackImage
    } else {
        CameraSource::RosTopic
    };
    let ros = Ros::new(
        camera,
        &args.camera_topic,
        args.camera_size,
        Arc::clone(&log),
    )?;
    let shared = Shared::new(log);

    // After color-eyre's panic hook, so the terminal is restored before a panic is printed.
    let terminal = ratatui::try_init()?;
    let result = thread::scope(|scope| {
        let (commands, received) = mpsc::channel();
        let worker = scope.spawn({
            let (ros, policy, shared) = (&ros, &policy, &shared);
            move || worker::serve(ros, policy, shared, &received)
        });
        scope.spawn(|| monitor(&monitored_policy, &shared));

        let mut app = App::new(
            &shared,
            commands,
            args.policy.clone(),
            camera,
            args.task.clone(),
        );
        app.camera_topic = args.camera_topic.clone();
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
        terminal.draw(|frame| ui::draw(frame, &app, robot))?;
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
mod camera_tests {
    use super::*;
    use test_case::test_case;

    #[test_case("640x360" => matches Ok((640, 360)))]
    #[test_case("640x480" => matches Ok((640, 480)))]
    #[test_case("0x480" => matches Err(_))]
    #[test_case("640x0" => matches Err(_))]
    #[test_case("bad" => matches Err(_))]
    fn dimensions(value: &str) -> std::result::Result<(usize, usize), String> {
        camera_size(value)
    }
}
