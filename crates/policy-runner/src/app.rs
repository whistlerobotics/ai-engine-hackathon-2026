use crate::{
    episode::Progress,
    gripper::GripperHardware,
    log::Log,
    policy::{PolicyStatus, Readiness},
    ros::{CameraSource, Received, Ros},
    worker::distance_to_start,
};
use parking_lot::Mutex;
use ratatui::crossterm::event::{KeyCode, KeyEvent, KeyEventKind, KeyModifiers};
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
    mpsc::Sender,
};

/// How close every joint must be to count as at the start pose.
const AT_START_TOLERANCE: f64 = 0.02;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Phase {
    Idle,
    MovingToStart,
    AtStart,
    Running,
}

impl Phase {
    pub const fn arm_moving(self) -> bool {
        matches!(self, Self::MovingToStart | Self::Running)
    }
}

/// What the main thread asks the arm worker to do.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Command {
    MoveToStart,
    RunEpisode { task_id: String },
}

/// What the arm worker and the policy monitor publish, and the display reads.
pub struct Shared {
    pub phase: Mutex<Phase>,
    pub progress: Mutex<Option<Progress>>,
    /// The latest poll and when it was taken.
    pub policy: Mutex<Option<PolicyStatus>>,
    pub reports: Mutex<Vec<String>>,
    pub log: Arc<Log>,
    /// Makes the arm worker stop the arm and hold.
    pub cancel: AtomicBool,
    /// Ends the policy monitor.
    pub quit: AtomicBool,
}

impl Shared {
    pub const fn new(log: Arc<Log>) -> Self {
        Self {
            phase: Mutex::new(Phase::Idle),
            progress: Mutex::new(None),
            policy: Mutex::new(None),
            reports: Mutex::new(Vec::new()),
            log,
            cancel: AtomicBool::new(false),
            quit: AtomicBool::new(false),
        }
    }
}

/// The robot as the display and the readiness checks see it.
#[derive(Debug, Clone, Copy)]
pub struct Robot {
    pub arm_live: bool,
    pub gripper_live: bool,
    /// None without a camera topic.
    pub camera_live: Option<bool>,
    pub at_start: bool,
    /// The pendant's speed slider, in percent; None while stale.
    pub speed_scaling: Option<f64>,
    /// None until the gripper's robot description arrives.
    pub gripper_hardware: Option<GripperHardware>,
}

impl Robot {
    pub fn read(ros: &Ros) -> Self {
        let arm = ros.arm().filter(Received::is_fresh);
        Self {
            arm_live: arm.is_some(),
            gripper_live: ros.finger().as_ref().is_some_and(Received::is_fresh),
            camera_live: ros.camera_live(),
            at_start: arm
                .is_some_and(|arm| distance_to_start(&arm.value.position) <= AT_START_TOLERANCE),
            speed_scaling: ros
                .speed_scaling()
                .filter(Received::is_fresh)
                .map(|received| received.value),
            gripper_hardware: ros.gripper_hardware(),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Action {
    MoveToStart,
    StartEpisode,
    Abort,
    Quit,
    EditTask,
    TypeTask(char),
    EraseTask,
    ConfirmTask,
    CancelTask,
    None,
}

/// What `key` means in `phase`, or while the task is being edited. `Space` is deliberately not
/// an abort key: it is too easily hit by mistake.
#[expect(
    clippy::wildcard_enum_match_arm,
    reason = "most keys mean nothing while typing a task"
)]
pub fn action(phase: Phase, editing: bool, key: KeyEvent) -> Action {
    if key.kind != KeyEventKind::Press {
        return Action::None;
    }
    if key.modifiers.contains(KeyModifiers::CONTROL) {
        return match (key.code, phase.arm_moving()) {
            (KeyCode::Char('c'), true) => Action::Abort,
            (KeyCode::Char('c'), false) => Action::Quit,
            _ => Action::None,
        };
    }
    if editing {
        return match key.code {
            KeyCode::Enter => Action::ConfirmTask,
            KeyCode::Esc => Action::CancelTask,
            KeyCode::Backspace => Action::EraseTask,
            KeyCode::Char(c) => Action::TypeTask(c),
            _ => Action::None,
        };
    }
    match (phase, key.code) {
        (Phase::Idle, KeyCode::Enter) => Action::MoveToStart,
        (Phase::AtStart, KeyCode::Enter) => Action::StartEpisode,
        (Phase::MovingToStart | Phase::Running, KeyCode::Esc) => Action::Abort,
        (Phase::Idle | Phase::AtStart, KeyCode::Char('q')) => Action::Quit,
        (Phase::Idle | Phase::AtStart, KeyCode::Char('t')) => Action::EditTask,
        _ => Action::None,
    }
}

/// The main thread's state: what it shows and what the operator's keys change.
pub struct App<'a> {
    pub shared: &'a Shared,
    commands: Sender<Command>,
    pub policy_url: String,
    pub camera: CameraSource,
    pub camera_topic: String,
    pub task_id: String,
    /// The new task being typed.
    pub editing: Option<String>,
    pub quit: bool,
}

impl<'a> App<'a> {
    pub fn new(
        shared: &'a Shared,
        commands: Sender<Command>,
        policy_url: String,
        camera: CameraSource,
        task_id: String,
    ) -> Self {
        Self {
            shared,
            commands,
            policy_url,
            camera,
            camera_topic: crate::ros::CAMERA_TOPIC.to_owned(),
            task_id,
            editing: None,
            quit: false,
        }
    }

    pub fn handle(&mut self, key: KeyEvent, robot: Robot) {
        let phase = *self.shared.phase.lock();
        match action(phase, self.editing.is_some(), key) {
            Action::MoveToStart => self.send(Command::MoveToStart, Phase::MovingToStart, robot),
            Action::StartEpisode => {
                let task_id = self.task_id.clone();
                self.send(Command::RunEpisode { task_id }, Phase::Running, robot);
            }
            Action::Abort => self.shared.cancel.store(true, Ordering::Relaxed),
            Action::Quit => self.quit = true,
            Action::EditTask => self.editing = Some(String::new()),
            Action::TypeTask(c) => self.editing.iter_mut().for_each(|task| task.push(c)),
            Action::EraseTask => self.editing.iter_mut().for_each(|task| _ = task.pop()),
            Action::ConfirmTask => self.confirm_task(),
            Action::CancelTask => self.editing = None,
            Action::None => {}
        }
    }

    /// Sends `command` to the arm worker and enters `phase` at once, so that a second key
    /// press cannot queue another command; or logs why not.
    fn send(&self, command: Command, phase: Phase, robot: Robot) {
        if let Some(reason) = self.refusal(&command, robot) {
            self.shared.log.push(format!("Enter refused: {reason}."));
            return;
        }
        self.shared.cancel.store(false, Ordering::Relaxed);
        *self.shared.phase.lock() = phase;
        if self.commands.send(command).is_err() {
            self.shared.log.push("The arm worker has stopped.");
        }
    }

    /// Moving to the start pose commands only the arm, so it needs neither the policy nor the
    /// gripper.
    fn refusal(&self, command: &Command, robot: Robot) -> Option<String> {
        let episode = matches!(command, Command::RunEpisode { .. });
        let mut reasons = Vec::new();
        if episode {
            match self.shared.policy.lock().as_ref() {
                Some(status) if status.readiness == Readiness::Ready => {}
                Some(status) => reasons.push(format!("the policy is {}", status.readiness)),
                None => reasons.push("the policy has not been polled yet".to_owned()),
            }
        }
        if !robot.arm_live {
            reasons.push("the arm's joint state is stale".to_owned());
        }
        if episode && !robot.gripper_live {
            reasons.push("the gripper's joint state is stale".to_owned());
        }
        (!reasons.is_empty()).then(|| reasons.join("; "))
    }

    fn confirm_task(&mut self) {
        let Some(task) = self.editing.as_deref().map(str::trim) else {
            return;
        };
        if task.is_empty() {
            self.shared.log.push("The task cannot be empty.");
            return;
        }
        self.task_id = task.to_owned();
        self.shared.log.push(format!("Task: {task}"));
        self.editing = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::policy::Health;
    use std::{
        assert_matches,
        sync::mpsc::{self, Receiver},
    };
    use test_case::test_case;

    fn press(code: KeyCode) -> KeyEvent {
        KeyEvent::new(code, KeyModifiers::NONE)
    }

    fn ctrl_c() -> KeyEvent {
        KeyEvent::new(KeyCode::Char('c'), KeyModifiers::CONTROL)
    }

    #[test_case(Phase::Idle, press(KeyCode::Enter) => Action::MoveToStart; "idle enter")]
    #[test_case(Phase::Idle, press(KeyCode::Esc) => Action::None; "idle esc")]
    #[test_case(Phase::Idle, ctrl_c() => Action::Quit; "idle ctrl c")]
    #[test_case(Phase::Idle, press(KeyCode::Char('q')) => Action::Quit; "idle q")]
    #[test_case(Phase::Idle, press(KeyCode::Char('t')) => Action::EditTask; "idle t")]
    #[test_case(Phase::MovingToStart, press(KeyCode::Enter) => Action::None; "moving enter")]
    #[test_case(Phase::MovingToStart, press(KeyCode::Esc) => Action::Abort; "moving esc")]
    #[test_case(Phase::MovingToStart, ctrl_c() => Action::Abort; "moving ctrl c")]
    #[test_case(Phase::MovingToStart, press(KeyCode::Char('q')) => Action::None; "moving q")]
    #[test_case(Phase::MovingToStart, press(KeyCode::Char('t')) => Action::None; "moving t")]
    #[test_case(Phase::AtStart, press(KeyCode::Enter) => Action::StartEpisode; "at start enter")]
    #[test_case(Phase::AtStart, press(KeyCode::Esc) => Action::None; "at start esc")]
    #[test_case(Phase::AtStart, ctrl_c() => Action::Quit; "at start ctrl c")]
    #[test_case(Phase::AtStart, press(KeyCode::Char('q')) => Action::Quit; "at start q")]
    #[test_case(Phase::AtStart, press(KeyCode::Char('t')) => Action::EditTask; "at start t")]
    #[test_case(Phase::Running, press(KeyCode::Enter) => Action::None; "running enter")]
    #[test_case(Phase::Running, press(KeyCode::Esc) => Action::Abort; "running esc")]
    #[test_case(Phase::Running, ctrl_c() => Action::Abort; "running ctrl c")]
    #[test_case(Phase::Running, press(KeyCode::Char('q')) => Action::None; "running q")]
    #[test_case(Phase::Running, press(KeyCode::Char('t')) => Action::None; "running t")]
    fn key_table(phase: Phase, key: KeyEvent) -> Action {
        action(phase, false, key)
    }

    #[test_case(Phase::Idle)]
    #[test_case(Phase::MovingToStart)]
    #[test_case(Phase::AtStart)]
    #[test_case(Phase::Running)]
    fn space_does_nothing(phase: Phase) {
        assert_eq!(
            action(phase, false, press(KeyCode::Char(' '))),
            Action::None
        );
    }

    #[test_case(KeyEventKind::Repeat)]
    #[test_case(KeyEventKind::Release)]
    fn only_presses_count(kind: KeyEventKind) {
        let key = KeyEvent::new_with_kind(KeyCode::Enter, KeyModifiers::NONE, kind);
        assert_eq!(action(Phase::AtStart, false, key), Action::None);
    }

    #[test_case(press(KeyCode::Char('q')) => Action::TypeTask('q'); "q types")]
    #[test_case(press(KeyCode::Char(' ')) => Action::TypeTask(' '); "space types")]
    #[test_case(press(KeyCode::Backspace) => Action::EraseTask; "backspace erases")]
    #[test_case(press(KeyCode::Enter) => Action::ConfirmTask; "enter confirms")]
    #[test_case(press(KeyCode::Esc) => Action::CancelTask; "esc cancels")]
    #[test_case(ctrl_c() => Action::Quit; "ctrl c quits")]
    fn task_editor(key: KeyEvent) -> Action {
        action(Phase::Idle, true, key)
    }

    const LIVE: Robot = Robot {
        arm_live: true,
        gripper_live: true,
        camera_live: None,
        at_start: false,
        speed_scaling: Some(100.0),
        gripper_hardware: None,
    };

    fn shared_with(readiness: Option<Readiness>) -> Shared {
        let shared = Shared::new(Arc::new(Log::new()));
        *shared.policy.lock() = readiness.map(|readiness| PolicyStatus {
            health: Health::Up,
            readiness,
        });
        shared
    }

    fn app(shared: &Shared) -> (App<'_>, Receiver<Command>) {
        let (commands, received) = mpsc::channel();
        let app = App::new(
            shared,
            commands,
            "url".to_owned(),
            CameraSource::BlackImage,
            "task".to_owned(),
        );
        (app, received)
    }

    fn last_log(shared: &Shared) -> String {
        shared
            .log
            .latest(1)
            .into_iter()
            .map(|line| line.text)
            .collect()
    }

    #[test_case(Phase::Idle, Some(Readiness::Ready), Robot { arm_live: false, ..LIVE }
        => "Enter refused: the arm's joint state is stale."; "move with stale arm")]
    #[test_case(Phase::AtStart, None, LIVE
        => "Enter refused: the policy has not been polled yet."; "episode, policy not polled")]
    #[test_case(Phase::AtStart, Some(Readiness::Unreachable("refused".to_owned())), LIVE
        => "Enter refused: the policy is unreachable (refused)."; "episode, policy unreachable")]
    #[test_case(Phase::AtStart, Some(Readiness::NotReady), LIVE
        => "Enter refused: the policy is not ready."; "episode, policy not ready")]
    #[test_case(
        Phase::AtStart,
        Some(Readiness::Incompatible { contract: Some("v1".to_owned()), schema_version: Some(2) }),
        LIVE
        => "Enter refused: the policy is incompatible: contract v1, schema version 2, not v0, 2.";
        "episode, policy incompatible")]
    #[test_case(Phase::AtStart, Some(Readiness::Ready), Robot { arm_live: false, ..LIVE }
        => "Enter refused: the arm's joint state is stale."; "episode with stale arm")]
    #[test_case(Phase::AtStart, Some(Readiness::Ready), Robot { gripper_live: false, ..LIVE }
        => "Enter refused: the gripper's joint state is stale."; "episode with stale gripper")]
    fn enter_refused(phase: Phase, readiness: Option<Readiness>, robot: Robot) -> String {
        let shared = shared_with(readiness);
        *shared.phase.lock() = phase;
        let (mut app, received) = app(&shared);
        app.handle(press(KeyCode::Enter), robot);
        assert_matches!(received.try_recv(), Err(_));
        assert_eq!(*shared.phase.lock(), phase);
        last_log(&shared)
    }

    #[test_case(None, LIVE; "policy not polled")]
    #[test_case(Some(Readiness::Unreachable("refused".to_owned())), LIVE; "policy unreachable")]
    #[test_case(Some(Readiness::Ready), Robot { gripper_live: false, ..LIVE }; "stale gripper")]
    fn move_needs_only_arm(readiness: Option<Readiness>, robot: Robot) {
        let shared = shared_with(readiness);
        let (mut app, received) = app(&shared);
        app.handle(press(KeyCode::Enter), robot);
        assert_eq!(*shared.phase.lock(), Phase::MovingToStart);
        assert_eq!(
            received.try_iter().collect::<Vec<_>>(),
            [Command::MoveToStart]
        );
    }

    #[test]
    fn second_enter_sends_nothing() {
        let shared = shared_with(Some(Readiness::Ready));
        let (mut app, received) = app(&shared);
        app.handle(press(KeyCode::Enter), LIVE);
        app.handle(press(KeyCode::Enter), LIVE);
        assert_eq!(*shared.phase.lock(), Phase::MovingToStart);
        assert_eq!(
            received.try_iter().collect::<Vec<_>>(),
            [Command::MoveToStart]
        );
    }

    #[test]
    fn episode_takes_the_task() {
        let shared = shared_with(Some(Readiness::Ready));
        *shared.phase.lock() = Phase::AtStart;
        let (mut app, received) = app(&shared);
        for key in [KeyCode::Char('t'), KeyCode::Char('b'), KeyCode::Char('x')] {
            app.handle(press(key), LIVE);
        }
        app.handle(press(KeyCode::Backspace), LIVE);
        app.handle(press(KeyCode::Enter), LIVE);
        assert_eq!(app.task_id, "b");
        app.handle(press(KeyCode::Enter), LIVE);
        assert_eq!(
            received.try_iter().collect::<Vec<_>>(),
            [Command::RunEpisode {
                task_id: "b".to_owned()
            }]
        );
    }

    #[test]
    fn empty_task_refused() {
        let shared = shared_with(None);
        let (mut app, _received) = app(&shared);
        app.handle(press(KeyCode::Char('t')), LIVE);
        app.handle(press(KeyCode::Char(' ')), LIVE);
        app.handle(press(KeyCode::Enter), LIVE);
        assert_eq!(app.task_id, "task");
        assert_eq!(app.editing.as_deref(), Some(" "));
        assert_eq!(last_log(&shared), "The task cannot be empty.");
    }

    #[test]
    fn discarded_task() {
        let shared = shared_with(None);
        let (mut app, _received) = app(&shared);
        app.handle(press(KeyCode::Char('t')), LIVE);
        app.handle(press(KeyCode::Char('x')), LIVE);
        app.handle(press(KeyCode::Esc), LIVE);
        assert_eq!(app.task_id, "task");
        assert_eq!(app.editing, None);
    }

    #[test]
    fn abort_sets_cancel() {
        let shared = shared_with(Some(Readiness::Ready));
        *shared.phase.lock() = Phase::Running;
        let (mut app, _received) = app(&shared);
        app.handle(press(KeyCode::Esc), LIVE);
        assert_matches!(shared.cancel.load(Ordering::Relaxed), true);
    }
}
