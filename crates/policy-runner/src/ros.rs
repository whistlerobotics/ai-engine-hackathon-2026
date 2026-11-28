use crate::{
    gripper::{GripperHardware, ROBOTIQ_PLUGIN},
    log::Log,
};
use base64::{Engine, engine::general_purpose::STANDARD as BASE64};
use color_eyre::{
    Result,
    eyre::{bail, eyre},
};
use parking_lot::Mutex;
use policy_contract::{
    ARM_JOINTS, Joints, gripper::FINGER_JOINT, image::check_policy_jpeg, wire::Images,
};
use rclrs::{
    Context, CreateBasicExecutor, IntoPrimitiveOptions, Node, Publisher, SpinOptions, Subscription,
};
use ros_env::{
    builtin_interfaces::msg::Duration as RosDuration,
    control_msgs::msg::JointTrajectoryControllerState,
    controller_manager_msgs::msg::ControllerManagerActivity,
    sensor_msgs::msg::{CompressedImage, JointState},
    std_msgs::msg::{Bool, Float64, Float64MultiArray, String as StringMsg},
    trajectory_msgs::msg::{JointTrajectory, JointTrajectoryPoint},
    ur_dashboard_msgs::msg::{RobotMode, SafetyMode},
};
use std::{
    sync::Arc,
    thread::{self, JoinHandle},
    time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};

pub const ROLES: [&str; 3] = ["wrist", "left", "right"];
const MAX_VIEW_SKEW: Duration = Duration::from_millis(60);
const SPEED_SCALING_TOPIC: &str = "/speed_scaling_state_broadcaster/speed_scaling";
const GRIPPER_DESCRIPTION_TOPIC: &str = "/gripper/robot_description";
const TRAJECTORY_TOPIC: &str = "/joint_trajectory_controller/joint_trajectory";
const FINGER_COMMAND_TOPIC: &str = "/gripper/finger_position_controller/commands";
const MOCK_PLUGIN: &str = "mock_components/GenericSystem";
const UR_PLUGIN: &str = "ur_robot_driver/URPositionHardwareInterface";
const HOLD_TIMEOUT: Duration = Duration::from_secs(1);
const HOLD_SETTLE: Duration = Duration::from_millis(100);
const HOLD_TOLERANCE: f64 = 0.02;

#[derive(Clone, Copy, Debug, PartialEq, Eq, clap::ValueEnum)]
pub enum Mode {
    Mock,
    Real,
}

#[derive(Debug, Clone)]
pub struct Received<T> {
    pub value: T,
    pub at: Instant,
}

impl<T> Received<T> {
    pub fn is_fresh(&self) -> bool {
        self.at.elapsed() <= STALE_AFTER
    }
}

#[derive(Debug, Clone, Copy)]
pub struct ArmState {
    pub position: Joints,
    pub velocity: Joints,
}

type Latest<T> = Arc<Mutex<Option<Received<T>>>>;

pub struct Ros {
    mode: Mode,
    arm: Latest<ArmState>,
    finger: Latest<f64>,
    /// The pendant's speed slider, in percent.
    speed_scaling: Latest<f64>,
    gripper_hardware: Arc<Mutex<Option<GripperHardware>>>,
    cameras: [Latest<Frame>; 3],
    arm_matches: Arc<Mutex<bool>>,
    gripper_matches: Arc<Mutex<bool>>,
    program_running: Arc<Mutex<bool>>,
    robot_running: Arc<Mutex<bool>>,
    safety_mode_ready: Arc<Mutex<bool>>,
    controller_active: Arc<Mutex<bool>>,
    finger_active: Arc<Mutex<bool>>,
    reference: Latest<Joints>,
    trajectory: Publisher<JointTrajectory>,
    finger_command: Publisher<Float64MultiArray>,
    _subscriptions: Subscriptions,
    _node: Node,
    _executor_thread: JoinHandle<()>,
}

#[derive(Clone)]
struct Frame {
    bytes: Arc<[u8]>,
    capture: Duration,
}

fn epoch() -> Duration {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
}

struct Subscriptions {
    _arm: Subscription<JointState>,
    _finger: Subscription<JointState>,
    _speed_scaling: Subscription<Float64>,
    _gripper_description: Subscription<StringMsg>,
    _cameras: Vec<Subscription<CompressedImage>>,
    _arm_description: Subscription<StringMsg>,
    _active: Subscription<ControllerManagerActivity>,
    _finger_active: Subscription<ControllerManagerActivity>,
    _reference: Subscription<JointTrajectoryControllerState>,
    _program_running: Subscription<Bool>,
    _robot_mode: Subscription<RobotMode>,
    _safety_mode: Subscription<SafetyMode>,
}

impl Ros {
    pub fn new(mode: Mode, topics: [&str; 3], log: Arc<Log>) -> Result<Self> {
        let context = Context::default_from_env()?;
        let mut executor = context.create_basic_executor();
        let node = executor.create_node("policy_runner")?;

        let arm = Latest::default();
        let arm_subscription =
            node.create_subscription("/joint_states".best_effort().keep_last(1), {
                let arm = Arc::clone(&arm);
                move |msg: JointState| {
                    let position = by_name(&msg.name, &msg.position, ARM_JOINTS);
                    let velocity = by_name(&msg.name, &msg.velocity, ARM_JOINTS);
                    if let (Some(position), Some(velocity)) = (position, velocity) {
                        store_stamped(
                            &arm,
                            ArmState { position, velocity },
                            msg.header.stamp.sec,
                            msg.header.stamp.nanosec,
                        );
                    }
                }
            })?;

        let finger = Latest::default();
        let finger_subscription =
            node.create_subscription("/gripper/joint_states".best_effort().keep_last(1), {
                let finger = Arc::clone(&finger);
                move |msg: JointState| {
                    if let Some([position]) = by_name(&msg.name, &msg.position, [FINGER_JOINT]) {
                        store_stamped(
                            &finger,
                            position,
                            msg.header.stamp.sec,
                            msg.header.stamp.nanosec,
                        );
                    }
                }
            })?;

        let speed_scaling = Latest::default();
        let speed_scaling_subscription = node.create_subscription(SPEED_SCALING_TOPIC, {
            let speed_scaling = Arc::clone(&speed_scaling);
            move |msg: Float64| store(&speed_scaling, msg.data)
        })?;

        let gripper_matches = Arc::new(Mutex::new(false));
        let gripper_hardware: Arc<Mutex<Option<GripperHardware>>> = Arc::default();
        // `robot_state_publisher` publishes the description once, transient local.
        let gripper_description_subscription =
            node.create_subscription(GRIPPER_DESCRIPTION_TOPIC.transient_local().keep_last(1), {
                let gripper_hardware = Arc::clone(&gripper_hardware);
                let log = Arc::clone(&log);
                let gripper_matches = Arc::clone(&gripper_matches);
                move |msg: StringMsg| {
                    let plugin = if mode == Mode::Real {
                        ROBOTIQ_PLUGIN
                    } else {
                        MOCK_PLUGIN
                    };
                    *gripper_matches.lock() = matches_hardware(&msg.data, plugin);
                    match GripperHardware::from_description(&msg.data) {
                        Ok(hardware) => *gripper_hardware.lock() = Some(hardware),
                        Err(error) => log.push(error.to_string()),
                    }
                }
            })?;

        let arm_matches = Arc::new(Mutex::new(false));
        let arm_description =
            node.create_subscription("/robot_description".transient_local().keep_last(1), {
                let arm_matches = Arc::clone(&arm_matches);
                let plugin = if mode == Mode::Real {
                    UR_PLUGIN
                } else {
                    MOCK_PLUGIN
                };
                move |msg: StringMsg| *arm_matches.lock() = matches_hardware(&msg.data, plugin)
            })?;
        // UR publishes these status messages on change, with transient-local durability.
        let program_running = Arc::new(Mutex::new(false));
        let program_subscription = node.create_subscription(
            "/io_and_status_controller/robot_program_running"
                .transient_local()
                .keep_last(1),
            {
                let status = Arc::clone(&program_running);
                move |msg: Bool| *status.lock() = msg.data
            },
        )?;
        let robot_running = Arc::new(Mutex::new(false));
        let robot_mode_subscription = node.create_subscription(
            "/io_and_status_controller/robot_mode"
                .transient_local()
                .keep_last(1),
            {
                let status = Arc::clone(&robot_running);
                move |msg: RobotMode| *status.lock() = msg.mode == RobotMode::RUNNING
            },
        )?;
        let safety_mode_ready = Arc::new(Mutex::new(false));
        let safety_mode_subscription = node.create_subscription(
            "/io_and_status_controller/safety_mode"
                .transient_local()
                .keep_last(1),
            {
                let status = Arc::clone(&safety_mode_ready);
                move |msg: SafetyMode| {
                    *status.lock() = matches!(msg.mode, SafetyMode::NORMAL | SafetyMode::REDUCED);
                }
            },
        )?;
        let controller_active = Arc::new(Mutex::new(false));
        let finger_active = Arc::new(Mutex::new(false));
        let activity = |topic: &'static str, name: &'static str, state: Arc<Mutex<bool>>| {
            node.create_subscription(
                topic.transient_local().keep_last(1),
                move |msg: ControllerManagerActivity| {
                    *state.lock() = msg
                        .controllers
                        .iter()
                        .any(|c| c.name == name && c.state.id == 3);
                },
            )
        };
        let active = activity(
            "/controller_manager/activity",
            "joint_trajectory_controller",
            Arc::clone(&controller_active),
        )?;
        let finger_status = activity(
            "/gripper/controller_manager/activity",
            "finger_position_controller",
            Arc::clone(&finger_active),
        )?;
        let reference = Latest::default();
        let ref_subscription = node.create_subscription(
            "/joint_trajectory_controller/controller_state"
                .best_effort()
                .keep_last(1),
            {
                let reference = Arc::clone(&reference);
                move |msg: JointTrajectoryControllerState| {
                    if let Some(position) =
                        by_name(&msg.joint_names, &msg.reference.positions, ARM_JOINTS)
                    {
                        store_stamped(
                            &reference,
                            position,
                            msg.header.stamp.sec,
                            msg.header.stamp.nanosec,
                        );
                    }
                }
            },
        )?;
        let cameras: [Latest<Frame>; 3] = std::array::from_fn(|_| Latest::default());
        let mut subscriptions = Vec::new();
        for (topic, frame) in topics.into_iter().zip(&cameras) {
            let frame = Arc::clone(frame);
            subscriptions.push(node.create_subscription(
                topic.best_effort().keep_last(1),
                move |msg: CompressedImage| {
                    if timestamp_fresh(msg.header.stamp.sec, msg.header.stamp.nanosec) {
                        let capture =
                            Duration::new(msg.header.stamp.sec as u64, msg.header.stamp.nanosec);
                        store(
                            &frame,
                            Frame {
                                bytes: msg.data.into(),
                                capture,
                            },
                        );
                    }
                },
            )?);
        }

        let trajectory = node.create_publisher(TRAJECTORY_TOPIC)?;
        let finger_command = node.create_publisher(FINGER_COMMAND_TOPIC)?;

        let executor_log = Arc::clone(&log);
        let executor_thread = thread::spawn(move || {
            for error in executor.spin(SpinOptions::default()) {
                executor_log.push(format!("ROS executor: {error}"));
            }
        });

        Ok(Self {
            mode,
            arm,
            finger,
            speed_scaling,
            gripper_hardware,
            cameras,
            arm_matches,
            gripper_matches,
            program_running,
            robot_running,
            safety_mode_ready,
            controller_active,
            finger_active,
            reference,
            trajectory,
            finger_command,
            _subscriptions: Subscriptions {
                _arm: arm_subscription,
                _finger: finger_subscription,
                _speed_scaling: speed_scaling_subscription,
                _gripper_description: gripper_description_subscription,
                _cameras: subscriptions,
                _arm_description: arm_description,
                _active: active,
                _finger_active: finger_status,
                _reference: ref_subscription,
                _program_running: program_subscription,
                _robot_mode: robot_mode_subscription,
                _safety_mode: safety_mode_subscription,
            },
            _node: node,
            _executor_thread: executor_thread,
        })
    }

    pub fn mode(&self) -> Mode {
        self.mode
    }

    pub fn arm(&self) -> Option<Received<ArmState>> {
        latest(&self.arm)
    }

    pub fn finger(&self) -> Option<Received<f64>> {
        latest(&self.finger)
    }

    pub fn speed_scaling(&self) -> Option<Received<f64>> {
        latest(&self.speed_scaling)
    }

    /// None until the gripper's robot description arrives.
    pub fn gripper_hardware(&self) -> Option<GripperHardware> {
        *self.gripper_hardware.lock()
    }

    pub fn camera_live(&self) -> [bool; 3] {
        camera_health(&self.cameras)
    }

    fn frames(&self) -> Result<[Received<Frame>; 3]> {
        let fetch = |index: usize| -> Result<Received<Frame>> {
            latest(&self.cameras[index])
                .filter(fresh_camera_frame)
                .ok_or_else(|| {
                    eyre!(
                        "{} camera missing or older than {} ms",
                        ROLES[index],
                        STALE_AFTER.as_millis()
                    )
                })
        };
        let frames = [fetch(0)?, fetch(1)?, fetch(2)?];
        let times = frames.each_ref().map(|f| f.value.capture);
        let earliest = times[0].min(times[1]).min(times[2]);
        let latest = times[0].max(times[1]).max(times[2]);
        if latest - earliest > MAX_VIEW_SKEW {
            bail!(
                "camera capture skew exceeds {} ms",
                MAX_VIEW_SKEW.as_millis()
            );
        }
        Ok(frames)
    }

    pub fn check_cameras(&self) -> Result<()> {
        self.frames().map(|_| ())
    }

    pub fn images(&self) -> Result<Images> {
        let [wrist, left, right] = self.frames()?;
        let encode = |role: &str, frame: Received<Frame>| {
            check_policy_jpeg(&frame.value.bytes).map_err(|e| eyre!("{role} camera: {e}"))?;
            Ok::<_, color_eyre::Report>(BASE64.encode(&frame.value.bytes))
        };
        Ok(Images {
            wrist: encode("wrist", wrist)?,
            left: encode("left", left)?,
            right: encode("right", right)?,
        })
    }

    fn isolated_arm(&self) -> bool {
        *self.arm_matches.lock() && matches!(self.trajectory.get_subscription_count(), Ok(1))
    }

    pub fn controller_issue(&self) -> Option<&'static str> {
        if !*self.arm_matches.lock() {
            return Some("arm description missing or hardware does not match selected mode");
        }
        if !*self.controller_active.lock() {
            return Some("joint_trajectory_controller is inactive or missing");
        }
        if !matches!(self.trajectory.get_subscription_count(), Ok(1)) {
            return Some("arm command topic requires exactly one controller subscriber");
        }
        if !self
            .reference
            .lock()
            .as_ref()
            .is_some_and(Received::is_fresh)
        {
            return Some("controller reference feedback is missing or stale");
        }
        if self.mode == Mode::Real {
            if !*self.robot_running.lock() || !*self.program_running.lock() {
                return Some("UR robot or External Control program is not running");
            }
            if !*self.safety_mode_ready.lock() {
                return Some("UR safety mode is unavailable or does not allow motion");
            }
            if self.speed_scaling().is_some_and(|sample| {
                sample.at.elapsed() <= Duration::from_secs(1) && sample.value <= 0.0
            }) {
                return Some("UR speed scaling is zero");
            }
        }
        None
    }

    pub fn gripper_issue(&self) -> Option<&'static str> {
        if !*self.gripper_matches.lock() {
            return Some("gripper description missing or hardware does not match selected mode");
        }
        if !*self.finger_active.lock() {
            return Some("finger_position_controller is inactive or missing");
        }
        if !matches!(self.finger_command.get_subscription_count(), Ok(1)) {
            return Some("gripper command topic requires exactly one controller subscriber");
        }
        None
    }

    pub fn controller_ready(&self) -> bool {
        self.controller_issue().is_none()
    }
    pub fn gripper_ready(&self) -> bool {
        self.gripper_issue().is_none()
    }

    pub fn reference(&self) -> Option<Received<Joints>> {
        latest(&self.reference).filter(Received::is_fresh)
    }

    /// One point per entry of `positions`, `segment` apart, starting on receipt.
    pub fn send_trajectory(&self, positions: &[Joints], segment: Duration) -> Result<()> {
        if self.mode == Mode::Real {
            if let Some(issue) = self.controller_issue() {
                bail!("{issue}");
            }
            if !positions.iter().all(ur12e_target_in_range) {
                bail!("trajectory target exceeds the UR12e joint range");
            }
        }
        let points = positions
            .iter()
            .zip(1..)
            .map(|(position, i)| JointTrajectoryPoint {
                positions: position.to_vec(),
                time_from_start: ros_duration(segment * i),
                ..Default::default()
            })
            .collect();
        self.publish_trajectory(points)
    }

    /// Makes the arm hold `position` at once, dropping the trajectory it was following.
    pub fn hold(&self, position: &Joints) -> Result<()> {
        let target = if self.mode == Mode::Real {
            self.arm()
                .filter(Received::is_fresh)
                .ok_or_else(|| eyre!("no fresh measured position for a hold command"))?
                .value
                .position
        } else {
            *position
        };
        let sent = Instant::now();
        self.publish_trajectory(vec![JointTrajectoryPoint {
            positions: target.to_vec(),
            ..Default::default()
        }])?;
        if self.mode == Mode::Real {
            self.confirm_hold(&target, sent)?;
        }
        Ok(())
    }

    fn confirm_hold(&self, target: &Joints, sent: Instant) -> Result<()> {
        let mut stable_since = None;
        while sent.elapsed() < HOLD_TIMEOUT {
            let reference_matches = self
                .reference()
                .is_some_and(|sample| sample.at > sent && near(&sample.value, target));
            let arm_still = self.arm().filter(Received::is_fresh).is_some_and(|sample| {
                sample.at > sent
                    && near(&sample.value.position, target)
                    && sample
                        .value
                        .velocity
                        .iter()
                        .all(|v| v.abs() <= HOLD_TOLERANCE)
            });
            if reference_matches && arm_still {
                if stable_since.get_or_insert_with(Instant::now).elapsed() >= HOLD_SETTLE {
                    return Ok(());
                }
            } else {
                stable_since = None;
            }
            thread::sleep(Duration::from_millis(10));
        }
        bail!("could not confirm the arm stopped at the hold target")
    }

    pub fn command_finger(&self, position: f64) -> Result<()> {
        if let Some(issue) = self.gripper_issue() {
            bail!("{issue}");
        }
        self.finger_command.publish(Float64MultiArray {
            data: vec![position],
            ..Default::default()
        })?;
        Ok(())
    }

    fn publish_trajectory(&self, points: Vec<JointTrajectoryPoint>) -> Result<()> {
        if !self.isolated_arm() {
            bail!("arm hardware does not match selected mode or command topic is not isolated");
        }
        self.trajectory.publish(JointTrajectory {
            joint_names: ARM_JOINTS.map(str::to_owned).to_vec(),
            points,
            ..Default::default()
        })?;
        Ok(())
    }
}

fn fresh_camera_frame(frame: &Received<Frame>) -> bool {
    frame.is_fresh()
        && epoch()
            .checked_sub(frame.value.capture)
            .is_some_and(|age| age <= STALE_AFTER)
}

fn camera_health(frames: &[Latest<Frame>; 3]) -> [bool; 3] {
    frames
        .each_ref()
        .map(|frame| frame.lock().as_ref().is_some_and(fresh_camera_frame))
}

fn near(measured: &Joints, target: &Joints) -> bool {
    measured
        .iter()
        .zip(target)
        .all(|(q, target)| (q - target).abs() <= HOLD_TOLERANCE)
}

fn ur12e_target_in_range(target: &Joints) -> bool {
    use std::f64::consts::TAU;
    target.iter().enumerate().all(|(joint, q)| {
        let limit = if joint == 2 {
            160.0_f64.to_radians()
        } else {
            TAU
        };
        q.abs() <= limit
    })
}

fn matches_hardware(xml: &str, expected: &str) -> bool {
    let Ok(doc) = roxmltree::Document::parse(xml) else {
        return false;
    };
    let controls: Vec<_> = doc
        .descendants()
        .filter(|n| n.has_tag_name("ros2_control") && n.attribute("type") == Some("system"))
        .collect();
    !controls.is_empty()
        && controls.iter().all(|c| {
            c.children()
                .find(|n| n.has_tag_name("hardware"))
                .and_then(|h| h.children().find(|n| n.has_tag_name("plugin")))
                .and_then(|p| p.text())
                .is_some_and(|p| p.trim() == expected)
        })
}

fn store_stamped<T>(sample: &Mutex<Option<Received<T>>>, value: T, sec: i32, nanos: u32) {
    if sec < 0 || nanos >= 1_000_000_000 {
        return;
    }
    if let Some(age) = epoch().checked_sub(Duration::new(sec as u64, nanos))
        && age <= STALE_AFTER
        && let Some(at) = Instant::now().checked_sub(age)
    {
        let mut last = sample.lock();
        if last.as_ref().is_none_or(|previous| at > previous.at) {
            *last = Some(Received { value, at });
        }
    }
}

fn timestamp_fresh(sec: i32, nanos: u32) -> bool {
    sec >= 0
        && nanos < 1_000_000_000
        && epoch()
            .checked_sub(Duration::new(sec as u64, nanos))
            .is_some_and(|age| age <= STALE_AFTER)
}

const STALE_AFTER: Duration = Duration::from_millis(100);

fn by_name<const N: usize>(
    names: &[String],
    values: &[f64],
    wanted: [&str; N],
) -> Option<[f64; N]> {
    let mut found = [0.0; N];
    for (slot, name) in found.iter_mut().zip(wanted) {
        let index = names.iter().position(|n| n == name)?;
        *slot = *values.get(index)?;
    }
    // `ros2_control` state interfaces can read NaN before the hardware fills them in.
    found.iter().all(|v| v.is_finite()).then_some(found)
}

fn store<T>(latest: &Mutex<Option<Received<T>>>, value: T) {
    *latest.lock() = Some(Received {
        value,
        at: Instant::now(),
    });
}

fn latest<T: Clone>(latest: &Mutex<Option<Received<T>>>) -> Option<Received<T>> {
    latest.lock().clone()
}

fn ros_duration(duration: Duration) -> RosDuration {
    RosDuration {
        sec: i32::try_from(duration.as_secs()).unwrap_or(i32::MAX),
        nanosec: duration.subsec_nanos(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use policy_contract::ARM_JOINTS;
    use std::assert_matches;
    use test_case::test_case;

    #[test_case("<broken" => false)]
    #[test_case("<robot/>" => false)]
    #[test_case("<robot><ros2_control type='system'><hardware><plugin>custom/Unknown</plugin></hardware></ros2_control></robot>" => false)]
    #[test_case("<robot><ros2_control type='system'><hardware><plugin>mock_components/GenericSystem</plugin></hardware></ros2_control></robot>" => true)]
    #[test_case("<robot><ros2_control type='system'><hardware><plugin>mock_components/GenericSystem</plugin></hardware></ros2_control><ros2_control type='system'><hardware><plugin>ur_robot_driver/URPositionHardwareInterface</plugin></hardware></ros2_control></robot>" => false)]
    fn verify_mock_identity(xml: &str) -> bool {
        matches_hardware(xml, MOCK_PLUGIN)
    }

    #[test]
    fn real_arm_rejects_mock_and_mixed_descriptions() {
        let arm = |plugin| {
            format!(
                "<ros2_control type='system'><hardware><plugin>{plugin}</plugin></hardware></ros2_control>"
            )
        };
        let real = arm(UR_PLUGIN);
        let mock = arm(MOCK_PLUGIN);
        assert!(matches_hardware(
            &format!("<robot>{real}</robot>"),
            UR_PLUGIN
        ));
        assert!(!matches_hardware(
            &format!("<robot>{mock}</robot>"),
            UR_PLUGIN
        ));
        assert!(!matches_hardware(
            &format!("<robot>{real}{mock}</robot>"),
            UR_PLUGIN
        ));
    }

    #[test]
    fn real_targets_reject_out_of_range_or_nonfinite_positions() {
        let mut target = crate::worker::START_POSE;
        assert!(ur12e_target_in_range(&target));
        target[2] = 161.0_f64.to_radians();
        assert!(!ur12e_target_in_range(&target));
        target[2] = 0.0;
        target[5] = f64::NAN;
        assert!(!ur12e_target_in_range(&target));
    }

    #[test_case(0)]
    #[test_case(1)]
    #[test_case(2)]
    fn camera_freshness_is_independent(index: usize) {
        let frames: [Latest<Frame>; 3] = std::array::from_fn(|_| Latest::default());
        assert_eq!(camera_health(&frames), [false; 3]);
        let receipt = epoch();
        store(
            &frames[index],
            Frame {
                bytes: Arc::from([]),
                capture: receipt,
            },
        );
        let mut expected = [false; 3];
        expected[index] = true;
        assert_eq!(camera_health(&frames), expected);
        frames[index].lock().as_mut().unwrap().value.capture -= Duration::from_secs(1);
        assert_eq!(camera_health(&frames), [false; 3]);
        store(
            &frames[index],
            Frame {
                bytes: Arc::from([]),
                capture: receipt,
            },
        );
        frames[index].lock().as_mut().unwrap().at -= Duration::from_secs(1);
        assert_eq!(camera_health(&frames), [false; 3]);
    }

    #[test]
    fn joints_by_name_in_any_order() {
        let names = [
            "elbow_joint",
            "shoulder_lift_joint",
            "shoulder_pan_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        .map(str::to_owned);
        let values = [3.0, 2.0, 1.0, 4.0, 5.0, 6.0];
        assert_matches!(
            by_name(&names, &values, ARM_JOINTS),
            Some([1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
        );
    }
}
