use crate::{gripper::GripperHardware, log::Log};
use base64::{Engine, engine::general_purpose::STANDARD as BASE64};
use color_eyre::{
    Result,
    eyre::{bail, eyre},
};
use parking_lot::Mutex;
use policy_contract::{
    ARM_JOINTS, Joints,
    gripper::FINGER_JOINT,
    image::{BLACK_JPEG, check_jpeg_size},
};
use rclrs::{
    Context, CreateBasicExecutor, IntoPrimitiveOptions, Node, Publisher, SpinOptions, Subscription,
};
use ros_env::{
    builtin_interfaces::msg::Duration as RosDuration,
    sensor_msgs::msg::{CompressedImage, JointState},
    std_msgs::msg::{Float64, Float64MultiArray, String as StringMsg},
    trajectory_msgs::msg::{JointTrajectory, JointTrajectoryPoint},
};
use std::{
    sync::Arc,
    thread::{self, JoinHandle},
    time::{Duration, Instant},
};

const CAMERA_TOPICS: [&str; 3] = [
    "/left_camera/color/image_raw/compressed",
    "/right_camera/color/image_raw/compressed",
    "/wrist_camera/color/image_raw/compressed",
];
pub const CAMERA_TOPIC: &str = CAMERA_TOPICS[1];
const SPEED_SCALING_TOPIC: &str = "/speed_scaling_state_broadcaster/speed_scaling";
const GRIPPER_DESCRIPTION_TOPIC: &str = "/gripper/robot_description";
const TRAJECTORY_TOPIC: &str = "/joint_trajectory_controller/joint_trajectory";
const FINGER_COMMAND_TOPIC: &str = "/gripper/finger_position_controller/commands";

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
    arm: Latest<ArmState>,
    finger: Latest<f64>,
    /// The pendant's speed slider, in percent.
    speed_scaling: Latest<f64>,
    gripper_hardware: Arc<Mutex<Option<GripperHardware>>>,
    camera: Camera,
    cameras: Cameras,
    camera_size: (usize, usize),
    trajectory: Publisher<JointTrajectory>,
    finger_command: Publisher<Float64MultiArray>,
    _subscriptions: Subscriptions,
    _node: Node,
    _executor_thread: JoinHandle<()>,
}

/// Where the images sent to the policy come from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CameraSource {
    RosTopic,
    BlackImage,
}

enum Camera {
    Black { base64: String },
    Topic(Latest<Vec<u8>>),
}

#[derive(Default)]
struct Cameras {
    frames: [Latest<Vec<u8>>; 3],
}

impl Cameras {
    fn frame(&self, topic: &str) -> Option<Latest<Vec<u8>>> {
        CAMERA_TOPICS
            .iter()
            .position(|candidate| *candidate == topic)
            .map(|index| Arc::clone(&self.frames[index]))
    }

    fn live(&self) -> [bool; 3] {
        self.frames
            .each_ref()
            .map(|frame| frame.lock().as_ref().is_some_and(Received::is_fresh))
    }
}

fn subscribe_camera(
    node: &Node,
    topic: &str,
    frame: &Latest<Vec<u8>>,
) -> Result<Subscription<CompressedImage>> {
    // Camera drivers publish best-effort, which a reliable subscription wouldn't match.
    let frame = Arc::clone(frame);
    Ok(node.create_subscription(topic.best_effort().keep_last(1), {
        move |msg: CompressedImage| store(&frame, msg.data)
    })?)
}

struct Subscriptions {
    _arm: Subscription<JointState>,
    _finger: Subscription<JointState>,
    _speed_scaling: Subscription<Float64>,
    _gripper_description: Subscription<StringMsg>,
    _cameras: Vec<Subscription<CompressedImage>>,
}

impl Ros {
    pub fn new(
        source: CameraSource,
        camera_topic: &str,
        camera_size: (usize, usize),
        log: Arc<Log>,
    ) -> Result<Self> {
        let context = Context::default_from_env()?;
        let mut executor = context.create_basic_executor();
        let node = executor.create_node("policy_runner")?;

        let arm = Latest::default();
        let arm_subscription = node.create_subscription("/joint_states", {
            let arm = Arc::clone(&arm);
            move |msg: JointState| {
                let position = by_name(&msg.name, &msg.position, ARM_JOINTS);
                let velocity = by_name(&msg.name, &msg.velocity, ARM_JOINTS);
                if let (Some(position), Some(velocity)) = (position, velocity) {
                    store(&arm, ArmState { position, velocity });
                }
            }
        })?;

        let finger = Latest::default();
        let finger_subscription = node.create_subscription("/gripper/joint_states", {
            let finger = Arc::clone(&finger);
            move |msg: JointState| {
                if let Some([position]) = by_name(&msg.name, &msg.position, [FINGER_JOINT]) {
                    store(&finger, position);
                }
            }
        })?;

        let speed_scaling = Latest::default();
        let speed_scaling_subscription = node.create_subscription(SPEED_SCALING_TOPIC, {
            let speed_scaling = Arc::clone(&speed_scaling);
            move |msg: Float64| store(&speed_scaling, msg.data)
        })?;

        let gripper_hardware: Arc<Mutex<Option<GripperHardware>>> = Arc::default();
        // `robot_state_publisher` publishes the description once, transient local.
        let gripper_description_subscription =
            node.create_subscription(GRIPPER_DESCRIPTION_TOPIC.transient_local().keep_last(1), {
                let gripper_hardware = Arc::clone(&gripper_hardware);
                let log = Arc::clone(&log);
                move |msg: StringMsg| match GripperHardware::from_description(&msg.data) {
                    Ok(hardware) => *gripper_hardware.lock() = Some(hardware),
                    Err(error) => log.push(error.to_string()),
                }
            })?;

        let cameras = Cameras::default();
        let mut camera_subscriptions = Vec::new();
        for (topic, frame) in CAMERA_TOPICS.iter().zip(&cameras.frames) {
            camera_subscriptions.push(subscribe_camera(&node, topic, frame)?);
        }
        let camera = match source {
            CameraSource::RosTopic => {
                let frame = if let Some(frame) = cameras.frame(camera_topic) {
                    frame
                } else {
                    let frame = Latest::default();
                    camera_subscriptions.push(subscribe_camera(&node, camera_topic, &frame)?);
                    frame
                };
                Camera::Topic(frame)
            }
            CameraSource::BlackImage => Camera::Black {
                base64: BASE64.encode(BLACK_JPEG),
            },
        };

        let trajectory = node.create_publisher(TRAJECTORY_TOPIC)?;
        let finger_command = node.create_publisher(FINGER_COMMAND_TOPIC)?;

        let executor_thread = thread::spawn(move || {
            for error in executor.spin(SpinOptions::default()) {
                log.push(format!("ROS executor: {error}"));
            }
        });

        Ok(Self {
            arm,
            finger,
            speed_scaling,
            gripper_hardware,
            camera,
            cameras,
            camera_size,
            trajectory,
            finger_command,
            _subscriptions: Subscriptions {
                _arm: arm_subscription,
                _finger: finger_subscription,
                _speed_scaling: speed_scaling_subscription,
                _gripper_description: gripper_description_subscription,
                _cameras: camera_subscriptions,
            },
            _node: node,
            _executor_thread: executor_thread,
        })
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

    /// Whether the camera's latest frame is fresh; None without a camera topic.
    pub fn camera_live(&self) -> Option<bool> {
        match &self.camera {
            Camera::Black { .. } => None,
            Camera::Topic(frame) => Some(frame.lock().as_ref().is_some_and(Received::is_fresh)),
        }
    }

    /// Left, right, and wrist feed freshness, independent of the selected policy input.
    pub fn cameras_live(&self) -> [bool; 3] {
        self.cameras.live()
    }

    /// The latest camera frame, base64-encoded, once it passes the frame checks; or the black
    /// frame without a camera.
    pub fn image(&self) -> Result<String> {
        match &self.camera {
            Camera::Black { base64 } => Ok(base64.clone()),
            Camera::Topic(frame) => {
                let frame = latest(frame).ok_or_else(|| eyre!("no camera frame received yet"))?;
                let age = frame.at.elapsed();
                if age > STALE_AFTER {
                    bail!("the latest camera frame is {} ms old", age.as_millis());
                }
                check_jpeg_size(&frame.value, self.camera_size)?;
                Ok(BASE64.encode(&frame.value))
            }
        }
    }

    /// One point per entry of `positions`, `segment` apart, starting on receipt.
    pub fn send_trajectory(&self, positions: &[Joints], segment: Duration) -> Result<()> {
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

    /// Like `send_trajectory`, for an episode's steps: each point also carries the velocity of the
    /// step ending at it, so the controller runs one message into the next instead of slowing to a
    /// stop at the end of every step.
    pub fn send_steps(&self, from: &Joints, positions: &[Joints], segment: Duration) -> Result<()> {
        let points = positions
            .iter()
            .zip(step_velocities(from, positions, segment))
            .zip(1..)
            .map(|((position, velocity), i)| JointTrajectoryPoint {
                positions: position.to_vec(),
                velocities: velocity.to_vec(),
                time_from_start: ros_duration(segment * i),
                ..Default::default()
            })
            .collect();
        self.publish_trajectory(points)
    }

    /// Makes the arm hold `position` at once, dropping the trajectory it was following.
    pub fn hold(&self, position: &Joints) -> Result<()> {
        self.publish_trajectory(vec![JointTrajectoryPoint {
            positions: position.to_vec(),
            ..Default::default()
        }])
    }

    pub fn command_finger(&self, position: f64) -> Result<()> {
        self.finger_command.publish(Float64MultiArray {
            data: vec![position],
            ..Default::default()
        })?;
        Ok(())
    }

    fn publish_trajectory(&self, points: Vec<JointTrajectoryPoint>) -> Result<()> {
        self.trajectory.publish(JointTrajectory {
            joint_names: ARM_JOINTS.map(str::to_owned).to_vec(),
            points,
            ..Default::default()
        })?;
        Ok(())
    }
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

/// The average velocity over each step, each from the position before it.
fn step_velocities(from: &Joints, positions: &[Joints], segment: Duration) -> Vec<Joints> {
    let seconds = segment.as_secs_f64();
    positions
        .iter()
        .scan(*from, |previous, position| {
            let velocity =
                std::array::from_fn(|joint| (position[joint] - previous[joint]) / seconds);
            *previous = *position;
            Some(velocity)
        })
        .collect()
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

    #[test_case(0)]
    #[test_case(1)]
    #[test_case(2)]
    fn camera_freshness_is_independent(index: usize) {
        let cameras = Cameras::default();
        assert_eq!(cameras.live(), [false; 3]);
        for frame in &cameras.frames {
            *frame.lock() = Some(Received {
                value: vec![1, 2, 3],
                at: Instant::now() - STALE_AFTER - Duration::from_secs(1),
            });
        }
        store(&cameras.frames[index], vec![4, 5, 6]);
        let mut expected = [false; 3];
        expected[index] = true;
        assert_eq!(cameras.live(), expected);
        cameras.frames[index].lock().as_mut().unwrap().at -= Duration::from_secs(1);
        assert_eq!(cameras.live(), [false; 3]);
    }

    #[test_case(0)]
    #[test_case(1)]
    #[test_case(2)]
    fn selected_camera_reuses_monitored_frame(index: usize) {
        let cameras = Cameras::default();
        let selected = cameras.frame(CAMERA_TOPICS[index]).unwrap();
        assert!(Arc::ptr_eq(&selected, &cameras.frames[index]));
        store(&cameras.frames[index], vec![1, 2, 3]);
        assert_eq!(latest(&selected).unwrap().value, vec![1, 2, 3]);
        assert_matches!(cameras.frame("/custom/image/compressed"), None);
    }

    #[test_case(&[[0.25; 6]] => vec![[0.5; 6]]; "one step")]
    #[test_case(&[[0.25; 6], [0.75; 6]] => vec![[0.5; 6], [1.0; 6]]; "two steps")]
    #[test_case(&[[0.0, 0.5, -0.5, 0.0, 0.0, 0.0]] => vec![[0.0, 1.0, -1.0, 0.0, 0.0, 0.0]]; "per joint")]
    fn velocities_per_step(positions: &[Joints]) -> Vec<Joints> {
        step_velocities(&[0.0; 6], positions, Duration::from_millis(500))
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
