use color_eyre::Result;
use parking_lot::Mutex;
use policy_contract::{ARM_JOINTS, Joints, gripper::FINGER_JOINT};
use rclrs::{Context, CreateBasicExecutor, IntoPrimitiveOptions, Node, SpinOptions, Subscription};
use ros_env::{
    sensor_msgs::msg::{CompressedImage, JointState},
    std_msgs::msg::{Bool, Float64MultiArray},
};
use std::{
    sync::Arc,
    thread::{self, JoinHandle},
    time::{Duration, Instant},
};

pub const CAMERA_TOPIC: &str = "/camera/color/image_raw/compressed";
pub const ACTIVE_TOPIC: &str = "/hand_guiding/active";
const FINGER_COMMAND_TOPIC: &str = "/gripper/finger_position_controller/commands";
const STALE_AFTER: Duration = Duration::from_millis(100);
/// hand-guiding sends its state every 20 ms.
const ACTIVE_STALE_AFTER: Duration = Duration::from_millis(200);

#[derive(Debug, Clone)]
pub struct Received<T> {
    pub value: T,
    pub at: Instant,
}

impl<T> Received<T> {
    fn new(value: T) -> Self {
        Self {
            value,
            at: Instant::now(),
        }
    }

    pub fn is_fresh(&self, now: Instant) -> bool {
        now - self.at <= STALE_AFTER
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HandGuiding {
    Idle,
    /// The arm is in freedrive, its button held.
    Guiding,
}

#[derive(Debug, Clone, Copy)]
pub struct ArmState {
    pub position: Joints,
    pub velocity: Joints,
}

type Latest<T> = Arc<Mutex<Option<Received<T>>>>;

/// Only listens: recording never commands the robot.
pub struct Ros {
    arm: Latest<ArmState>,
    finger: Latest<f64>,
    camera: Latest<Vec<u8>>,
    hand_guiding: Latest<HandGuiding>,
    finger_command: Latest<f64>,
    _subscriptions: Subscriptions,
    _node: Node,
    _executor_thread: JoinHandle<()>,
}

struct Subscriptions {
    _arm: Subscription<JointState>,
    _finger: Subscription<JointState>,
    _camera: Subscription<CompressedImage>,
    _hand_guiding: Subscription<Bool>,
    _finger_command: Subscription<Float64MultiArray>,
}

impl Ros {
    pub fn new() -> Result<Self> {
        let context = Context::default_from_env()?;
        let mut executor = context.create_basic_executor();
        let node = executor.create_node("record")?;

        let arm = Latest::default();
        let arm_subscription = node.create_subscription("/joint_states", {
            let arm = Arc::clone(&arm);
            move |msg: JointState| {
                let position = by_name(&msg.name, &msg.position, ARM_JOINTS);
                let velocity = by_name(&msg.name, &msg.velocity, ARM_JOINTS);
                if let (Some(position), Some(velocity)) = (position, velocity) {
                    *arm.lock() = Some(Received::new(ArmState { position, velocity }));
                }
            }
        })?;

        let finger = Latest::default();
        let finger_subscription = node.create_subscription("/gripper/joint_states", {
            let finger = Arc::clone(&finger);
            move |msg: JointState| {
                if let Some([position]) = by_name(&msg.name, &msg.position, [FINGER_JOINT]) {
                    *finger.lock() = Some(Received::new(position));
                }
            }
        })?;

        let camera = Latest::default();
        // The camera driver publishes best-effort, which a reliable subscription wouldn't match.
        let camera_subscription =
            node.create_subscription(CAMERA_TOPIC.best_effort().keep_last(1), {
                let camera = Arc::clone(&camera);
                move |msg: CompressedImage| *camera.lock() = Some(Received::new(msg.data))
            })?;

        let hand_guiding = Latest::default();
        let hand_guiding_subscription = node.create_subscription(ACTIVE_TOPIC, {
            let hand_guiding = Arc::clone(&hand_guiding);
            move |msg: Bool| {
                let state = if msg.data {
                    HandGuiding::Guiding
                } else {
                    HandGuiding::Idle
                };
                *hand_guiding.lock() = Some(Received::new(state));
            }
        })?;

        let finger_command = Latest::default();
        let finger_command_subscription = node.create_subscription(FINGER_COMMAND_TOPIC, {
            let finger_command = Arc::clone(&finger_command);
            move |msg: Float64MultiArray| {
                if let Some(&position) = msg.data.first().filter(|p| p.is_finite()) {
                    *finger_command.lock() = Some(Received::new(position));
                }
            }
        })?;

        let executor_thread = thread::spawn(move || {
            for error in executor.spin(SpinOptions::default()) {
                eprintln!("ROS executor: {error}");
            }
        });

        Ok(Self {
            arm,
            finger,
            camera,
            hand_guiding,
            finger_command,
            _subscriptions: Subscriptions {
                _arm: arm_subscription,
                _finger: finger_subscription,
                _camera: camera_subscription,
                _hand_guiding: hand_guiding_subscription,
                _finger_command: finger_command_subscription,
            },
            _node: node,
            _executor_thread: executor_thread,
        })
    }

    pub fn arm(&self) -> Option<Received<ArmState>> {
        self.arm.lock().clone()
    }

    /// The finger joint's angle.
    pub fn finger(&self) -> Option<Received<f64>> {
        self.finger.lock().clone()
    }

    pub fn camera(&self) -> Option<Received<Vec<u8>>> {
        self.camera.lock().clone()
    }

    /// None once hand-guiding has gone quiet: it is not running, or has hung.
    pub fn hand_guiding(&self, now: Instant) -> Option<HandGuiding> {
        self.hand_guiding
            .lock()
            .as_ref()
            .filter(|state| now - state.at <= ACTIVE_STALE_AFTER)
            .map(|state| state.value)
    }

    /// The finger joint angle last commanded, by whoever commanded it.
    pub fn finger_command(&self) -> Option<Received<f64>> {
        self.finger_command.lock().clone()
    }
}

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

#[cfg(test)]
mod tests {
    use super::*;
    use test_case::test_case;

    #[test_case(&[1.0, 2.0] => Some([2.0]); "found")]
    #[test_case(&[1.0] => None; "missing value")]
    #[test_case(&[1.0, f64::NAN] => None; "not finite")]
    fn finger_by_name(values: &[f64]) -> Option<[f64; 1]> {
        let names = ["other_joint", FINGER_JOINT].map(str::to_owned);
        by_name(&names, values, [FINGER_JOINT])
    }
}
