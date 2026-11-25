use crate::{button::Debouncer, freedrive::Freedrive, throttle};
use color_eyre::Result;
use parking_lot::Mutex;
use rclrs::{
    Context, CreateBasicExecutor, IntoPrimitiveOptions, Node, Publisher, SpinOptions, Subscription,
};
use ros_env::{
    std_msgs::msg::{Bool, Float64MultiArray},
    ur_msgs::msg::IOStates,
};
use std::{
    sync::Arc,
    thread::{self, JoinHandle},
    time::Instant,
};

pub const IO_TOPIC: &str = "/io_and_status_controller/io_states";
pub const BUTTON_PIN: u8 = 0;
pub const THROTTLE_PIN: u8 = 1;
/// Latest freedrive state, sent every control cycle; `record` takes a silence as hand-guiding gone.
const ACTIVE_TOPIC: &str = "/hand_guiding/active";
const FINGER_COMMAND_TOPIC: &str = "/gripper/finger_position_controller/commands";

#[derive(Debug, Clone, Copy)]
pub struct Button {
    pub pressed: bool,
    pub at: Instant,
}

#[derive(Default)]
struct Io {
    debouncer: Debouncer,
    button: Option<Button>,
    throttle: Option<throttle::Reading>,
}

pub struct Ros {
    io: Arc<Mutex<Io>>,
    pub freedrive: Freedrive,
    finger_command: Publisher<Float64MultiArray>,
    active: Publisher<Bool>,
    _io_subscription: Subscription<IOStates>,
    _node: Node,
    _executor_thread: JoinHandle<()>,
}

impl Ros {
    pub fn new() -> Result<Self> {
        let context = Context::default_from_env()?;
        let mut executor = context.create_basic_executor();
        let node = executor.create_node("hand_guiding")?;

        let io: Arc<Mutex<Io>> = Arc::default();
        let io_subscription = node.create_subscription(IO_TOPIC.keep_last(100), {
            let io = Arc::clone(&io);
            move |msg: IOStates| {
                let at = Instant::now();
                let mut io = io.lock();
                if let Some(analog) = msg.analog_in_states.iter().find(|a| a.pin == THROTTLE_PIN) {
                    io.throttle = Some(throttle::Reading {
                        domain: analog.domain,
                        volts: f64::from(analog.state),
                        at,
                    });
                }
                if let Some(digital) = msg.digital_in_states.iter().find(|d| d.pin == BUTTON_PIN) {
                    let pressed = io.debouncer.update(digital.state, at);
                    io.button = Some(Button { pressed, at });
                }
            }
        })?;

        let freedrive = Freedrive::new(&node)?;
        let finger_command = node.create_publisher(FINGER_COMMAND_TOPIC)?;
        let active = node.create_publisher(ACTIVE_TOPIC)?;

        let executor_thread = thread::spawn(move || {
            for error in executor.spin(SpinOptions::default()) {
                eprintln!("ROS executor: {error}");
            }
        });

        Ok(Self {
            io,
            freedrive,
            finger_command,
            active,
            _io_subscription: io_subscription,
            _node: node,
            _executor_thread: executor_thread,
        })
    }

    pub fn button(&self) -> Option<Button> {
        self.io.lock().button
    }

    pub fn throttle(&self) -> Option<throttle::Reading> {
        self.io.lock().throttle
    }

    pub fn command_finger(&self, position: f64) -> Result<()> {
        self.finger_command.publish(Float64MultiArray {
            data: vec![position],
            ..Default::default()
        })?;
        Ok(())
    }

    pub fn publish_active(&self, active: bool) -> Result<()> {
        self.active.publish(Bool { data: active })?;
        Ok(())
    }
}
