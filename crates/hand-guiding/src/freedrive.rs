use color_eyre::{
    Result,
    eyre::{bail, eyre},
};
use rclrs::{Client, Node, Publisher};
use ros_env::{
    builtin_interfaces::msg::Duration as RosDuration,
    controller_manager_msgs::srv::{
        ListControllers, ListControllers_Request, SwitchController, SwitchController_Request,
    },
    std_msgs::msg::Bool,
};
use std::{sync::mpsc, time::Duration};

const FREEDRIVE_CONTROLLER: &str = "freedrive_mode_controller";
const MOTION_CONTROLLER: &str = "joint_trajectory_controller";
const ENABLE_TOPIC: &str = "/freedrive_mode_controller/enable_freedrive_mode";
const REPLY_TIMEOUT: Duration = Duration::from_secs(2);

/// The arm in freedrive by turns with `joint_trajectory_controller`, so that between presses
/// whatever commands the arm, such as the policy runner, still can.
pub struct Freedrive {
    switch: Client<SwitchController>,
    list: Client<ListControllers>,
    enable: Publisher<Bool>,
}

impl Freedrive {
    pub fn new(node: &Node) -> Result<Self> {
        Ok(Self {
            switch: node.create_client("/controller_manager/switch_controller")?,
            list: node.create_client("/controller_manager/list_controllers")?,
            enable: node.create_publisher(ENABLE_TOPIC)?,
        })
    }

    pub fn controller_manager_ready(&self) -> bool {
        self.switch.service_is_ready().unwrap_or(false)
            && self.list.service_is_ready().unwrap_or(false)
    }

    pub fn start(&self) -> Result<()> {
        self.switch_to(FREEDRIVE_CONTROLLER, MOTION_CONTROLLER)?;
        self.keep_alive()
    }

    pub fn keep_alive(&self) -> Result<()> {
        self.enable.publish(Bool { data: true })?;
        Ok(())
    }

    pub fn stop(&self) -> Result<()> {
        self.enable.publish(Bool { data: false })?;
        self.switch_to(MOTION_CONTROLLER, FREEDRIVE_CONTROLLER)
    }

    /// Hands the arm back to `joint_trajectory_controller` if a previous run left
    /// `freedrive_mode_controller` holding it; returns whether it had to.
    pub fn recover(&self) -> Result<bool> {
        let (sender, receiver) = mpsc::channel();
        drop(
            self.list
                .call_then(ListControllers_Request::default(), move |response| {
                    let _ = sender.send(response);
                })?,
        );
        let response = receiver
            .recv_timeout(REPLY_TIMEOUT)
            .map_err(|_| eyre!("the controller manager did not list its controllers"))?;
        let freedrive_active = response
            .controller
            .iter()
            .any(|c| c.name == FREEDRIVE_CONTROLLER && c.state == "active");
        if freedrive_active {
            self.stop()?;
        }
        Ok(freedrive_active)
    }

    fn switch_to(&self, activate: &str, deactivate: &str) -> Result<()> {
        let request = SwitchController_Request {
            activate_controllers: vec![activate.to_owned()],
            deactivate_controllers: vec![deactivate.to_owned()],
            strictness: SwitchController_Request::STRICT,
            activate_asap: true,
            timeout: RosDuration { sec: 1, nanosec: 0 },
        };
        let (sender, receiver) = mpsc::channel();
        drop(self.switch.call_then(request, move |response| {
            let _ = sender.send(response);
        })?);
        let response = receiver
            .recv_timeout(REPLY_TIMEOUT)
            .map_err(|_| eyre!("the controller manager did not answer switching to {activate}"))?;
        if !response.ok {
            bail!("switching to {activate} failed: {}", response.message);
        }
        Ok(())
    }
}
