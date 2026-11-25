use roxmltree::{Document, Node};

const ROBOTIQ_PLUGIN: &str = "robotiq_driver/RobotiqGripperHardwareInterface";
/// `robotiq_driver` multiplies `gripper_force_multiplier` by this, not by `gripper_max_force`,
/// which only caps the result.
const DRIVER_FORCE: f64 = 235.0;

/// The gripper's `ros2_control` hardware, as its robot description configures it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GripperHardware {
    /// `force` is in newtons.
    Robotiq { speed_multiplier: f64, force: f64 },
    /// Any other, such as mock hardware, which has no speed or force.
    Other,
}

#[derive(Debug, PartialEq, Eq, thiserror::Error)]
pub enum DescriptionError {
    #[error("the gripper's robot description is not XML: {0}")]
    NotXml(#[from] roxmltree::Error),
    #[error("the gripper's robot description has no ros2_control hardware")]
    NoHardware,
    #[error("the gripper's robot description has no {0}")]
    MissingParam(String),
    #[error("a parameter in the gripper's robot description is not a number: {0:?}")]
    NotANumber(String),
}

impl GripperHardware {
    /// Works out what `robotiq_driver` sends from the parameters it reads.
    pub fn from_description(urdf: &str) -> Result<Self, DescriptionError> {
        let document = Document::parse(urdf)?;
        let hardware = document
            .descendants()
            .find(|node| node.has_tag_name("hardware"))
            .ok_or(DescriptionError::NoHardware)?;
        let plugin = hardware
            .children()
            .find(|node| node.has_tag_name("plugin"))
            .and_then(|node| node.text())
            .map(str::trim);
        if plugin != Some(ROBOTIQ_PLUGIN) {
            return Ok(Self::Other);
        }

        let force_multiplier = param(hardware, "gripper_force_multiplier")?;
        let max_force = param(hardware, "gripper_max_force")?;
        Ok(Self::Robotiq {
            speed_multiplier: param(hardware, "gripper_speed_multiplier")?,
            force: (DRIVER_FORCE * force_multiplier).abs().min(max_force),
        })
    }
}

fn param(hardware: Node<'_, '_>, name: &str) -> Result<f64, DescriptionError> {
    let node = hardware
        .children()
        .find(|node| node.has_tag_name("param") && node.attribute("name") == Some(name))
        .ok_or_else(|| DescriptionError::MissingParam(name.to_owned()))?;
    let value = node.text().unwrap_or_default().trim();
    value
        .parse()
        .map_err(|_| DescriptionError::NotANumber(value.to_owned()))
}

#[cfg(test)]
mod tests {
    use super::*;
    use test_case::test_case;

    fn description(plugin: &str, params: &str) -> String {
        format!(
            r#"<robot name="robotiq_gripper">
                 <ros2_control name="RobotiqGripperHardwareInterface" type="system">
                   <hardware>
                     <plugin>{plugin}</plugin>
                     {params}
                   </hardware>
                 </ros2_control>
               </robot>"#
        )
    }

    fn params(speed_multiplier: &str, force_multiplier: &str) -> String {
        format!(
            r#"<param name="gripper_speed_multiplier">{speed_multiplier}</param>
               <param name="gripper_force_multiplier">{force_multiplier}</param>
               <param name="gripper_max_force">125.0</param>"#
        )
    }

    #[test_case(&params("1.0", "0.5")
        => Ok(GripperHardware::Robotiq { speed_multiplier: 1.0, force: 117.5 }); "as configured")]
    #[test_case(&params("1.0", "1.0")
        => Ok(GripperHardware::Robotiq { speed_multiplier: 1.0, force: 125.0 }); "force capped")]
    #[test_case(""
        => Err(DescriptionError::MissingParam("gripper_force_multiplier".to_owned())); "missing")]
    #[test_case(&params("half", "0.5")
        => Err(DescriptionError::NotANumber("half".to_owned())); "not a number")]
    fn robotiq(params: &str) -> Result<GripperHardware, DescriptionError> {
        GripperHardware::from_description(&description(ROBOTIQ_PLUGIN, params))
    }

    #[test]
    fn mock() {
        let description = description("mock_components/GenericSystem", "");
        assert_eq!(
            GripperHardware::from_description(&description),
            Ok(GripperHardware::Other)
        );
    }
}
