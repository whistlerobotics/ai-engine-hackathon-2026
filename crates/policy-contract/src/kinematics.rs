use crate::{Joints, STEP};
use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};
use std::f64::consts::{FRAC_PI_2, FRAC_PI_8, PI};

struct DhLink {
    d: f64,
    a: f64,
    alpha: f64,
}

/// The `UR12e`'s published, not calibrated, DH parameters, on the assumption that the policy's
/// training simulation used them too. They must agree with `ur_description`'s
/// `config/ur12e/default_kinematics.yaml`, which gives them as URDF joint origins; they are not
/// read from there so that passing the robot's calibration to its bringup cannot change them.
const UR12E: [DhLink; 6] = [
    DhLink {
        d: 0.1807,
        a: 0.0,
        alpha: FRAC_PI_2,
    },
    DhLink {
        d: 0.0,
        a: -0.6127,
        alpha: 0.0,
    },
    DhLink {
        d: 0.0,
        a: -0.57155,
        alpha: 0.0,
    },
    DhLink {
        d: 0.17415,
        a: 0.0,
        alpha: FRAC_PI_2,
    },
    DhLink {
        d: 0.11985,
        a: 0.0,
        alpha: -FRAC_PI_2,
    },
    DhLink {
        d: 0.11655,
        a: 0.0,
        alpha: 0.0,
    },
];

/// The wrist bracket in `robot/gripper.urdf.xacro`, which also carries the camera. Its face is
/// turned 22.5° from the flange's so that both can be bolted on, and the gripper turns with it.
const BRACKET_THICKNESS: f64 = 0.0244;
const BRACKET_TWIST: f64 = -FRAC_PI_8;
const V3_PALM_IN_TOOL0_Z: f64 = 0.1314;

/// Where the training simulation, whose gripper is bolted straight onto the flange, has the palm.
const PALM_OFFSET_IN_GRIPPER_MOUNT: Vector3<f64> = Vector3::new(0.0, 0.107, 0.0);

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PalmPose {
    pub position: [f64; 3],
    /// `w, x, y, z`, with its largest-magnitude component positive.
    pub orientation: [f64; 4],
}

/// The palm's pose in `base_link`, which is the DH base (`base`) turned 180° about z.
#[must_use]
pub fn palm_pose(joints: &Joints) -> PalmPose {
    // Contract v3 uses the simulator's nominal gripperframe (+Z approach, +Y opening).
    // The physical mount and measured kinematics require independent real-cell calibration.
    let tool0_to_palm =
        Isometry3::new(Vector3::z() * V3_PALM_IN_TOOL0_Z, Vector3::z() * -FRAC_PI_2);
    PalmPose::new(&(tool0_pose(joints) * tool0_to_palm))
}

/// The legacy demonstration recorder's original physical-bracket convention.
#[must_use]
pub fn recorder_palm_pose(joints: &Joints) -> PalmPose {
    PalmPose::new(&(tool0_pose(joints) * tool0_to_gripper_mount() * gripper_mount_to_palm()))
}

impl PalmPose {
    fn new(palm: &Isometry3<f64>) -> Self {
        let position = palm.translation.vector;
        let rotation = palm.rotation;
        let mut orientation = [rotation.w, rotation.i, rotation.j, rotation.k];
        // On a tie the first component, in `w, x, y, z` order, decides the sign; `max_by` returns the
        // last of equals, hence the reversal.
        let largest = orientation
            .iter()
            .rev()
            .max_by(|a, b| a.abs().total_cmp(&b.abs()));
        if largest.is_some_and(|&c| c < 0.0) {
            orientation = orientation.map(|c| -c);
        }
        Self {
            position: [position.x, position.y, position.z],
            orientation,
        }
    }
}

/// Commanded palm and elbow speeds in m/s, using the position Jacobians at the clipped target.
#[must_use]
pub fn palm_elbow_speeds(target: &Joints, delta: &Joints) -> (f64, f64) {
    let mut frame = dh_base();
    let mut angular = Vector3::zeros();
    let mut linear = Vector3::zeros();
    let mut elbow = Vector3::zeros();
    for (joint, link) in UR12E.iter().enumerate() {
        angular += frame.rotation * Vector3::z() * delta[joint];
        let next = frame * dh_transform(link, target[joint]);
        linear += angular.cross(&(next.translation.vector - frame.translation.vector));
        frame = next;
        if joint == 1 {
            elbow = linear; // DH frame 2 is the simulator's forearm_link origin.
        }
    }
    let palm = linear + angular.cross(&(frame.rotation * Vector3::z() * V3_PALM_IN_TOOL0_Z));
    let dt = STEP.as_secs_f64();
    (palm.norm() / dt, elbow.norm() / dt)
}

fn dh_base() -> Isometry3<f64> {
    Isometry3::from_parts(
        Translation3::identity(),
        UnitQuaternion::from_axis_angle(&Vector3::z_axis(), PI),
    )
}

fn tool0_pose(joints: &Joints) -> Isometry3<f64> {
    UR12E
        .iter()
        .zip(joints)
        .fold(dh_base(), |pose, (link, &theta)| {
            pose * dh_transform(link, theta)
        })
}

fn dh_transform(link: &DhLink, theta: f64) -> Isometry3<f64> {
    let translation = Translation3::new(link.a * theta.cos(), link.a * theta.sin(), link.d);
    let rotation = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), theta)
        * UnitQuaternion::from_axis_angle(&Vector3::x_axis(), link.alpha);
    Isometry3::from_parts(translation, rotation)
}

fn tool0_to_gripper_mount() -> Isometry3<f64> {
    Isometry3::new(
        Vector3::z() * BRACKET_THICKNESS,
        Vector3::z() * BRACKET_TWIST,
    )
}

fn gripper_mount_to_palm() -> Isometry3<f64> {
    let palm_axes_in_gripper_mount = [-Vector3::x(), Vector3::z(), Vector3::y()];
    Isometry3::from_parts(
        PALM_OFFSET_IN_GRIPPER_MOUNT.into(),
        UnitQuaternion::from_basis_unchecked(&palm_axes_in_gripper_mount),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ARM_JOINTS;
    use approx::assert_abs_diff_eq;
    use nalgebra::{Rotation3, Unit, Vector4};
    use roxmltree::{Document, Node};
    use std::{process::Command, sync::LazyLock};
    use test_case::test_case;

    static ARM_URDF: LazyLock<Document<'static>> = LazyLock::new(|| {
        Document::parse(include_str!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../robot/urdf/ur12e.urdf"
        )))
        .unwrap()
    });

    #[test_case([0.0;6], [1.18425,0.4221,0.06085],[0.5,-0.5,0.5,0.5])]
    #[test_case([0.2,-1.1,1.3,-0.9,-1.5,0.4], [0.673560537,0.332125508,0.362193317],[0.093908217,0.900117085,-0.105314716,-0.412164154])]
    fn simulator_palm_fixture(joints: Joints, xyz: [f64; 3], wxyz: [f64; 4]) {
        let pose = palm_pose(&joints);
        assert_abs_diff_eq!(
            Vector3::from(pose.position),
            Vector3::from(xyz),
            epsilon = 1e-6
        );
        let actual = Vector4::from(pose.orientation);
        let reference = Vector4::from(wxyz);
        assert_abs_diff_eq!(actual.dot(&reference).abs(), 1.0, epsilon = 1e-6);
    }

    /// What launch.xml gives `robot_state_publisher`, which needs `robotiq_description` sourced.
    static GRIPPER_URDF: LazyLock<Document<'static>> = LazyLock::new(|| {
        let xacro = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../robot/gripper.urdf.xacro"
        );
        let output = Command::new("xacro").arg(xacro).output().unwrap();
        assert!(
            output.status.success(),
            "{}",
            String::from_utf8_lossy(&output.stderr)
        );
        Document::parse(String::from_utf8(output.stdout).unwrap().leak()).unwrap()
    });

    fn gripper_pose(link: &str) -> Isometry3<f64> {
        urdf_pose(&GRIPPER_URDF, "tool0", link, &[0.0; 6]).unwrap()
    }

    /// `link`'s pose in `root`, by composing the URDF's joints down to it.
    fn urdf_pose(
        urdf: &Document<'_>,
        root: &str,
        link: &str,
        joints: &Joints,
    ) -> Option<Isometry3<f64>> {
        if link == root {
            return Some(Isometry3::identity());
        }
        let joint = urdf.root_element().children().find(|n| {
            n.has_tag_name("joint")
                && child_element(*n, "child").and_then(|c| c.attribute("link")) == Some(link)
        })?;
        let parent = child_element(joint, "parent")?.attribute("link")?;
        let origin = child_element(joint, "origin")?;
        let rpy = vector(origin, "rpy")?;
        let joint_to_parent = Isometry3::from_parts(
            vector(origin, "xyz")?.into(),
            UnitQuaternion::from_euler_angles(rpy.x, rpy.y, rpy.z),
        );
        let motion = match joint.attribute("type")? {
            "fixed" => UnitQuaternion::identity(),
            "revolute" => {
                let name = joint.attribute("name")?;
                let index = ARM_JOINTS.iter().position(|&j| j == name)?;
                let axis = Unit::new_normalize(vector(child_element(joint, "axis")?, "xyz")?);
                UnitQuaternion::from_axis_angle(&axis, joints[index])
            }
            other => panic!("joint type {other}"),
        };
        Some(urdf_pose(urdf, root, parent, joints)? * joint_to_parent * motion)
    }

    fn child_element<'a, 'input>(node: Node<'a, 'input>, tag: &str) -> Option<Node<'a, 'input>> {
        node.children().find(|n| n.has_tag_name(tag))
    }

    fn vector(node: Node<'_, '_>, attribute: &str) -> Option<Vector3<f64>> {
        let values: Vec<f64> = node
            .attribute(attribute)?
            .split_whitespace()
            .map(|v| v.parse().ok())
            .collect::<Option<_>>()?;
        Some(Vector3::from_column_slice(&values))
    }

    #[test_case([0.0; 6])]
    #[test_case([-0.2, -0.6, 1.15, -3.6, 1.6, 0.0])]
    #[test_case([1.0, -1.2, 2.0, -0.5, -1.0, 2.5])]
    #[test_case([-2.8, 0.4, -2.6, 3.1, 0.3, -4.0])]
    fn dh_matches_urdf(joints: Joints) {
        let dh = tool0_pose(&joints);
        let urdf = urdf_pose(&ARM_URDF, "base_link", "tool0", &joints).unwrap();

        assert_abs_diff_eq!(dh.to_homogeneous(), urdf.to_homogeneous(), epsilon = 1e-6);
    }

    #[test_case(&ARM_URDF)]
    #[test_case(&GRIPPER_URDF)]
    fn bracket_matches_urdf(urdf: &Document<'_>) {
        let urdf = urdf_pose(urdf, "tool0", "gripper_mount", &[0.0; 6]).unwrap();

        assert_abs_diff_eq!(
            tool0_to_gripper_mount().to_homogeneous(),
            urdf.to_homogeneous(),
            epsilon = 1e-6
        );
    }

    /// The palm's x is the fingers' closing axis and its y the approach.
    #[test]
    fn palm_axes_match_gripper() {
        let gripper = gripper_pose("robotiq_140_base_link");

        let palm_in_gripper =
            gripper.inverse() * tool0_to_gripper_mount() * gripper_mount_to_palm();

        let palm_axes_in_gripper = [Vector3::y(), Vector3::z(), Vector3::x()];
        assert_abs_diff_eq!(
            palm_in_gripper.rotation.to_rotation_matrix(),
            Rotation3::from_basis_unchecked(&palm_axes_in_gripper),
            epsilon = 1e-6
        );
    }

    /// The example is the training simulation's, whose gripper is bolted straight onto the flange.
    #[test]
    #[expect(
        clippy::approx_constant,
        reason = "the contract's example pose, as written"
    )]
    fn contract_example() {
        let joints = [-0.2055, -0.60, 1.15, -3.60, 1.5708, 0.0];

        let pose = PalmPose::new(&(tool0_pose(&joints) * gripper_mount_to_palm()));

        assert_abs_diff_eq!(
            Vector3::from(pose.position),
            Vector3::new(0.8951, -0.0087, 0.2514),
            epsilon = 0.001
        );
        assert_abs_diff_eq!(
            Vector4::from(pose.orientation),
            Vector4::new(-0.0289, -0.6302, 0.7751, 0.0355),
            epsilon = 0.001
        );
    }
}
