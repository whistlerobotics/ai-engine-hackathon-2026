use crate::Joints;
use nalgebra::{Isometry3, Matrix3, Rotation3, Translation3, UnitQuaternion, Vector3};
use std::f64::consts::{FRAC_PI_2, PI};

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

const PALM_OFFSET_IN_TOOL0: Vector3<f64> = Vector3::new(0.0, 0.107, 0.0);

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PalmPose {
    pub position: [f64; 3],
    /// `w, x, y, z`, with its largest-magnitude component positive.
    pub orientation: [f64; 4],
}

/// The palm's pose in `base_link`, which is the DH base (`base`) turned 180° about z.
#[must_use]
pub fn palm_pose(joints: &Joints) -> PalmPose {
    let base_link_to_base = Isometry3::from_parts(
        Translation3::identity(),
        UnitQuaternion::from_axis_angle(&Vector3::z_axis(), PI),
    );
    let tool0 = UR12E
        .iter()
        .zip(joints)
        .fold(base_link_to_base, |pose, (link, &theta)| {
            pose * dh_transform(link, theta)
        });
    let palm = tool0 * tool0_to_palm();

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
    PalmPose {
        position: [position.x, position.y, position.z],
        orientation,
    }
}

fn dh_transform(link: &DhLink, theta: f64) -> Isometry3<f64> {
    let translation = Translation3::new(link.a * theta.cos(), link.a * theta.sin(), link.d);
    let rotation = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), theta)
        * UnitQuaternion::from_axis_angle(&Vector3::x_axis(), link.alpha);
    Isometry3::from_parts(translation, rotation)
}

/// The palm's x, y and z axes are `tool0`'s -x, z and y.
fn tool0_to_palm() -> Isometry3<f64> {
    #[rustfmt::skip]
    let axes = Matrix3::new(
        -1.0, 0.0, 0.0,
         0.0, 0.0, 1.0,
         0.0, 1.0, 0.0,
    );
    Isometry3::from_parts(
        PALM_OFFSET_IN_TOOL0.into(),
        UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(axes)),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    #[expect(
        clippy::approx_constant,
        reason = "the contract's example pose, as written"
    )]
    fn contract_example() {
        let pose = palm_pose(&[-0.2055, -0.60, 1.15, -3.60, 1.5708, 0.0]);

        let expected_position = [0.8951, -0.0087, 0.2514];
        let expected_orientation = [-0.0289, -0.6302, 0.7751, 0.0355];
        let close = |actual: &[f64], expected: &[f64]| {
            actual
                .iter()
                .zip(expected)
                .all(|(a, e)| (a - e).abs() < 0.001)
        };
        assert!(close(&pose.position, &expected_position), "{pose:?}");
        assert!(close(&pose.orientation, &expected_orientation), "{pose:?}");
    }
}
