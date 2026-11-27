use crate::task::joint_limit;
use policy_contract::{Joints, kinematics::palm_elbow_speeds, wire::Action};

const MAX_LINEAR_SPEED: f64 = 1.5;

pub struct Motion {
    pub target: Joints,
    pub tcp_speed: f64,
    pub elbow_speed: f64,
    pub scale: f64,
}

/// The simulator runner clips each chunk against its projected joint targets before stepping the engine.
/// TCP/elbow scaling below then acts on the actual command carried from one step to the next.
pub fn clip_chunk(mut projected: Joints, actions: &mut [Action]) {
    for action in actions {
        for (j, value) in action.delta.iter_mut().enumerate() {
            let requested = projected[j] + *value;
            let target = requested.clamp(-joint_limit(j), joint_limit(j));
            if target != requested {
                *value = target - projected[j];
            }
            projected[j] = target;
        }
    }
}

/// Integrates an already normalized policy increment onto the previous command.
/// Reality Check evaluates J(target) · (target − command) after joint-position clipping.
pub fn integrate(command: Joints, delta: Joints) -> Motion {
    let target =
        std::array::from_fn(|j| (command[j] + delta[j]).clamp(-joint_limit(j), joint_limit(j)));
    let displacement = std::array::from_fn(|j| target[j] - command[j]);
    let (tcp_speed, elbow_speed) = palm_elbow_speeds(&target, &displacement);
    let peak = tcp_speed.max(elbow_speed);
    let scale = if peak > MAX_LINEAR_SPEED {
        MAX_LINEAR_SPEED / peak
    } else {
        1.0
    };
    let target = if scale < 1.0 {
        std::array::from_fn(|j| command[j] + scale * displacement[j])
    } else {
        target
    };
    Motion {
        target,
        tcp_speed,
        elbow_speed,
        scale,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn near(actual: f64, expected: f64) {
        assert!((actual - expected).abs() < 1e-8, "{actual} != {expected}");
    }

    #[test]
    fn successive_deltas_and_zero_hold_share_commanded_target() {
        let start = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5];
        let single = [0.011, -0.009, 0.01, 0.0, 0.0, 0.005];
        let first = integrate(start, single);
        assert_eq!(first.scale, 1.0);
        let held = integrate(first.target, [0.0; 6]);
        assert_eq!(held.target, first.target);
        assert_eq!(held.tcp_speed, 0.0);
        assert_eq!(held.elbow_speed, 0.0);
        let second = integrate(held.target, single);
        for j in 0..6 {
            near(second.target[j], start[j] + 2.0 * single[j]);
        }
    }

    #[test]
    fn joint_positions_are_bounded_before_speed_scaling() {
        let elbow = joint_limit(2);
        for sign in [-1.0, 1.0] {
            let initial = [0.0, 0.0, sign * (elbow - 0.01), 0.0, 0.0, 0.0];
            let result = integrate(initial, [0.0, 0.0, sign * 0.05, 0.0, 0.0, 0.0]);
            near(result.target[2], sign * elbow);
        }
        // Compiled MuJoCo UR12e, rc_engine.arm_limits.SpeedScaler at the clipped target.
        let result = integrate([6.275, 0.0, 0.0, 0.0, 0.0, 0.0], [0.05; 6]);
        near(result.tcp_speed, 1.8088625937730891);
        near(result.elbow_speed, 0.6208448375476096);
        near(result.target[0], 6.281791560642744);
        for joint in &result.target[1..] {
            near(*joint, 0.04146251918646746);
        }
    }

    #[test]
    fn tcp_limit_preserves_direction_at_simulator_home() {
        let start = [-0.2055, -1.1091, 1.6007, -1.9708, -1.5708, 1.5708];
        let delta = [-0.05, -0.05, -0.05, -0.05, -0.05, 0.05];
        let result = integrate(start, delta);
        near(result.tcp_speed, 2.2029211052971243);
        near(result.elbow_speed, 0.6599355320574997);
        near(result.scale * result.tcp_speed, 1.5);
        for j in 0..6 {
            near(result.target[j], start[j] + result.scale * delta[j]);
        }
    }

    #[test]
    fn scaled_steps_near_a_joint_limit_match_runner_across_chunk_boundary() {
        // Reference: rc_runner.motion.clamp_action, then the compiled simulator per step.
        let start = [6.275, 0.0, 0.0, 0.0, 0.0, 0.0];
        let action = Action {
            delta: [0.05; 6],
            gripper: 1.0,
        };
        let mut chunk = [action; 2];
        clip_chunk(start, &mut chunk);
        near(chunk[0].delta[0], 0.00819);
        near(chunk[1].delta[0], 0.0);
        let first = integrate(start, chunk[0].delta).target;
        let second = integrate(first, chunk[1].delta).target;
        near(second[0], 6.281791560642744);
        near(second[1], 0.08285727228550571);
        let mut next = [action];
        clip_chunk(second, &mut next);
        let third = integrate(second, next[0].delta).target;
        near(third[0], 6.2829334960836425);
        near(third[1], 0.12368619471869091);
    }

    #[test]
    fn elbow_limit_also_applies_when_palm_is_slow() {
        // A larger internal step exercises the independent elbow guard above the wire cap.
        let result = integrate(
            [0.0, 0.0, 2.0, 0.0, 0.0, 0.0],
            [0.105, -0.105, 0.0, 0.0, 0.0, 0.0],
        );
        near(result.tcp_speed, 0.932198574983417);
        near(result.elbow_speed, 1.8146223439573732);
        near(result.scale * result.elbow_speed, 1.5);
        near(result.target[0], 0.08679491935303746);
    }
}
