//! The contract gives the gripper as its opening, `0.0` closed to `1.0` open; the 2F-140's
//! `finger_joint` turns from `0.0` open to `FINGER_CLOSED`.

pub const FINGER_JOINT: &str = "finger_joint";
const FINGER_CLOSED: f64 = 0.695;

#[must_use]
pub fn opening(finger: f64) -> f64 {
    (1.0 - finger / FINGER_CLOSED).clamp(0.0, 1.0)
}

#[must_use]
pub fn finger(opening: f64) -> f64 {
    (1.0 - opening) * FINGER_CLOSED
}

#[cfg(test)]
mod tests {
    use super::*;
    use test_case::test_case;

    #[test_case(0.0 => 1.0; "open")]
    #[test_case(FINGER_CLOSED => 0.0; "closed")]
    #[test_case(-0.01 => 1.0; "past open")]
    #[test_case(0.8 => 0.0; "past closed")]
    fn opening_of(finger: f64) -> f64 {
        opening(finger)
    }

    #[test]
    fn round_trip() {
        assert!((opening(finger(0.25)) - 0.25).abs() < 1e-12);
    }
}
