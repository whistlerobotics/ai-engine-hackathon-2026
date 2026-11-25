use std::{
    ops::RangeInclusive,
    time::{Duration, Instant},
};

/// `ur_msgs/Analog`'s `VOLTAGE` domain.
const VOLTAGE: u8 = 1;
// throttle_cal measured 0.84 V released and 4.35 V pressed fully; these ends sit just inside, so
// noise and a resting thumb still read as released, and every full press as fully pressed.
const RELEASED_V: f64 = 0.90;
const PRESSED_V: f64 = 4.32;
const VALID_V: RangeInclusive<f64> = 0.75..=4.50;
// The gripper's driver spreads its travel over 227 steps of its position register, and held still
// mid-travel, the throttle's raw reading spanned up to 1.5 of them, peak to peak.
const HYSTERESIS: f64 = 2.0 / 227.0;
const STALE_AFTER: Duration = Duration::from_millis(100);

#[derive(Debug, Clone, Copy)]
pub struct Reading {
    pub domain: u8,
    pub volts: f64,
    pub at: Instant,
}

#[derive(Debug, Clone, Copy, PartialEq, thiserror::Error)]
pub enum Fault {
    #[error("no fresh reading")]
    Stale,
    #[error("not in voltage mode")]
    NotVoltage,
    #[error("reads {0:.2} V, outside {start}-{end} V", start = VALID_V.start(), end = VALID_V.end())]
    OutOfRange(f64),
}

/// The throttle's voltage, if it can be trusted.
pub fn volts(reading: Option<Reading>, now: Instant) -> Result<f64, Fault> {
    let reading = reading
        .filter(|reading| now - reading.at <= STALE_AFTER)
        .ok_or(Fault::Stale)?;
    if reading.domain != VOLTAGE {
        return Err(Fault::NotVoltage);
    }
    if !VALID_V.contains(&reading.volts) {
        return Err(Fault::OutOfRange(reading.volts));
    }
    Ok(reading.volts)
}

/// The gripper opening the throttle asks for: released closes it, pressed fully opens it.
pub fn opening(volts: f64) -> f64 {
    ((volts - RELEASED_V) / (PRESSED_V - RELEASED_V)).clamp(0.0, 1.0)
}

/// Holds the opening until the throttle asks for `HYSTERESIS` more or less, or for either end.
#[derive(Debug, Default)]
pub struct Hysteresis {
    held: Option<f64>,
}

impl Hysteresis {
    pub fn update(&mut self, target: f64) -> f64 {
        let at_end = target <= 0.0 || target >= 1.0;
        match self.held.map(|held| (held, (target - held).abs())) {
            Some((held, moved)) if moved < HYSTERESIS && !(at_end && moved > 0.0) => held,
            _ => *self.held.insert(target),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use test_case::test_case;

    #[test_case(0.5 => 0.0; "below released")]
    #[test_case(RELEASED_V => 0.0; "released")]
    #[test_case(PRESSED_V => 1.0; "pressed")]
    #[test_case(4.4 => 1.0; "past pressed")]
    fn opening_at(volts: f64) -> f64 {
        opening(volts)
    }

    #[test]
    fn half_way_is_half_open() {
        assert!((opening(f64::midpoint(RELEASED_V, PRESSED_V)) - 0.5).abs() < 1e-12);
    }

    #[test_case(None, 0 => Err(Fault::Stale); "no reading")]
    #[test_case(Some((VOLTAGE, 2.0)), 150 => Err(Fault::Stale); "stale")]
    #[test_case(Some((0, 2.0)), 0 => Err(Fault::NotVoltage); "current mode")]
    #[test_case(Some((VOLTAGE, 0.5)), 0 => Err(Fault::OutOfRange(0.5)); "below range")]
    #[test_case(Some((VOLTAGE, 4.6)), 0 => Err(Fault::OutOfRange(4.6)); "above range")]
    #[test_case(Some((VOLTAGE, 2.0)), 0 => Ok(2.0); "valid")]
    fn checked(reading: Option<(u8, f64)>, age_ms: u64) -> Result<f64, Fault> {
        let at = Instant::now();
        let reading = reading.map(|(domain, volts)| Reading { domain, volts, at });
        volts(reading, at + Duration::from_millis(age_ms))
    }

    #[test_case(&[0.5] => vec![0.5]; "first target")]
    #[test_case(&[0.5, 0.505, 0.495] => vec![0.5, 0.5, 0.5]; "jitter held")]
    #[test_case(&[0.5, 0.51] => vec![0.5, 0.51]; "a real move")]
    #[test_case(&[0.005, 0.0] => vec![0.005, 0.0]; "closed end always reached")]
    #[test_case(&[0.995, 1.0] => vec![0.995, 1.0]; "open end always reached")]
    fn held(targets: &[f64]) -> Vec<f64> {
        let mut hysteresis = Hysteresis::default();
        targets.iter().map(|&t| hysteresis.update(t)).collect()
    }
}
