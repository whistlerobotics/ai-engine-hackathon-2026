use std::time::{Duration, Instant};

const DEBOUNCE: Duration = Duration::from_millis(20);

/// Takes on a new state only once the input has held it for `DEBOUNCE`.
#[derive(Debug, Default)]
pub struct Debouncer {
    stable: Option<bool>,
    candidate: Option<(bool, Instant)>,
}

impl Debouncer {
    /// The debounced state, with `pressed` read at `now`.
    pub fn update(&mut self, pressed: bool, now: Instant) -> bool {
        match (self.stable, self.candidate) {
            (None, _) => self.stable = Some(pressed),
            (Some(stable), _) if stable == pressed => self.candidate = None,
            (Some(_), Some((candidate, since))) if candidate == pressed => {
                if now - since >= DEBOUNCE {
                    self.stable = Some(pressed);
                    self.candidate = None;
                }
            }
            (Some(_), _) => self.candidate = Some((pressed, now)),
        }
        self.stable.unwrap_or(pressed)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use test_case::test_case;

    /// The debounced states after reading each `(pressed, milliseconds)` in turn.
    fn debounce(readings: &[(bool, u64)]) -> Vec<bool> {
        let start = Instant::now();
        let mut debouncer = Debouncer::default();
        readings
            .iter()
            .map(|&(pressed, ms)| debouncer.update(pressed, start + Duration::from_millis(ms)))
            .collect()
    }

    #[test_case(&[(true, 0)] => vec![true]; "first reading counts at once")]
    #[test_case(&[(false, 0), (true, 5), (true, 20), (true, 25)] => vec![false, false, false, true];
        "a press counts once held")]
    #[test_case(&[(false, 0), (true, 5), (false, 10), (true, 15), (true, 30)]
        => vec![false, false, false, false, false]; "a bounce restarts the hold")]
    #[test_case(&[(true, 0), (false, 5), (false, 25), (true, 30)] => vec![true, true, false, false];
        "a release counts once held")]
    fn states(readings: &[(bool, u64)]) -> Vec<bool> {
        debounce(readings)
    }
}
