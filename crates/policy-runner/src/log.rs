use parking_lot::Mutex;
use std::{
    collections::VecDeque,
    time::{Duration, Instant},
};

const CAPACITY: usize = 500;

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Line {
    /// Since the runner started.
    pub at: Duration,
    pub text: String,
}

/// The operator's log, newest last, keeping the latest `CAPACITY` lines.
pub struct Log {
    started: Instant,
    lines: Mutex<VecDeque<Line>>,
}

impl Log {
    pub fn new() -> Self {
        Self {
            started: Instant::now(),
            lines: Mutex::new(VecDeque::with_capacity(CAPACITY)),
        }
    }

    pub fn push(&self, text: impl Into<String>) {
        let line = Line {
            at: self.started.elapsed(),
            text: text.into(),
        };
        let mut lines = self.lines.lock();
        if lines.len() == CAPACITY {
            lines.pop_front();
        }
        lines.push_back(line);
    }

    /// The latest `count` lines, oldest first.
    pub fn latest(&self, count: usize) -> Vec<Line> {
        let lines = self.lines.lock();
        lines
            .iter()
            .skip(lines.len().saturating_sub(count))
            .cloned()
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn texts(lines: &[Line]) -> Vec<&str> {
        lines.iter().map(|line| line.text.as_str()).collect()
    }

    #[test]
    fn keeps_order() {
        let log = Log::new();
        log.push("first");
        log.push("second");
        log.push("third");
        assert_eq!(texts(&log.latest(10)), ["first", "second", "third"]);
        assert_eq!(texts(&log.latest(2)), ["second", "third"]);
    }

    #[test]
    fn drops_oldest() {
        let log = Log::new();
        for i in 0..=CAPACITY {
            log.push(i.to_string());
        }
        let lines = log.latest(usize::MAX);
        assert_eq!(lines.len(), CAPACITY);
        assert_eq!(lines.first().map(|line| line.text.as_str()), Some("1"));
        assert_eq!(
            lines.last().map(|line| line.text.clone()),
            Some(CAPACITY.to_string())
        );
    }
}
