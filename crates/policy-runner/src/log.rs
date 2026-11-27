use parking_lot::Mutex;
use std::{
    collections::VecDeque,
    fs::{File, OpenOptions},
    io::{self, Write},
    os::unix::fs::OpenOptionsExt,
    path::Path,
    sync::atomic::{AtomicU64, Ordering},
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
    request: AtomicU64,
    file: Mutex<Option<File>>,
    lines: Mutex<VecDeque<Line>>,
}

impl Log {
    pub fn new() -> Self {
        Self {
            started: Instant::now(),
            request: AtomicU64::new(0),
            file: Mutex::new(None),
            lines: Mutex::new(VecDeque::with_capacity(CAPACITY)),
        }
    }

    pub fn open(path: &Path) -> io::Result<Self> {
        let file = OpenOptions::new()
            .create(true)
            .append(true)
            .mode(0o600)
            .open(path)?;
        let log = Self::new();
        *log.file.lock() = Some(file);
        Ok(log)
    }

    pub fn begin_request(&self) {
        let now = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos();
        self.request
            .store(u64::try_from(now).unwrap_or(u64::MAX), Ordering::SeqCst);
    }

    pub fn end_request(&self) {
        self.request.store(0, Ordering::SeqCst);
    }

    pub fn push(&self, text: impl Into<String>) {
        let line = Line {
            at: self.started.elapsed(),
            text: text.into(),
        };
        let mut line = line;
        if let Some(file) = self.file.lock().as_mut() {
            let stamp = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos();
            let id = self.request.load(Ordering::SeqCst);
            let record = serde_json::json!({"utc_ns":stamp.to_string(),"request_id":(id!=0).then(||id.to_string()),"event":&line.text});
            if let Err(error) = writeln!(file, "{record}").and_then(|()| file.flush()) {
                line.text = format!("Event log write failed: {error} / {}", line.text);
            }
        }
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
