use crate::{
    Recorder, Recording,
    episode::{End, Episode, Observation, Stale, observe},
    ros::HandGuiding,
    signal_name,
};
use color_eyre::{Result, eyre::eyre};
use policy_contract::{STEP, gripper};
use std::{
    io::{self, BufRead, Write},
    sync::{
        atomic::{AtomicUsize, Ordering},
        mpsc::{self, Receiver, TryRecvError},
    },
    thread,
    time::{Duration, Instant},
};

enum Command {
    Start,
    Finish,
}

pub(super) fn run(recorder: &mut Recorder, signal: &AtomicUsize) -> Result<()> {
    let (send, receive) = mpsc::channel();
    thread::spawn(move || {
        for line in io::stdin().lock().lines() {
            let command = line.map_err(Into::into).and_then(|line| match line.trim() {
                "start" => Ok(Command::Start),
                "finish" => Ok(Command::Finish),
                value => Err(eyre!("Unknown policy capture command: {value:?}")),
            });
            if send.send(command).is_err() {
                break;
            }
        }
    });

    loop {
        if command(&receive, signal)?.is_some() {
            return Err(eyre!("Wait for READY before starting a policy take"));
        }
        let now = Instant::now();
        if recorder.ros.hand_guiding(now) != Some(HandGuiding::Guiding)
            && sample(recorder, now).is_ok()
        {
            break;
        }
        thread::sleep(Duration::from_millis(10));
    }
    println!("READY");
    io::stdout().flush()?;

    let (mut recording, mut finger) = loop {
        match command(&receive, signal)? {
            Some(Command::Start) => {
                let now = Instant::now();
                ensure_clear(recorder, now)?;
                let observation = sample(recorder, now)?;
                let finger = fresh_finger_command(recorder, now)
                    .or_else(|| observation.proprio.gripper.map(gripper::finger))
                    .ok_or_else(|| eyre!("No gripper position to start policy capture"))?;
                let mut episode = Episode::new(&recorder.args.task, now);
                episode.add_step(now, observation, finger);
                break (
                    Recording {
                        episode,
                        next_step_at: now + STEP,
                    },
                    finger,
                );
            }
            Some(Command::Finish) => return Err(eyre!("No policy take has started")),
            None => thread::sleep(Duration::from_millis(10)),
        }
    };
    println!("STARTED");
    io::stdout().flush()?;

    let end = loop {
        let requested = command(&receive, signal)?;
        let now = Instant::now();
        ensure_clear(recorder, now)?;
        match requested {
            Some(Command::Finish) => {
                sample(recorder, now)?;
                break End::Finish;
            }
            Some(Command::Start) => return Err(eyre!("A policy take is already recording")),
            None => {}
        }
        if now >= recording.next_step_at {
            let observation = sample(recorder, now)?;
            finger = fresh_finger_command(recorder, now).unwrap_or(finger);
            recording.episode.add_step(now, observation, finger);
            recording.next_step_at += STEP;
            if now - recording.next_step_at > STEP {
                recording.next_step_at = now + STEP;
            }
            if recording.episode.steps.len() >= recorder.max_steps {
                break End::TimeLimit;
            }
        }
        thread::sleep(Duration::from_millis(2));
    };

    if recording.episode.steps.len() < 2 {
        return Err(eyre!("Policy capture requires at least two samples"));
    }
    recorder.finish(&recording.episode, &end)
}

fn command(input: &Receiver<Result<Command>>, signal: &AtomicUsize) -> Result<Option<Command>> {
    if let Some(name) = signal_name(signal.load(Ordering::Relaxed)) {
        return Err(eyre!("Policy capture cancelled by {name}"));
    }
    match input.try_recv() {
        Ok(command) => command.map(Some),
        Err(TryRecvError::Empty) => Ok(None),
        Err(TryRecvError::Disconnected) => {
            Err(eyre!("Policy capture control closed before finish"))
        }
    }
}

fn ensure_clear(recorder: &Recorder, now: Instant) -> Result<()> {
    if recorder.ros.hand_guiding(now) == Some(HandGuiding::Guiding) {
        return Err(eyre!("Hand-guiding is active during policy capture"));
    }
    Ok(())
}

fn sample(recorder: &Recorder, now: Instant) -> Result<Observation, Stale> {
    observe(&recorder.ros, now, false)
}

fn fresh_finger_command(recorder: &Recorder, now: Instant) -> Option<f64> {
    recorder
        .ros
        .finger_command()
        .filter(|command| command.is_fresh(now))
        .map(|command| command.value)
}
