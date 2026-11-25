mod button;
mod freedrive;
mod ros;
mod throttle;

use crate::{
    ros::{BUTTON_PIN, IO_TOPIC, Ros, THROTTLE_PIN},
    throttle::{Fault, Hysteresis},
};
use color_eyre::Result;
use policy_contract::gripper;
use std::{
    mem::{self, Discriminant},
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
    },
    thread,
    time::{Duration, Instant},
};

/// The gripper's driver passes a new target on only every 15-20 ms, so faster gains nothing.
const CYCLE: Duration = Duration::from_millis(20);
const IO_STALE_AFTER: Duration = Duration::from_millis(100);
/// `freedrive_mode_controller` ends freedrive after its `inactive_timeout`, 1 s, without a message.
const KEEP_ALIVE: Duration = Duration::from_millis(200);

enum Phase {
    /// Until the button reads released, so that a press under way at startup, or across a gap in
    /// `io_states`, is not taken for a new one.
    WaitingForRelease,
    Idle,
    Guiding {
        kept_alive_at: Instant,
        hysteresis: Hysteresis,
    },
}

/// While the button is held, the arm is in freedrive and the throttle opens the gripper.
fn main() -> Result<()> {
    color_eyre::install()?;
    let stop = Arc::new(AtomicBool::new(false));
    ctrlc::set_handler({
        let stop = Arc::clone(&stop);
        move || stop.store(true, Ordering::Relaxed)
    })?;

    let ros = Ros::new()?;
    println!("Waiting for the controller manager.");
    while !ros.freedrive.controller_manager_ready() {
        if stop.load(Ordering::Relaxed) {
            return Ok(());
        }
        thread::sleep(CYCLE);
    }
    if ros.freedrive.recover()? {
        println!("Freedrive was left on; handed the arm back to joint_trajectory_controller.");
    }
    println!("Waiting for {IO_TOPIC} with the button on digital input {BUTTON_PIN} released.");

    let mut phase = Phase::WaitingForRelease;
    let result = run(&ros, &stop, &mut phase);
    if matches!(phase, Phase::Guiding { .. }) {
        ros.freedrive.stop()?;
        println!("Freedrive off.");
    }
    ros.publish_active(false)?;
    result
}

fn run(ros: &Ros, stop: &AtomicBool, phase: &mut Phase) -> Result<()> {
    let mut reported_fault = Some(mem::discriminant(&Fault::Stale));
    while !stop.load(Ordering::Relaxed) {
        let now = Instant::now();
        let pressed = ros
            .button()
            .filter(|button| now - button.at <= IO_STALE_AFTER)
            .map(|button| button.pressed);
        let volts = throttle::volts(ros.throttle(), now);
        report_throttle(volts, &mut reported_fault);
        advance(ros, phase, pressed, volts, now)?;
        ros.publish_active(matches!(phase, Phase::Guiding { .. }))?;
        thread::sleep(CYCLE);
    }
    Ok(())
}

/// `pressed` is None while the button's state is unknown.
fn advance(
    ros: &Ros,
    phase: &mut Phase,
    pressed: Option<bool>,
    volts: Result<f64, Fault>,
    now: Instant,
) -> Result<()> {
    match phase {
        Phase::WaitingForRelease => {
            if pressed == Some(false) {
                println!("Ready: hold the button to guide the arm by hand.");
                *phase = Phase::Idle;
            }
        }
        Phase::Idle => match pressed {
            Some(true) => match ros.freedrive.start() {
                Ok(()) => {
                    println!("Freedrive on.");
                    *phase = Phase::Guiding {
                        kept_alive_at: now,
                        hysteresis: Hysteresis::default(),
                    };
                }
                Err(error) => {
                    eprintln!(
                        "Freedrive did not start: {error:#}. Release the button to try again."
                    );
                    *phase = Phase::WaitingForRelease;
                }
            },
            Some(false) => {}
            None => {
                println!("No fresh {IO_TOPIC}; waiting for it with the button released.");
                *phase = Phase::WaitingForRelease;
            }
        },
        Phase::Guiding {
            kept_alive_at,
            hysteresis,
        } => {
            if pressed != Some(true) {
                // A failed stop leaves the phase as it is, for `main` to try once more on the way
                // out; a restart then finds the arm still in freedrive and recovers it.
                ros.freedrive.stop()?;
                if pressed == Some(false) {
                    println!("Freedrive off.");
                    *phase = Phase::Idle;
                } else {
                    println!("No fresh {IO_TOPIC}: freedrive off.");
                    *phase = Phase::WaitingForRelease;
                }
                return Ok(());
            }
            if now - *kept_alive_at >= KEEP_ALIVE {
                ros.freedrive.keep_alive()?;
                *kept_alive_at = now;
            }
            if let Ok(volts) = volts {
                let opening = hysteresis.update(throttle::opening(volts));
                ros.command_finger(gripper::finger(opening))?;
            }
        }
    }
    Ok(())
}

fn report_throttle(volts: Result<f64, Fault>, reported: &mut Option<Discriminant<Fault>>) {
    let fault = volts.err();
    let kind = fault.as_ref().map(mem::discriminant);
    if kind == *reported {
        return;
    }
    match fault {
        Some(fault) => {
            println!("Throttle on analog input {THROTTLE_PIN}: {fault}, so the gripper holds.");
        }
        None => println!("Throttle on analog input {THROTTLE_PIN} reads normally."),
    }
    *reported = kind;
}
