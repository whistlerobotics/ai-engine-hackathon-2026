use crate::{
    app::{App, Phase, Robot},
    episode::{BUDGET, MAX_TRACKING_ERROR, Progress, STEPS},
    gripper::GripperHardware,
    policy::{Health, Readiness},
    ros::Mode,
};
use policy_contract::STEP;
use ratatui::{
    Frame,
    layout::{Constraint, Layout, Rect},
    style::{Color, Style, Stylize},
    text::{Line, Span},
    widgets::{Block, LineGauge, Paragraph, Sparkline, SparklineBar},
};
use std::time::Duration;

pub fn draw(frame: &mut Frame<'_>, app: &App<'_>, robot: Robot, mode: Mode) {
    let [header, middle, log, footer] = Layout::vertical([
        Constraint::Length(4),
        Constraint::Length(12),
        Constraint::Min(3),
        Constraint::Length(1),
    ])
    .areas(frame.area());
    let [robot_area, episode_area] =
        Layout::horizontal([Constraint::Length(30), Constraint::Fill(1)]).areas(middle);
    let phase = *app.shared.phase.lock();

    draw_header(frame, header, app, phase, mode);
    draw_robot(frame, robot_area, robot);
    draw_episode(frame, episode_area, app, phase);
    draw_log(frame, log, app);
    frame.render_widget(footer_line(app, phase), footer);
}

fn draw_header(frame: &mut Frame<'_>, area: Rect, app: &App<'_>, phase: Phase, mode: Mode) {
    let mut policy = vec![Span::raw("Policy  "), Span::raw(app.policy_url.clone())];
    match app.shared.policy.lock().as_ref() {
        None => policy.push(Span::raw("    not polled yet")),
        Some(status) => policy.extend([
            Span::raw("    health "),
            health_label(&status.health),
            Span::raw("    ready "),
            readiness_label(&status.readiness),
        ]),
    }
    if phase == Phase::Running {
        policy.push(Span::raw("    polling paused during the episode"));
    }
    let mode = match mode {
        Mode::Mock => "MOCK",
        Mode::Real => "REAL ARM",
    };
    let connection = Line::from(format!(
        "Task    {}    Policy input  wrist, left, right    {mode}",
        app.task_id,
    ));
    frame.render_widget(
        Paragraph::new(vec![Line::from(policy), connection])
            .block(Block::bordered().title(" Policy runner ")),
        area,
    );
}

/// Why the policy is down is in the log.
fn health_label(health: &Health) -> Span<'static> {
    match health {
        Health::Up => Span::styled("up", Color::Green),
        Health::Unreachable(_) | Health::HttpStatus(_) => Span::styled("down", Color::Red),
    }
}

/// Names a contract mismatch, the one reason for `no` that nothing else on screen explains.
fn readiness_label(readiness: &Readiness) -> Span<'static> {
    match readiness {
        Readiness::Ready => Span::styled("yes", Color::Green),
        Readiness::Incompatible {
            contract,
            schema_version,
        } => {
            let contract = contract.as_deref().unwrap_or("no contract");
            let schema_version =
                schema_version.map_or_else(|| "no schema version".to_owned(), |v| v.to_string());
            Span::styled(
                format!("no (incompatible: {contract}, {schema_version})"),
                Color::Yellow,
            )
        }
        Readiness::NotReady
        | Readiness::Malformed(_)
        | Readiness::Unreachable(_)
        | Readiness::HttpStatus(_) => Span::styled("no", Color::Yellow),
    }
}

fn dot(color: Color) -> Span<'static> {
    Span::styled("● ", color)
}

fn draw_robot(frame: &mut Frame<'_>, area: Rect, robot: Robot) {
    let live = |name: &str, live: bool| {
        let (text, color) = if live {
            ("live", Color::Green)
        } else {
            ("stale", Color::Red)
        };
        Line::from(vec![
            Span::raw(format!("{name:<13}")),
            dot(color),
            Span::raw(text),
        ])
    };
    let start_pose = if robot.at_start {
        Span::styled("✓ reached", Color::Green)
    } else {
        Span::raw("✗ away")
    };
    let setting = |name: &str, value: String| Line::from(format!("{name:<13}{value}"));
    let (grip_speed, grip_force) = match robot.gripper_hardware {
        None => ("–".to_owned(), "–".to_owned()),
        Some(GripperHardware::Other) => ("– mock".to_owned(), "– mock".to_owned()),
        Some(GripperHardware::Robotiq {
            speed_multiplier,
            force,
        }) => (format!("{speed_multiplier:.2}×"), format!("{force:.1} N")),
    };
    let lines = vec![
        live("Arm", robot.arm_live),
        live("Gripper", robot.gripper_live),
        live("Left camera", robot.camera_live[1]),
        live("Right camera", robot.camera_live[2]),
        live("Wrist camera", robot.camera_live[0]),
        live("Policy input", robot.camera_live.iter().all(|&live| live)),
        Line::from(vec![Span::raw(format!("{:<13}", "Start pose")), start_pose]),
        setting(
            "Arm speed",
            robot
                .speed_scaling
                .map_or_else(|| "–".to_owned(), |percent| format!("{percent:.0} %")),
        ),
        setting("Grip speed", grip_speed),
        setting("Grip force", grip_force),
    ];
    frame.render_widget(
        Paragraph::new(lines).block(Block::bordered().title(" Robot ")),
        area,
    );
}

fn draw_episode(frame: &mut Frame<'_>, area: Rect, app: &App<'_>, phase: Phase) {
    let block = Block::bordered().title(" Episode ");
    let inner = block.inner(area);
    frame.render_widget(block, area);

    let rows: [Rect; 6] = Layout::vertical([Constraint::Length(1); 6]).areas(inner);
    let phase = match phase {
        Phase::Idle => "idle",
        Phase::MovingToStart => "moving to the start pose",
        Phase::AtStart => "at the start pose",
        Phase::Running => "running an episode",
    };
    frame.render_widget(Paragraph::new(format!("{:<10}{phase}", "Phase")), rows[0]);

    let Some(progress) = app.shared.progress.lock().clone() else {
        frame.render_widget(Paragraph::new("No episode yet."), rows[1]);
        return;
    };
    draw_progress(frame, &rows[1..], &progress);
}

fn draw_progress(frame: &mut Frame<'_>, rows: &[Rect], progress: &Progress) {
    let [steps, budget, counts, tracking, latencies] = rows else {
        return;
    };
    let elapsed = progress.counts.http;
    gauge_row(
        frame,
        *steps,
        "Steps",
        f64::from(progress.counts.steps) / f64::from(STEPS),
        format!("{} / {STEPS}", progress.counts.steps),
        Color::Cyan,
    );
    gauge_row(
        frame,
        *budget,
        "Budget",
        elapsed.as_secs_f64() / BUDGET.as_secs_f64(),
        format!("{:.1} / {} s", elapsed.as_secs_f64(), BUDGET.as_secs()),
        Color::Blue,
    );
    frame.render_widget(
        Paragraph::new(format!("{:<10}{}", "Late", progress.counts.late)),
        *counts,
    );
    let margin = progress.tracking_error / MAX_TRACKING_ERROR;
    let tracking_color = if margin > 0.8 {
        Color::Red
    } else if margin > 0.5 {
        Color::Yellow
    } else {
        Color::Green
    };
    gauge_row(
        frame,
        *tracking,
        "Tracking",
        margin,
        format!("{:.2} / {MAX_TRACKING_ERROR} rad", progress.tracking_error),
        tracking_color,
    );
    latency_row(frame, *latencies, &progress.latencies);
}

fn row_areas(area: Rect) -> [Rect; 3] {
    Layout::horizontal([
        Constraint::Length(10),
        Constraint::Fill(1),
        Constraint::Length(18),
    ])
    .spacing(1)
    .areas(area)
}

fn gauge_row(
    frame: &mut Frame<'_>,
    area: Rect,
    name: &str,
    ratio: f64,
    text: String,
    color: Color,
) {
    let [name_area, gauge_area, text_area] = row_areas(area);
    frame.render_widget(Paragraph::new(name), name_area);
    frame.render_widget(
        LineGauge::default()
            .ratio(ratio.clamp(0.0, 1.0))
            .label("")
            .filled_symbol("█")
            .unfilled_symbol("░")
            .filled_style(Style::new().fg(color))
            .unfilled_style(Style::new().fg(Color::DarkGray)),
        gauge_area,
    );
    frame.render_widget(Paragraph::new(text), text_area);
}

/// The latest `/act` latencies, scaled so the 50 ms step is at half
/// height, with slower replies in yellow.
fn latency_row(frame: &mut Frame<'_>, area: Rect, latencies: &[Duration]) {
    let [name_area, chart_area, text_area] = row_areas(area);
    let millis = |latency: &Duration| u64::try_from(latency.as_millis()).unwrap_or(u64::MAX);
    // Replies faster than an eighth of the scale would draw nothing at all.
    let lowest_bar = millis(&(STEP * 2)).div_ceil(8);
    let shown = latencies
        .iter()
        .skip(
            latencies
                .len()
                .saturating_sub(usize::from(chart_area.width)),
        )
        .map(|latency| {
            let color = if *latency > STEP {
                Color::Yellow
            } else {
                Color::Green
            };
            SparklineBar::from(millis(latency).max(lowest_bar)).style(Style::new().fg(color))
        });
    frame.render_widget(Paragraph::new("/act"), name_area);
    frame.render_widget(
        Sparkline::default().data(shown).max(millis(&(STEP * 2))),
        chart_area,
    );
    let last = latencies.last().map_or_else(String::new, |latency| {
        format!("last {} ms", millis(latency))
    });
    frame.render_widget(Paragraph::new(last), text_area);
}

fn draw_log(frame: &mut Frame<'_>, area: Rect, app: &App<'_>) {
    let block = Block::bordered().title(" Log ");
    let lines: Vec<Line<'_>> = app
        .shared
        .log
        .latest(usize::from(block.inner(area).height))
        .into_iter()
        .map(|line| Line::from(format!("{:>7.1} s  {}", line.at.as_secs_f64(), line.text)))
        .collect();
    frame.render_widget(Paragraph::new(lines).block(block), area);
}

fn footer_line(app: &App<'_>, phase: Phase) -> Line<'static> {
    let key = |key: &'static str, what: &'static str| {
        [Span::raw(key).bold(), Span::raw(format!(" {what}   "))]
    };
    if let Some(task) = &app.editing {
        let mut line = vec![
            Span::raw("New task: "),
            Span::raw(task.clone()),
            Span::raw("█   "),
        ];
        line.extend(key("Enter", "confirm"));
        line.extend(key("Esc", "cancel"));
        return Line::from(line);
    }
    let keys: Vec<[Span<'static>; 2]> = match phase {
        Phase::Idle => vec![
            key("Enter", "move to the start pose"),
            key("t", "change the task"),
            key("q", "quit"),
        ],
        Phase::AtStart => vec![
            key("Enter", "start the episode"),
            key("t", "change the task"),
            key("q", "quit"),
        ],
        Phase::MovingToStart | Phase::Running => {
            vec![key("Esc or Ctrl-C", "abort and hold")]
        }
    };
    Line::from(keys.into_iter().flatten().collect::<Vec<_>>())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{app::Shared, episode::Counts, log::Log, policy::PolicyStatus};
    use ratatui::{Terminal, backend::TestBackend};
    use std::sync::{Arc, mpsc};
    use test_case::test_case;

    const LIVE: Robot = Robot {
        arm_live: true,
        gripper_live: true,
        camera_live: [true; 3],
        controller_live: true,
        gripper_controller_live: true,
        at_start: false,
        speed_scaling: Some(100.0),
        gripper_hardware: None,
    };

    fn screen(shared: &Shared, robot: Robot) -> Vec<String> {
        let (commands, _received) = mpsc::channel();
        let app = App::new(
            shared,
            commands,
            "http://policy.test:8000".to_owned(),
            "bench_lift_can_v0".to_owned(),
        );
        let mut terminal = Terminal::new(TestBackend::new(120, 24)).unwrap();
        terminal
            .draw(|frame| draw(frame, &app, robot, Mode::Mock))
            .unwrap();
        let buffer = terminal.backend().buffer();
        buffer
            .content()
            .chunks(usize::from(buffer.area.width))
            .map(|row| row.iter().map(ratatui::buffer::Cell::symbol).collect())
            .collect()
    }

    fn shows(screen: &[String], text: &str) -> bool {
        screen.iter().any(|row| row.contains(text))
    }

    fn shared() -> Shared {
        Shared::new(Arc::new(Log::new()))
    }

    #[test]
    fn connection() {
        let screen = screen(&shared(), LIVE);
        assert!(shows(&screen, "http://policy.test:8000"), "{screen:#?}");
        assert!(shows(&screen, "bench_lift_can_v0"), "{screen:#?}");
        assert!(shows(&screen, "MOCK"), "{screen:#?}");
        assert!(
            shows(&screen, "Policy input  wrist, left, right"),
            "{screen:#?}"
        );
    }

    #[test_case(Health::Up, Readiness::Ready => "health up    ready yes"; "ready")]
    #[test_case(Health::Up, Readiness::NotReady => "health up    ready no"; "not ready")]
    #[test_case(
        Health::Unreachable("refused".to_owned()),
        Readiness::Unreachable("refused".to_owned())
        => "health down    ready no";
        "unreachable")]
    #[test_case(Health::HttpStatus(503), Readiness::HttpStatus(503) => "health down    ready no"; "http error")]
    #[test_case(
        Health::Up,
        Readiness::Incompatible { contract: Some("v1".to_owned()), schema_version: Some(2) }
        => "health up    ready no (incompatible: v1, 2)";
        "incompatible")]
    fn policy_status(health: Health, readiness: Readiness) -> String {
        let shared = shared();
        *shared.policy.lock() = Some(PolicyStatus { health, readiness });
        let screen = screen(&shared, LIVE);
        let header = screen[1].trim_matches([' ', '│']);
        header
            .trim_start_matches("Policy  http://policy.test:8000    ")
            .to_owned()
    }

    #[test_case(&Health::Up => Some(Color::Green); "up")]
    #[test_case(&Health::Unreachable("refused".to_owned()) => Some(Color::Red); "down")]
    fn health_color(health: &Health) -> Option<Color> {
        health_label(health).style.fg
    }

    #[test_case(&Readiness::Ready => Some(Color::Green); "ready")]
    #[test_case(&Readiness::NotReady => Some(Color::Yellow); "not ready")]
    fn readiness_color(readiness: &Readiness) -> Option<Color> {
        readiness_label(readiness).style.fg
    }

    #[test]
    fn stale_arm() {
        let robot = Robot {
            arm_live: false,
            camera_live: [true; 3],
            ..LIVE
        };
        let screen = screen(&shared(), robot);
        assert!(shows(&screen, "Arm          ● stale"), "{screen:#?}");
        assert!(shows(&screen, "Gripper      ● live"), "{screen:#?}");
        assert!(shows(&screen, "Wrist camera ● live"), "{screen:#?}");
    }

    #[test_case(None, None => ["Arm speed    –", "Grip speed   –", "Grip force   –"]; "nothing received")]
    #[test_case(Some(100.0), Some(GripperHardware::Other)
        => ["Arm speed    100 %", "Grip speed   – mock", "Grip force   – mock"]; "mock")]
    #[test_case(Some(30.0), Some(GripperHardware::Robotiq { speed_multiplier: 1.0, force: 117.5 })
        => ["Arm speed    30 %", "Grip speed   1.00×", "Grip force   117.5 N"]; "real")]
    fn speeds_and_force(
        speed_scaling: Option<f64>,
        gripper_hardware: Option<GripperHardware>,
    ) -> [String; 3] {
        let robot = Robot {
            speed_scaling,
            gripper_hardware,
            ..LIVE
        };
        let screen = screen(&shared(), robot);
        ["Arm speed", "Grip speed", "Grip force"].map(|name| {
            screen
                .iter()
                .find_map(|row| {
                    let start = row.find(name)?;
                    Some(row[start..].split('│').next()?.trim_end().to_owned())
                })
                .unwrap_or_default()
        })
    }

    #[test_case([true,false,false])]
    #[test_case([false,true,false])]
    #[test_case([false,false,true])]
    fn independent_camera_rows(cameras: [bool; 3]) {
        let screen = screen(
            &shared(),
            Robot {
                camera_live: cameras,
                ..LIVE
            },
        );
        for (role, live) in ["Wrist camera", "Left camera", "Right camera"]
            .into_iter()
            .zip(cameras)
        {
            let state = if live { "live" } else { "stale" };
            assert!(
                shows(&screen, &format!("{role:<13}● {state}")),
                "{screen:#?}"
            );
        }
        for label in [
            "Start pose",
            "Arm speed",
            "Grip speed",
            "Grip force",
            "No episode yet.",
            "Enter move to the start pose",
        ] {
            assert!(shows(&screen, label), "{screen:#?}");
        }
    }

    #[test]
    fn running_episode() {
        let shared = shared();
        *shared.phase.lock() = Phase::Running;
        *shared.progress.lock() = Some(Progress {
            counts: Counts {
                steps: 120,
                late: 3,
                http: Duration::from_secs(3),
            },
            latencies: vec![Duration::from_millis(23), Duration::from_millis(80)],
            tracking_error: 0.04,
        });
        let screen = screen(&shared, LIVE);
        assert!(shows(&screen, "running an episode"), "{screen:#?}");
        assert!(shows(&screen, &format!("120 / {STEPS}")), "{screen:#?}");
        assert!(shows(&screen, "Late      3"), "{screen:#?}");
        assert!(shows(&screen, "last 80 ms"), "{screen:#?}");

        let footer = screen.last().map_or("", String::as_str).trim_end();
        assert_eq!(footer, "Esc or Ctrl-C abort and hold");
    }
}
