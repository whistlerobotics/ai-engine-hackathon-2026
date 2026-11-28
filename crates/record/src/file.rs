use crate::episode::{Episode, Image, RecordedProprio};
use base64::{Engine, engine::general_purpose::STANDARD as BASE64};
use color_eyre::{Result, eyre::eyre};
use policy_contract::{ARM_JOINTS, STEP, wire::Images};
use serde::Serialize;
use std::{
    fs::{self, File},
    io::{BufWriter, Write},
    path::{Path, PathBuf},
};

#[derive(Serialize)]
struct Header<'a> {
    episode_id: &'a str,
    task_id: &'a str,
    schema_version: u32,
    step_s: f64,
    started_at: &'a str,
    ended_by: &'a str,
    steps_without_image: u32,
    joint_names: [&'a str; 6],
}

const BLACK_JPEG: &[u8] = include_bytes!("../assets/black-640x360.jpg");

#[derive(Serialize)]
struct StepLine<'a> {
    step: usize,
    t: f64,
    images: Images,
    proprio: &'a RecordedProprio,
    gripper_command: f64,
}

/// Writes `<out_dir>/<task_id>/<episode_id>.json`, replacing any episode of the same name, with one
/// step per line so that the file stays readable however long the episode.
pub fn save(episode: &Episode, out_dir: &Path, ended_by: &str) -> Result<PathBuf> {
    let path = out_dir
        .join(&episode.task_id)
        .join(format!("{}.json", episode.id));
    let directory = path
        .parent()
        .ok_or_else(|| eyre!("{} has no parent", path.display()))?;
    fs::create_dir_all(directory)?;
    let partial = path.with_extension("json.part");

    let header = Header {
        episode_id: &episode.id,
        task_id: &episode.task_id,
        schema_version: 3,
        step_s: STEP.as_secs_f64(),
        started_at: &episode.started_at,
        ended_by,
        steps_without_image: episode.steps_without_image,
        joint_names: ARM_JOINTS,
    };
    let header = serde_json::to_string_pretty(&header)?;
    let header = header
        .strip_suffix("\n}")
        .ok_or_else(|| eyre!("the header does not end in a closing brace"))?;

    let mut writer = BufWriter::new(File::create(&partial)?);
    write!(writer, "{header},\n  \"steps\": [\n")?;
    for (index, step) in episode.steps.iter().enumerate() {
        if index > 0 {
            writer.write_all(b",\n")?;
        }
        let [wrist, left, right] = step.images.each_ref().map(|image| {
            BASE64.encode(match image {
                Image::Camera(jpeg) => jpeg.as_slice(),
                Image::Black => BLACK_JPEG,
            })
        });
        writer.write_all(b"    ")?;
        serde_json::to_writer(
            &mut writer,
            &StepLine {
                step: index,
                t: step.t,
                images: Images { wrist, left, right },
                proprio: &step.proprio,
                gripper_command: step.gripper_command,
            },
        )?;
    }
    writer.write_all(b"\n  ]\n}\n")?;
    writer
        .into_inner()
        .map_err(std::io::IntoInnerError::into_error)?
        .sync_all()?;
    fs::rename(&partial, &path)?;
    Ok(path)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::episode::{Observation, Step};
    use serde_json::Value;
    use std::time::{Duration, Instant};

    fn step(image: &Image) -> Observation {
        Observation {
            proprio: RecordedProprio {
                joint_pos: [0.0; 6],
                joint_vel: [0.0; 6],
                gripper: None,
                ee_pos: [0.0; 3],
                ee_quat: [1.0, 0.0, 0.0, 0.0],
            },
            images: std::array::from_fn(|_| image.clone()),
        }
    }

    #[test]
    fn layout() -> Result<()> {
        let start = Instant::now();
        let mut episode = Episode::new("task", start);
        episode.add_step(start, step(&Image::Black), 0.0);
        episode.add_step(start + STEP, step(&Image::Camera(BLACK_JPEG.to_vec())), 0.0);
        let out_dir = std::env::temp_dir().join(format!("record-test-{}", std::process::id()));

        let path = save(&episode, &out_dir, "release")?;
        let text = fs::read_to_string(&path)?;
        fs::remove_dir_all(&out_dir)?;

        let lines: Vec<&str> = text.lines().collect();
        assert_eq!(lines[1], format!("  \"episode_id\": \"{}\",", episode.id));
        assert_eq!(lines[lines.len() - 5], "  \"steps\": [");
        assert!(lines[lines.len() - 4].starts_with("    {\"step\":0,"));
        assert!(lines[lines.len() - 3].starts_with("    {\"step\":1,"));
        assert_eq!(&lines[lines.len() - 2..], ["  ]", "}"]);

        let keys = [
            "episode_id",
            "task_id",
            "schema_version",
            "step_s",
            "started_at",
            "ended_by",
            "steps_without_image",
            "joint_names",
            "steps",
        ];
        let positions: Vec<Option<usize>> = keys
            .iter()
            .map(|key| text.find(&format!("\n  \"{key}\": ")))
            .collect();
        assert!(positions.iter().all(Option::is_some), "{positions:?}");
        assert!(positions.is_sorted(), "{positions:?}");

        let json: Value = serde_json::from_str(&text)?;
        assert_eq!(json["steps_without_image"], 1);
        assert_eq!(json["steps"][1]["t"], 0.05);
        assert_eq!(json["steps"][0]["proprio"]["gripper"], Value::Null);
        assert_eq!(json["schema_version"], 3);
        let images = json["steps"][0]["images"].as_object().unwrap();
        assert_eq!(images.len(), 3);
        for role in ["wrist", "left", "right"] {
            assert_eq!(images[role], BASE64.encode(BLACK_JPEG));
        }
        assert_eq!(json["steps"][0]["gripper_command"], 1.0);
        Ok(())
    }

    #[test]
    fn t_rounded_to_tenth_ms() {
        let start = Instant::now();
        let mut episode = Episode::new("task", start);
        episode.add_step(
            start + Duration::from_micros(50_049),
            step(&Image::Black),
            0.0,
        );
        let Step { t, .. } = episode.steps[0].clone();
        assert!((t - 0.05).abs() < 1e-12, "{t}");
    }

    #[test]
    fn saves_distinct_named_camera_frames() -> Result<()> {
        let frames: [&[u8]; 3] = [
            include_bytes!("../../../task-packs/simulator/bolt_to_tray/sim-0/wrist.jpg"),
            include_bytes!("../../../task-packs/simulator/bolt_to_tray/sim-0/left.jpg"),
            include_bytes!("../../../task-packs/simulator/bolt_to_tray/sim-0/right.jpg"),
        ];
        let start = Instant::now();
        let mut episode = Episode::new("three_camera_test", start);
        let mut observation = step(&Image::Black);
        observation.images = frames.map(|jpeg| Image::Camera(jpeg.to_vec()));
        episode.add_step(start, observation, 0.0);
        let out_dir =
            std::env::temp_dir().join(format!("record-three-camera-test-{}", std::process::id()));
        let path = save(&episode, &out_dir, "release")?;
        let json: Value = serde_json::from_reader(File::open(path)?)?;
        fs::remove_dir_all(out_dir)?;
        for (role, expected) in ["wrist", "left", "right"].into_iter().zip(frames) {
            let actual = BASE64.decode(json["steps"][0]["images"][role].as_str().unwrap())?;
            assert_eq!(actual, expected);
            policy_contract::image::check_policy_jpeg(&actual).unwrap();
        }
        assert_eq!(json["steps_without_image"], 0);
        Ok(())
    }
}
