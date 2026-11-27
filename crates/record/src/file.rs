use crate::episode::{Episode, Image, RecordedProprio};
use base64::{Engine, engine::general_purpose::STANDARD as BASE64};
use color_eyre::{Result, eyre::eyre};
use policy_contract::{ARM_JOINTS, STEP, image::BLACK_JPEG};
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

#[derive(Serialize)]
struct Images {
    top: String,
}

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
        schema_version: 2,
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
        let jpeg = match &step.image {
            Image::Camera(jpeg) => jpeg.as_slice(),
            Image::Black => BLACK_JPEG,
        };
        writer.write_all(b"    ")?;
        serde_json::to_writer(
            &mut writer,
            &StepLine {
                step: index,
                t: step.t,
                images: Images {
                    top: BASE64.encode(jpeg),
                },
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

    fn step(image: Image) -> Observation {
        Observation {
            proprio: RecordedProprio {
                joint_pos: [0.0; 6],
                joint_vel: [0.0; 6],
                gripper: None,
                ee_pos: [0.0; 3],
                ee_quat: [1.0, 0.0, 0.0, 0.0],
            },
            image,
        }
    }

    #[test]
    fn layout() -> Result<()> {
        let start = Instant::now();
        let mut episode = Episode::new("task", start);
        episode.add_step(start, step(Image::Black), 0.0);
        episode.add_step(start + STEP, step(Image::Camera(BLACK_JPEG.to_vec())), 0.0);
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
        assert_eq!(json["steps"][0]["images"]["top"], BASE64.encode(BLACK_JPEG));
        assert_eq!(json["steps"][0]["gripper_command"], 1.0);
        Ok(())
    }

    #[test]
    fn t_rounded_to_tenth_ms() {
        let start = Instant::now();
        let mut episode = Episode::new("task", start);
        episode.add_step(
            start + Duration::from_micros(50_049),
            step(Image::Black),
            0.0,
        );
        let Step { t, .. } = episode.steps[0].clone();
        assert!((t - 0.05).abs() < 1e-12, "{t}");
    }
}
