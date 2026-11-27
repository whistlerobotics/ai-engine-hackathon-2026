use base64::{Engine, engine::general_purpose::STANDARD};
use color_eyre::{
    Result,
    eyre::{WrapErr, ensure, eyre},
};
use policy_contract::{Joints, SCHEMA_VERSION, image::check_policy_jpeg, wire::Images};
use serde::Deserialize;
use sha2::{Digest, Sha256};
use std::{
    collections::HashMap,
    fs,
    path::{Path, PathBuf},
};

#[derive(Deserialize)]
struct Manifest {
    schema_version: u32,
    revision: String,
    start_pose_rad: Joints,
    tasks: Vec<Entry>,
}

#[derive(Deserialize)]
struct Entry {
    task_id: String,
    instruction: String,
    variant: String,
    seed: i64,
    wrist_pose_rad: Joints,
    goal: Goals,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Goals {
    wrist: Asset,
    left: Asset,
    right: Asset,
}
#[derive(Deserialize)]
struct Asset {
    width: u32,
    height: u32,
    encoding: String,
    path: PathBuf,
    sha256: String,
}

pub struct TaskPack {
    pub revision: String,
    pub start: Joints,
    root: PathBuf,
    entries: HashMap<String, Entry>,
}

#[derive(Debug)]
pub struct Task {
    pub id: String,
    pub instruction: String,
    pub goal: Images,
    pub variant: String,
    pub seed: i64,
}

impl TaskPack {
    pub fn open(path: &Path) -> Result<Self> {
        let path = path
            .canonicalize()
            .wrap_err("task pack manifest unavailable")?;
        let manifest: Manifest =
            serde_json::from_reader(fs::File::open(&path)?).wrap_err("invalid task manifest")?;
        ensure!(
            manifest.schema_version == SCHEMA_VERSION && !manifest.revision.trim().is_empty(),
            "task pack requires schema 3 and a revision"
        );
        ensure!(
            manifest
                .start_pose_rad
                .iter()
                .enumerate()
                .all(|(j, q)| q.is_finite() && q.abs() <= joint_limit(j)),
            "start pose exceeds mock joint limits"
        );
        let mut entries = HashMap::new();
        for task in manifest.tasks {
            ensure!(
                matches!(
                    task.task_id.as_str(),
                    "bolt_to_tray" | "bolt_and_nut_to_fixture" | "1" | "2" | "3" | "4"
                ),
                "invalid policy-facing task ID in manifest"
            );
            ensure!(
                entries.insert(task.task_id.clone(), task).is_none(),
                "duplicate policy-facing task ID"
            );
        }
        Ok(Self {
            revision: manifest.revision,
            start: manifest.start_pose_rad,
            root: path
                .parent()
                .ok_or_else(|| eyre!("invalid task pack location"))?
                .to_owned(),
            entries,
        })
    }

    pub fn select(&self, id: &str) -> Result<Task> {
        let entry = self
            .entries
            .get(id)
            .ok_or_else(|| eyre!("unknown policy-facing task ID"))?;
        ensure!(
            !entry.instruction.trim().is_empty()
                && !entry.variant.is_empty()
                && entry
                    .variant
                    .bytes()
                    .all(|b| b.is_ascii_alphanumeric() || matches!(b, b'-' | b'_'))
                && entry.wrist_pose_rad.iter().all(|q| q.is_finite()),
            "incomplete task context"
        );
        let load = |role: &str, asset: &Asset| -> Result<String> {
            ensure!(
                (asset.width, asset.height) == (640, 360) && asset.encoding == "jpeg",
                "{role} requires 640x360 JPEG metadata"
            );
            let expected = Path::new(id)
                .join(&entry.variant)
                .join(format!("{role}.jpg"));
            ensure!(
                asset.path == expected,
                "{role} goal path does not match task and variant"
            );
            let path = self
                .root
                .join(&expected)
                .canonicalize()
                .wrap_err_with(|| format!("{role} goal unavailable"))?;
            ensure!(path.starts_with(&self.root), "goal path outside task pack");
            let bytes = fs::read(path)?;
            ensure!(
                format!("{:x}", Sha256::digest(&bytes)) == asset.sha256,
                "{role} goal SHA-256 mismatch"
            );
            check_policy_jpeg(&bytes).map_err(|msg| eyre!("{role} goal: {msg}"))?;
            Ok(STANDARD.encode(bytes))
        };
        let goals = &entry.goal;
        ensure!(
            goals.wrist.sha256 != goals.left.sha256
                && goals.wrist.sha256 != goals.right.sha256
                && goals.left.sha256 != goals.right.sha256,
            "three distinct named goal views required"
        );
        let goal = Images {
            wrist: load("wrist", &goals.wrist)?,
            left: load("left", &goals.left)?,
            right: load("right", &goals.right)?,
        };
        Ok(Task {
            id: id.to_owned(),
            instruction: entry.instruction.clone(),
            goal,
            variant: entry.variant.clone(),
            seed: entry.seed,
        })
    }
}

pub const fn joint_limit(joint: usize) -> f64 {
    if joint == 2 {
        2.792526803190927
    } else {
        6.28319
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn simulator_pack() -> TaskPack {
        let manifest =
            Path::new(env!("CARGO_MANIFEST_DIR")).join("../../task-packs/simulator/manifest.json");
        TaskPack::open(&manifest).unwrap()
    }
    #[test]
    fn shipped_public_task_contexts_are_loadable() {
        let pack = simulator_pack();
        assert_eq!(pack.entries.len(), 2);
        for task in ["bolt_to_tray", "bolt_and_nut_to_fixture"] {
            let selected = pack.select(task).unwrap();
            assert_eq!(selected.id, task);
        }
    }
    #[test]
    fn swapped_role_refused() {
        let mut pack = simulator_pack();
        let goals = &mut pack.entries.get_mut("bolt_to_tray").unwrap().goal;
        std::mem::swap(&mut goals.wrist, &mut goals.left);
        std::assert_matches!(pack.select("bolt_to_tray"), Err(_));
    }
    #[test]
    fn mismatched_digest_refused() {
        let mut pack = simulator_pack();
        pack.entries
            .get_mut("bolt_to_tray")
            .unwrap()
            .goal
            .left
            .sha256
            .push('a');
        std::assert_matches!(pack.select("bolt_to_tray"), Err(_));
    }
}
