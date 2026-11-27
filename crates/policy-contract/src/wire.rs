use crate::Joints;
use serde::{Deserialize, Serialize};
use serde_json::{Map, Value};

const MAX_DELTA: f64 = 0.05;
const MAX_CHUNK: usize = 16;

#[derive(Debug, Serialize)]
pub struct ActRequest<'a> {
    pub episode_id: &'a str,
    pub step: u32,
    pub task_id: &'a str,
    pub schema_version: u32,
    pub instruction: &'a str,
    pub images: Images,
    pub goal: &'a Images,
    pub proprio: Proprio,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Images {
    pub wrist: String,
    pub left: String,
    pub right: String,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct Proprio {
    pub joint_pos: Joints,
    pub joint_vel: Joints,
    pub gripper: f64,
    pub ee_pos: [f64; 3],
    pub ee_quat: [f64; 4],
}

/// One validated 50 ms action.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Action {
    pub delta: Joints,
    /// The commanded opening: 0 closed, 1 open.
    pub gripper: f64,
}

#[derive(Debug, Deserialize)]
struct Reply {
    episode_id: String,
    step: u64,
    action: ReplyAction,
}

#[derive(Debug, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
enum ReplyAction {
    JointDelta(JointDelta),
    ActionChunk {
        actions: Vec<ChunkedAction>,
        #[serde(default = "missing_chunk_len", deserialize_with = "chunk_length")]
        chunk_len: Option<usize>,
        #[serde(flatten)]
        unknown: Map<String, Value>,
    },
}

/// A chunk's actions still carry `"type": "joint_delta"`, and can be nothing else.
#[derive(Debug, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
enum ChunkedAction {
    JointDelta(JointDelta),
}

#[derive(Debug, Deserialize)]
struct JointDelta {
    values: Vec<f64>,
    gripper: f64,
    #[serde(flatten)]
    unknown: Map<String, Value>,
}

#[derive(Debug, thiserror::Error)]
pub enum InvalidReply {
    #[error("invalid UTF-8 in JSON response")]
    Encoding,
    #[error("malformed reply")]
    Malformed(#[from] serde_json::Error),
    #[error("unknown field {0}")]
    UnknownField(String),
    #[error("{at} has {count} entries, not 6")]
    WrongValueCount { at: String, count: usize },
    #[error("a chunk of {0} actions, not 1 to {MAX_CHUNK}")]
    ChunkSize(usize),
    #[error("chunk_len is {chunk_len}, but there are {actions} actions")]
    ChunkLenMismatch { chunk_len: usize, actions: usize },
    #[error("{at} is {received:?}, not {expected:?}")]
    WrongEcho {
        at: &'static str,
        expected: String,
        received: String,
    },
}

/// The `X-RC-*` headers of a reply, if present.
#[derive(Debug, Default)]
pub struct EchoHeaders {
    pub episode_id: Option<String>,
    pub step: Option<String>,
}

/// Validates a reply to the request for `episode_id` and `step`, in full, before any of it is
/// applied.
///
/// # Errors
///
/// The first way in which the reply breaks the contract.
pub fn validate(
    body: &str,
    headers: &EchoHeaders,
    episode_id: &str,
    step: u32,
) -> Result<Vec<Action>, InvalidReply> {
    let reply: Reply = serde_json::from_value(super::strict_json::parse(body)?)?;
    check_echo("episode_id", episode_id, &reply.episode_id)?;
    check_echo("step", &step.to_string(), &reply.step.to_string())?;
    let episode_header = headers.episode_id.as_deref().unwrap_or("missing");
    let step_header = headers.step.as_deref().unwrap_or("missing");
    check_echo("X-RC-Episode-Id", episode_id, episode_header)?;
    check_echo("X-RC-Step", &step.to_string(), step_header)?;

    match reply.action {
        ReplyAction::JointDelta(action) => Ok(vec![joint_delta(&action, "action")?]),
        ReplyAction::ActionChunk {
            actions,
            chunk_len,
            unknown,
        } => {
            if let Some(field) = ["values", "gripper"]
                .into_iter()
                .find(|key| unknown.contains_key(*key))
            {
                return Err(InvalidReply::UnknownField(format!("action.{field}")));
            }
            if !(1..=MAX_CHUNK).contains(&actions.len()) {
                return Err(InvalidReply::ChunkSize(actions.len()));
            }
            if let Some(chunk_len) = chunk_len
                && chunk_len != actions.len()
            {
                return Err(InvalidReply::ChunkLenMismatch {
                    chunk_len,
                    actions: actions.len(),
                });
            }
            actions
                .into_iter()
                .enumerate()
                .map(|(i, ChunkedAction::JointDelta(action))| {
                    joint_delta(&action, &format!("action.actions[{i}]"))
                })
                .collect()
        }
    }
}

fn joint_delta(action: &JointDelta, at: &str) -> Result<Action, InvalidReply> {
    reject_unknown(&action.unknown, &format!("{at}."))?;

    let delta =
        Joints::try_from(action.values.as_slice()).map_err(|_| InvalidReply::WrongValueCount {
            at: format!("{at}.values"),
            count: action.values.len(),
        })?;
    Ok(Action {
        delta: delta.map(|v| v.clamp(-MAX_DELTA, MAX_DELTA)),
        gripper: action.gripper.clamp(0.0, 1.0),
    })
}

const fn missing_chunk_len() -> Option<usize> {
    None
}

fn chunk_length<'de, D: serde::Deserializer<'de>>(d: D) -> Result<Option<usize>, D::Error> {
    usize::deserialize(d).map(Some)
}

fn reject_unknown(fields: &Map<String, Value>, prefix: &str) -> Result<(), InvalidReply> {
    fields.keys().next().map_or(Ok(()), |name| {
        Err(InvalidReply::UnknownField(format!("{prefix}{name}")))
    })
}

fn check_echo(at: &'static str, expected: &str, received: &str) -> Result<(), InvalidReply> {
    if expected == received {
        Ok(())
    } else {
        Err(InvalidReply::WrongEcho {
            at,
            expected: expected.to_owned(),
            received: received.to_owned(),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;
    use test_case::test_case;

    type Edit = fn(&mut Value, &mut EchoHeaders);

    fn delta(gripper: f64) -> Value {
        json!({ "type": "joint_delta", "values": [0.05, -0.05, 0.0, 0.01, 0.0, 0.0], "gripper": gripper })
    }

    fn chunk(len: usize) -> Value {
        json!({ "type": "action_chunk", "chunk_len": len, "actions": vec![delta(0.5); len] })
    }

    fn without(mut object: Value, field: &str) -> Value {
        if let Value::Object(fields) = &mut object {
            fields.remove(field);
        }
        object
    }

    fn validate_edited(edit: Edit) -> Result<Vec<Action>, InvalidReply> {
        let mut reply = json!({ "episode_id": "ep", "step": 7, "action": delta(1.0) });
        let mut headers = EchoHeaders {
            episode_id: Some("ep".to_owned()),
            step: Some("7".to_owned()),
        };
        edit(&mut reply, &mut headers);
        validate(&reply.to_string(), &headers, "ep", 7)
    }

    #[test_case(|_, _| () => matches Ok(a) if a.len() == 1; "joint delta")]
    #[test_case(|r, _| r["action"] = chunk(3) => matches Ok(a) if a.len() == 3; "chunk")]
    #[test_case(|r, _| r["action"] = chunk(16) => matches Ok(a) if a.len() == 16; "chunk of 16")]
    #[test_case(|r, _| { r["action"] = without(chunk(2), "chunk_len"); }
        => matches Ok(a) if a.len() == 2; "chunk without chunk_len")]
    #[test_case(|r, _| r["_debug"] = json!(2) => matches Ok(_); "extra response field")]
    #[test_case(|r, _| r["action"]["_debug"] = json!(2) => matches Err(InvalidReply::UnknownField(_)); "strict element")]
    #[test_case(|r, _| r["action"]["values"][0] = json!(2.0) => matches Ok(a) if a[0].delta[0] == 0.05; "positive clamp")]
    #[test_case(|r, _| r["action"]["values"][3] = json!(-2.0) => matches Ok(a) if a[0].delta[3] == -0.05; "negative clamp")]
    #[test_case(|r, _| r["action"]["gripper"] = json!(-1.0) => matches Ok(a) if a[0].gripper == 0.0; "gripper clamp")]
    #[test_case(|r, _| r["action"]["values"] = json!([0.0, 0.0, 0.0, 0.0, 0.0])
        => matches Err(InvalidReply::WrongValueCount { count: 5, .. }); "five values")]
    #[test_case(|r, _| r["action"]["values"] = json!([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        => matches Err(InvalidReply::WrongValueCount { count: 7, .. }); "seven values")]
    #[test_case(|r, _| r["action"]["values"][0] = json!(null) => matches Err(InvalidReply::Malformed(_)); "null value")]
    #[test_case(|r, _| r["action"]["values"][0] = json!("0.0") => matches Err(InvalidReply::Malformed(_)); "string value")]
    #[test_case(|r, _| { r["action"] = without(delta(1.0), "gripper"); }
        => matches Err(InvalidReply::Malformed(_)); "missing gripper")]
    #[test_case(|r, _| { r["action"] = without(delta(1.0), "values"); }
        => matches Err(InvalidReply::Malformed(_)); "missing values")]
    #[test_case(|r, _| r["extra"] = json!(1) => matches Ok(_); "unknown at top")]
    #[test_case(|r, _| r["action"]["speed"] = json!(1) => matches Err(InvalidReply::UnknownField(f)) if f == "action.speed"; "unknown in action")]
    #[test_case(|r, _| { r["action"] = chunk(2); r["action"]["actions"][0]["x"] = json!(1); }
        => matches Err(InvalidReply::UnknownField(f)) if f == "action.actions[0].x"; "unknown in chunk action")]
    #[test_case(|r, _| r["action"]["type"] = json!("cartesian") => matches Err(InvalidReply::Malformed(_)); "unknown action type")]
    #[test_case(|r, _| { r["action"] = chunk(1); r["action"]["actions"][0] = chunk(1); }
        => matches Err(InvalidReply::Malformed(_)); "chunk in a chunk")]
    #[test_case(|r, _| r["action"] = chunk(0) => matches Err(InvalidReply::ChunkSize(0)); "empty chunk")]
    #[test_case(|r, _| r["action"] = chunk(17) => matches Err(InvalidReply::ChunkSize(17)); "chunk of 17")]
    #[test_case(|r, _| { r["action"] = chunk(3); r["action"]["chunk_len"] = json!(2); }
        => matches Err(InvalidReply::ChunkLenMismatch { chunk_len: 2, actions: 3 }); "wrong chunk_len")]
    #[test_case(|r, _| r["episode_id"] = json!("other") => matches Err(InvalidReply::WrongEcho { at: "episode_id", .. }); "wrong episode_id")]
    #[test_case(|r, _| r["step"] = json!(8) => matches Err(InvalidReply::WrongEcho { at: "step", .. }); "wrong step")]
    #[test_case(|_, h| h.episode_id = Some("other".to_owned())
        => matches Err(InvalidReply::WrongEcho { at: "X-RC-Episode-Id", .. }); "wrong episode header")]
    #[test_case(|_, h| h.step = Some("8".to_owned()) => matches Err(InvalidReply::WrongEcho { at: "X-RC-Step", .. }); "wrong step header")]
    #[test_case(|_, h| h.step = None => matches Err(InvalidReply::WrongEcho { at: "X-RC-Step", .. }); "missing step header")]
    fn reply_validation(edit: Edit) -> Result<Vec<Action>, InvalidReply> {
        validate_edited(edit)
    }

    #[test]
    fn shared_wire_fixture() {
        use base64::{Engine, engine::general_purpose::STANDARD};
        let fixture: Value = serde_json::from_str(include_str!(
            "../../../docs/contract-fixtures/whistle-policy-v3.json"
        ))
        .unwrap();
        let sample = &fixture["request"];
        let episode = sample["episode_id"].as_str().unwrap();
        let step = sample["step"].as_u64().unwrap() as u32;
        let goal: Images = serde_json::from_value(sample["goal"].clone()).unwrap();
        let request = ActRequest {
            episode_id: episode,
            step,
            task_id: sample["task_id"].as_str().unwrap(),
            instruction: sample["instruction"].as_str().unwrap(),
            schema_version: crate::SCHEMA_VERSION,
            images: serde_json::from_value(sample["images"].clone()).unwrap(),
            goal: &goal,
            proprio: serde_json::from_value(sample["proprio"].clone()).unwrap(),
        };
        assert_eq!(serde_json::to_value(&request).unwrap(), *sample);
        for imgs in [&request.images, request.goal] {
            for image in [&imgs.wrist, &imgs.left, &imgs.right] {
                crate::image::check_policy_jpeg(&STANDARD.decode(image).unwrap()).unwrap();
            }
        }
        for case in fixture["cases"].as_array().unwrap() {
            let headers = EchoHeaders {
                episode_id: case["headers"]["X-RC-Episode-Id"]
                    .as_str()
                    .map(str::to_owned),
                step: case["headers"]["X-RC-Step"].as_str().map(str::to_owned),
            };
            let result = validate(&case["response"].to_string(), &headers, episode, step);
            assert_eq!(
                result.is_err(),
                case["hold"].as_bool().unwrap(),
                "{}",
                case["name"]
            );
            let actions = result.unwrap_or_else(|_| {
                vec![Action {
                    delta: [0.0; 6],
                    gripper: 0.75,
                }]
            });
            let normalized=actions.iter().map(|action|json!({"type":"joint_delta","values":action.delta,"gripper":action.gripper})).collect::<Vec<_>>();
            assert_eq!(json!(normalized), case["actions"], "{}", case["name"]);
        }
    }

    #[test]
    fn rejects_duplicate_keys_inside_ignored_fields() {
        let reply = r#"{"episode_id":"ep","step":7,"action":{"type":"joint_delta","values":[0,0,0,0,0,0],"gripper":1},"extra":{"a":1,"a":2}}"#;
        let headers = EchoHeaders {
            episode_id: Some("ep".into()),
            step: Some("7".into()),
        };
        std::assert_matches!(
            validate(reply, &headers, "ep", 7),
            Err(InvalidReply::Malformed(_))
        );
    }

    #[test]
    fn non_finite_number() {
        let body = r#"{"episode_id": "ep", "step": 7,
            "action": {"type": "joint_delta", "values": [1e400, 0, 0, 0, 0, 0], "gripper": 1}}"#;
        let headers = EchoHeaders {
            episode_id: Some("ep".to_owned()),
            step: Some("7".to_owned()),
        };
        std::assert_matches!(
            validate(body, &headers, "ep", 7),
            Err(InvalidReply::Malformed(_))
        );
    }
}
