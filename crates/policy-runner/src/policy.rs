//! The policy server, over HTTP.

use color_eyre::eyre::ensure;
use policy_contract::{
    SCHEMA_VERSION,
    wire::{ActRequest, Action, EchoHeaders, InvalidReply, validate},
};
use reqwest::{Client, Url};
use serde::Deserialize;
use std::{fmt, time::Duration};
use tokio::runtime::Runtime;

const CONTRACT: &str = "v0";
const CONNECT_TIMEOUT: Duration = Duration::from_secs(5);
const STATUS_TIMEOUT: Duration = Duration::from_secs(1);

#[derive(Debug, thiserror::Error)]
pub enum ActError {
    #[error("policy connection failed: {0:#}")]
    Transport(#[from] color_eyre::Report),
    #[error("invalid reply")]
    Invalid(#[from] InvalidReply),
}

#[derive(Deserialize)]
struct Readyz {
    ready: bool,
    contract: Option<String>,
    schema_version: Option<u32>,
}

/// `/healthz`, as last polled.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Health {
    Up,
    Unreachable(String),
    HttpStatus(u16),
}

/// `/readyz`, as last polled.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Readiness {
    Ready,
    NotReady,
    Incompatible {
        contract: Option<String>,
        schema_version: Option<u32>,
    },
    Malformed(String),
    Unreachable(String),
    HttpStatus(u16),
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PolicyStatus {
    pub health: Health,
    pub readiness: Readiness,
}

impl fmt::Display for Health {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Up => write!(f, "up"),
            Self::Unreachable(reason) => write!(f, "unreachable ({reason})"),
            Self::HttpStatus(status) => write!(f, "HTTP status {status}"),
        }
    }
}

impl fmt::Display for Readiness {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Ready => write!(f, "ready ({CONTRACT}, schema version {SCHEMA_VERSION})"),
            Self::NotReady => write!(f, "not ready"),
            Self::Incompatible {
                contract,
                schema_version,
            } => {
                let contract = contract.as_deref().unwrap_or("missing");
                let schema_version =
                    schema_version.map_or_else(|| "missing".to_owned(), |v| v.to_string());
                write!(
                    f,
                    "incompatible: contract {contract}, schema version {schema_version}, \
                     not {CONTRACT}, {SCHEMA_VERSION}"
                )
            }
            Self::Malformed(reason) => write!(f, "malformed reply ({reason})"),
            Self::Unreachable(reason) => write!(f, "unreachable ({reason})"),
            Self::HttpStatus(status) => write!(f, "HTTP status {status}"),
        }
    }
}

/// A reply's status and body, or why there was none.
type HttpReply = Result<(u16, String), color_eyre::Report>;

impl From<HttpReply> for Health {
    fn from(reply: HttpReply) -> Self {
        match reply {
            Ok((200, _)) => Self::Up,
            Ok((status, _)) => Self::HttpStatus(status),
            Err(error) => Self::Unreachable(format!("{error:#}")),
        }
    }
}

impl From<HttpReply> for Readiness {
    fn from(reply: HttpReply) -> Self {
        match reply {
            Ok((200, body)) => match serde_json::from_str::<Readyz>(&body) {
                Ok(Readyz { ready: false, .. }) => Self::NotReady,
                Ok(Readyz {
                    ready: true,
                    contract,
                    schema_version,
                }) => {
                    if contract.as_deref() == Some(CONTRACT)
                        && schema_version == Some(SCHEMA_VERSION)
                    {
                        Self::Ready
                    } else {
                        Self::Incompatible {
                            contract,
                            schema_version,
                        }
                    }
                }
                Err(error) => Self::Malformed(error.to_string()),
            },
            Ok((status, _)) => Self::HttpStatus(status),
            Err(error) => Self::Unreachable(format!("{error:#}")),
        }
    }
}

pub struct Policy {
    pub runtime: Runtime,
    client: Client,
    url: Url,
}

impl Policy {
    pub fn new(url: &str) -> color_eyre::Result<Self> {
        let url = Url::parse(url)?;
        ensure!(
            url.scheme() == "http"
                && url.has_host()
                && url.username().is_empty()
                && url.password().is_none()
                && url.path() == "/"
                && url.query().is_none()
                && url.fragment().is_none(),
            "--policy must be an HTTP origin without credentials or a path"
        );
        let client = Client::builder()
            .no_proxy()
            .redirect(reqwest::redirect::Policy::none())
            .retry(reqwest::retry::never())
            .connect_timeout(CONNECT_TIMEOUT)
            .build()?;
        let runtime = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()?;
        Ok(Self {
            runtime,
            client,
            url,
        })
    }

    pub fn status(&self) -> PolicyStatus {
        self.runtime.block_on(async {
            PolicyStatus {
                health: self.get("/healthz").await.into(),
                readiness: self.get("/readyz").await.into(),
            }
        })
    }

    async fn get(&self, path: &str) -> HttpReply {
        let response = self
            .client
            .get(self.url.join(path)?)
            .timeout(STATUS_TIMEOUT)
            .send()
            .await?;
        let status = response.status().as_u16();
        Ok((
            status,
            String::from_utf8(body_bytes(response, 64 * 1024).await?)?,
        ))
    }

    pub async fn act(
        &self,
        request: &ActRequest<'_>,
        remaining: Duration,
    ) -> Result<Vec<Action>, ActError> {
        let response = self
            .client
            .post(self.url.join("/act").map_err(color_eyre::Report::new)?)
            .header("X-RC-Episode-Id", request.episode_id)
            .header("X-RC-Step", request.step)
            .timeout(remaining)
            .json(request)
            .send()
            .await
            .map_err(color_eyre::Report::new)?;
        let status = response.status().as_u16();
        if status != 200 {
            return Err(InvalidReply::Status(status).into());
        }
        let header = |name: &str| {
            let mut values = response.headers().get_all(name).iter();
            let first = values.next()?.to_str().ok()?;
            values.next().is_none().then(|| first.to_owned())
        };
        let headers = EchoHeaders {
            episode_id: header("X-RC-Episode-Id"),
            step: header("X-RC-Step"),
        };
        let bytes = body_bytes(response, 2 * 1024 * 1024).await?;
        let body = std::str::from_utf8(&bytes).map_err(color_eyre::Report::new)?;
        Ok(validate(body, &headers, request.episode_id, request.step)?)
    }
}

async fn body_bytes(mut response: reqwest::Response, limit: usize) -> color_eyre::Result<Vec<u8>> {
    let mut bytes = Vec::new();
    while let Some(part) = response.chunk().await? {
        ensure!(
            bytes.len() + part.len() <= limit,
            "policy reply exceeds {limit} byte limit"
        );
        bytes.extend_from_slice(&part);
    }
    Ok(bytes)
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::{Value, json};
    use test_case::test_case;

    fn ok(body: &Value) -> (u16, String) {
        (200, body.to_string())
    }

    #[test_case(Ok(ok(&json!({"ready": true, "contract": "v0", "schema_version": 3}))) => Readiness::Ready; "ready")]
    #[test_case(Ok(ok(&json!({"ready": false}))) => Readiness::NotReady; "not ready")]
    #[test_case(Ok(ok(&json!({"ready": true, "contract": "v1", "schema_version": 2})))
        => Readiness::Incompatible { contract: Some("v1".to_owned()), schema_version: Some(2) };
        "wrong contract")]
    #[test_case(Ok(ok(&json!({"ready": true, "contract": "v0", "schema_version": 2})))
        => Readiness::Incompatible { contract: Some("v0".to_owned()), schema_version: Some(2) };
        "wrong schema version")]
    #[test_case(Ok(ok(&json!({"ready": true})))
        => Readiness::Incompatible { contract: None, schema_version: None };
        "missing fields")]
    #[test_case(Ok((503, String::new())) => Readiness::HttpStatus(503); "non-200")]
    #[test_case(Ok((200, "<html>".to_owned())) => matches Readiness::Malformed(_); "malformed")]
    #[test_case(Err(color_eyre::eyre::eyre!("connection failed")) => matches Readiness::Unreachable(_); "unreachable")]
    fn readiness_of(reply: HttpReply) -> Readiness {
        Readiness::from(reply)
    }

    #[test_case(Ok(ok(&json!({}))) => Health::Up; "up")]
    #[test_case(Ok((500, String::new())) => Health::HttpStatus(500); "non-200")]
    #[test_case(Err(color_eyre::eyre::eyre!("connection failed")) => matches Health::Unreachable(_); "unreachable")]
    fn health_of(reply: HttpReply) -> Health {
        Health::from(reply)
    }
}

#[cfg(test)]
mod http_tests {
    use super::*;
    use policy_contract::wire::Images;
    use serde_json::Value;
    use std::{
        io::{BufRead, BufReader},
        path::Path,
        process::{Child, Command, Stdio},
    };

    struct Peer {
        process: Child,
        port: u16,
    }

    impl Peer {
        fn start() -> Self {
            let script = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/http_fixture.py");
            let mut process = Command::new("python3")
                .arg(script)
                .stdin(Stdio::piped())
                .stdout(Stdio::piped())
                .spawn()
                .unwrap();
            let mut line = String::new();
            BufReader::new(process.stdout.take().unwrap())
                .read_line(&mut line)
                .unwrap();
            let port = line.trim().parse().unwrap();
            Self { process, port }
        }

        fn client(&self) -> Policy {
            Policy::new(&format!("http://127.0.0.1:{}", self.port)).unwrap()
        }
    }

    impl Drop for Peer {
        fn drop(&mut self) {
            drop(self.process.stdin.take());
            let _ = self.process.wait();
        }
    }

    #[test]
    fn direct_http_and_bounded_calls_without_retry() {
        let peer = Peer::start();
        let client = peer.client();
        assert_eq!(
            client.status(),
            PolicyStatus {
                health: Health::Up,
                readiness: Readiness::Ready
            }
        );
        let sample: Value = serde_json::from_str(include_str!(
            "../../../docs/contract-fixtures/whistle-policy-v3-request.json"
        ))
        .unwrap();
        let goals: Images = serde_json::from_value(sample["goal"].clone()).unwrap();
        let mut request = ActRequest {
            episode_id: "http-test",
            step: 7,
            task_id: "bolt_to_tray",
            instruction: sample["instruction"].as_str().unwrap(),
            schema_version: SCHEMA_VERSION,
            images: serde_json::from_value(sample["images"].clone()).unwrap(),
            goal: &goals,
            proprio: serde_json::from_value(sample["proprio"].clone()).unwrap(),
        };
        let actions = client
            .runtime
            .block_on(client.act(&request, Duration::from_secs(1)))
            .unwrap();
        assert_eq!(actions[0].delta, [0.04; 6]);
        request.task_id = "slow";
        let timeout = client
            .runtime
            .block_on(client.act(&request, Duration::from_millis(100)));
        std::assert_matches!(timeout, Err(ActError::Transport(_)));
        let (_, body) = client.runtime.block_on(client.get("/healthz")).unwrap();
        assert_eq!(serde_json::from_str::<Value>(&body).unwrap()["calls"], 2);
    }
}
