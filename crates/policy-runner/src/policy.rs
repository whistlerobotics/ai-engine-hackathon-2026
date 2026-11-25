//! The policy server, over HTTP.

use policy_contract::{
    SCHEMA_VERSION,
    wire::{ActRequest, Action, EchoHeaders, InvalidReply, validate},
};
use serde::Deserialize;
use std::{fmt, time::Duration};

const CONTRACT: &str = "v0";
pub const ACT_TIMEOUT: Duration = Duration::from_millis(100);
const STATUS_TIMEOUT: Duration = Duration::from_secs(1);

#[derive(Debug, thiserror::Error)]
pub enum ActError {
    /// A timeout or a connection error: worth sending again.
    #[error(transparent)]
    Transient(#[from] ureq::Error),
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
type HttpReply = Result<(u16, String), ureq::Error>;

impl From<HttpReply> for Health {
    fn from(reply: HttpReply) -> Self {
        match reply {
            Ok((200, _)) => Self::Up,
            Ok((status, _)) => Self::HttpStatus(status),
            Err(error) => Self::Unreachable(error.to_string()),
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
            Err(error) => Self::Unreachable(error.to_string()),
        }
    }
}

pub struct Policy {
    agent: ureq::Agent,
    url: String,
}

impl Policy {
    pub fn new(url: &str) -> Self {
        let agent = ureq::Agent::config_builder()
            .http_status_as_error(false)
            // A proxy from the environment would sit between us and a policy on the local network.
            .proxy(None)
            .build()
            .new_agent();
        Self {
            agent,
            url: url.trim_end_matches('/').to_owned(),
        }
    }

    /// Polls `/healthz` and `/readyz` once each.
    pub fn status(&self) -> PolicyStatus {
        PolicyStatus {
            health: self.get("/healthz").into(),
            readiness: self.get("/readyz").into(),
        }
    }

    fn get(&self, path: &str) -> HttpReply {
        let mut response = self
            .agent
            .get(format!("{}{path}", self.url))
            .config()
            .timeout_global(Some(STATUS_TIMEOUT))
            .build()
            .call()?;
        let status = response.status().as_u16();
        Ok((status, response.body_mut().read_to_string()?))
    }

    pub fn act(&self, request: &ActRequest<'_>) -> Result<Vec<Action>, ActError> {
        let mut response = self
            .agent
            .post(format!("{}/act", self.url))
            .config()
            .timeout_global(Some(ACT_TIMEOUT))
            .build()
            .send_json(request)?;

        let status = response.status().as_u16();
        let header = |name: &str| {
            response
                .headers()
                .get(name)
                .and_then(|value| value.to_str().ok())
                .map(str::to_owned)
        };
        let headers = EchoHeaders {
            episode_id: header("X-RC-Episode-Id"),
            step: header("X-RC-Step"),
        };
        let body = response.body_mut().read_to_string()?;
        if status != 200 {
            return Err(InvalidReply::Status(status).into());
        }
        Ok(validate(&body, &headers, request.episode_id, request.step)?)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::{Value, json};
    use test_case::test_case;

    fn ok(body: &Value) -> (u16, String) {
        (200, body.to_string())
    }

    #[test_case(Ok(ok(&json!({"ready": true, "contract": "v0", "schema_version": 2}))) => Readiness::Ready; "ready")]
    #[test_case(Ok(ok(&json!({"ready": false}))) => Readiness::NotReady; "not ready")]
    #[test_case(Ok(ok(&json!({"ready": true, "contract": "v1", "schema_version": 2})))
        => Readiness::Incompatible { contract: Some("v1".to_owned()), schema_version: Some(2) };
        "wrong contract")]
    #[test_case(Ok(ok(&json!({"ready": true, "contract": "v0", "schema_version": 3})))
        => Readiness::Incompatible { contract: Some("v0".to_owned()), schema_version: Some(3) };
        "wrong schema version")]
    #[test_case(Ok(ok(&json!({"ready": true})))
        => Readiness::Incompatible { contract: None, schema_version: None };
        "missing fields")]
    #[test_case(Ok((503, String::new())) => Readiness::HttpStatus(503); "non-200")]
    #[test_case(Ok((200, "<html>".to_owned())) => matches Readiness::Malformed(_); "malformed")]
    #[test_case(Err(ureq::Error::ConnectionFailed) => matches Readiness::Unreachable(_); "unreachable")]
    fn readiness_of(reply: HttpReply) -> Readiness {
        Readiness::from(reply)
    }

    #[test_case(Ok(ok(&json!({}))) => Health::Up; "up")]
    #[test_case(Ok((500, String::new())) => Health::HttpStatus(500); "non-200")]
    #[test_case(Err(ureq::Error::ConnectionFailed) => matches Health::Unreachable(_); "unreachable")]
    fn health_of(reply: HttpReply) -> Health {
        Health::from(reply)
    }
}
