# Policy HTTP contract v0, schema version 3

The runner connects directly to a policy server over HTTP on the
private container network. Set `--policy` to its HTTP origin.

- `GET /healthz` returns 200 once the server responds.
- `GET /readyz` returns 200 with `{"ready":true,"contract":"v0","schema_version":3}`
  after the policy loads. Both versions must match before starting an episode.
- `POST /act` receives the envelope below on every call.

| Field | Value |
| --- | --- |
| `schema_version` | JSON integer 3 |
| `episode_id`, `step` | An opaque string and the zero-based action step |
| `task_id`, `instruction` | Policy-facing identifier and natural-language instruction for the same task |
| `images` | Current `wrist`, `left` and `right` images |
| `goal` | Finished scene `wrist`, `left` and `right` images from the selected variant. A policy can ignore them. |
| `proprio.joint_pos`, `proprio.joint_vel` | Shoulder pan, shoulder lift, elbow, wrists 1–3, in radians and radians/second |
| `proprio.gripper` | Measured opening, 0 closed and 1 fully open |
| `proprio.ee_pos`, `proprio.ee_quat` | Nominal palm pose in robot `base_link`, in metres and unit wxyz quaternion, with palm +Z pointing along approach and +Y along finger opening |

All six views are standard-base64 RGB 640×360 JPEGs. The goal is constant throughout
the episode. Organizer task IDs are sent only as their policy-facing aliases 1–4.

The response body contains `episode_id`, `step` and `action`. Echo the first two
fields in both the body and the `X-RC-Episode-Id` and `X-RC-Step` response headers.
An action is one `{"type":"joint_delta","values":[0,0,0,0,0,0],"gripper":1}` or
`{"type":"action_chunk","actions":[...]}` with 1–16 such elements.
Optional integer `chunk_len` must equal the array length. Whistle uses its
existing reply validator: unknown fields must begin with `_`, each joint delta
must be finite and within ±0.05 rad, and gripper opening must be in [0, 1].
The runner rejects an invalid reply before applying any element of the chunk.

Reality Check's scoring runner and engine own clamping, joint target bounds,
and tool and elbow speed scaling. Send in-range values for a policy that runs
on both paths. Whistle sends each accepted chunk as one timed ROS trajectory
and stops the episode on an invalid reply.

Each element consumes one 50 ms action step, up to 1,200 steps. Cumulative
`/act` transport is limited to 90 seconds, with a connection timeout at most
5 seconds and no retries. Time waiting for a response does not consume steps.
Mock runs have no manipulation success signal.

The [request fixture](docs/contract-fixtures/whistle-policy-v3-request.json)
matches the request in Reality Check's fixture. Its six images are synthetic.
The mock task pack provides the goal images used during episodes.
