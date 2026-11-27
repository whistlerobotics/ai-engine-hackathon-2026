# Policy HTTP contract v0, schema version 3

The runner connects directly to the contestant's HTTPS server. The server name in
`--policy` must match the certificate's DNS subject alternative name. The runner
trusts the PEM CA supplied as `--ca-cert` and verifies the certificate chain,
server name and expiry.

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
Optional integer `chunk_len` must equal the array length. Unknown response and
chunk wrapper keys are ignored, except that `values` or `gripper` on a chunk
wrapper makes it malformed. Action elements accept exactly `type`, `values`,
and `gripper`.

Finite deltas are clamped to ±0.05 rad and added to the commanded joint target.
The gripper target is absolute and clamped to [0, 1]. Mock motion also follows
Reality Check's joint position bounds (±6.28319 rad, with the elbow at
±2.792526803190927 rad) and scales steps that would exceed 1.5 m/s at the tool
center point or elbow. The whole reply is
validated before movement. A malformed delivered response, including a bad echo
or any malformed chunk element, becomes one zero-delta step with the previous
gripper target. Transport failure or cancellation stops the run. Stop and
invalid feedback discard unapplied actions.

Each element consumes one 50 ms action step, up to 1,200 steps. Cumulative
`/act` transport is limited to 90 seconds, with a connection timeout at most
5 seconds and no retries. Time waiting for a response does not consume steps.
Mock runs have no manipulation success signal.

The [shared fixture](docs/contract-fixtures/whistle-policy-v3.json) is identical to
Reality Check's fixture. It includes a complete observation and normalization and
hold cases. The fixture images are synthetic. The deployed task pack contains
the actual goal images for each supported run context.
