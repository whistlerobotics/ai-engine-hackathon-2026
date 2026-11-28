# Current Spark workspace

This private repository preserves the Whistle Robotics history and the current Spark changes.
The active checkout is `/home/joe-spark-2/ai-engine-hackathon-2026`.

For the v3 integration, follow the [schema v3 mock workflow](ROS_DRIVER_SIMULATION.md#schema-v3-mock-run).
The commands below document the earlier schema v2 checkout.

Open a terminal on the Spark desktop (including through Moonlight) and run:

```bash
bash ~/run-rc-ros-sim.sh
```

The Rust terminal UI uses real right-camera frames and an isolated ROS mock arm
and gripper. All three camera feeds are relayed into domain 77; the real stack
stays on domain 42. This test uses the deterministic test policy, not Qwen.
Enter moves the mock arm to start; Enter again runs; Esc aborts; q quits while idle.
The terminal shows camera status, not a video preview.
To open the 3D robot and three live camera panels, use a second Spark terminal:

```bash
bash ~/run-rc-ros-rviz.sh
```

See [ROS driver simulation](ROS_DRIVER_SIMULATION.md).

The host scripts are preserved under `tools/spark/`. They describe the current
Spark setup and assume its existing Docker images, containers and checkout paths;
a fresh clone alone does not install these dependencies. The MuJoCo launcher
also references the separate PR7 checkout. BF16 Qwen code and downloaded weights
remain in `~/rc-qwen-visionik-policy` and `~/models/rc-qwen-visionik-policy`.
Weights, recordings and GitHub credentials are excluded from this repository.

The three-view translation layer supplies the instruction and goal image;
`tasks/` contains the sample simulation goal and the separately captured wrist goal.
Camera-to-base calibration is still unverified.
See [camera recording](THREE_CAMERA_RECORDING.md) and
[alignment notes](CAMERA_ALIGNMENT_DRAFT.md).

To replace the deterministic test policy with the existing BF16 Qwen CV/IK
policy while retaining the mock ROS arm and real cameras:

```bash
bash ~/run-rc-qwen-ros-sim.sh
```

Quit the existing mock runner with q while idle before opening the Qwen runner.
See the Qwen section in [ROS driver simulation](ROS_DRIVER_SIMULATION.md).

## Real-arm policy runner

Use the Spark ROS Kilted devcontainer on domain 42 with the cell operator.
Use Whistle's existing real ROS stack and cabinet configuration. If the
stack is already up, reuse it. The original bring-up command is:

```bash
ROS_DOMAIN_ID=42 mise run real-robot robot_ip:=192.168.56.101
```

Start the v3 policy separately. For a connection check before real goal photos
are available, use the bundled simulator pack. It has three rendered goal
views per public task. In a second terminal at the checkout root, run:

```bash
export ROS_DOMAIN_ID=42
export WHISTLE_TASK_PACK="$PWD/task-packs/simulator/manifest.json"
mise run policy-runner -- --task bolt_to_tray --policy http://whistle-policy:8080
```

The runner reads the wrist, left and right camera topics directly on domain
42 and sends those images with the selected task's instruction and three goal
images. Enter moves to the saved start pose, Enter starts the episode, and Esc
sends Whistle's hold command. Verify those controls with the cell
operator before running a policy on the physical arm.

For physical goal evaluation, set `WHISTLE_TASK_PACK` to the cell's manifest.
For each task, photograph the same finished arrangement from the wrist, left,
and right cameras. Save distinct 640×360 JPEGs as
`<task_id>/<variant>/{wrist,left,right}.jpg` next to that manifest and update
their paths and SHA-256 values. The runner loads the selected task's three
goal images once when the episode starts.

## Real-arm ROS preflight

From the Spark ROS Kilted environment that can see the existing real ROS domain,
run:

```bash
ROS_DOMAIN_ID=42 mise run real-arm-preflight
```

This checks for arm and gripper messages, the External Control program status,
active arm and gripper controllers, and frames from the three cameras. It exits
nonzero on missing or inactive inputs. It sends no arm or gripper commands.

A passing preflight reports a live ROS graph. Check the driver checksum and
physical stop and hold behavior at the cell.

## Paired demo recorder

Open the unconnected preview from the checkout root:

```bash
python3 tools/demo-tui.py --preview
```

Press **T** to choose a task from the manifest, **H** to simulate homing,
and **C** to arm the recorder. Preview uses **Space** to simulate the
physical button hold and release. Then **R** simulates home and replay,
**S** saves, and **D** discards the draft. Preview data goes under
`recording/demo-preview/`.

The TUI starts on its own. Bring up ROS, the robot driver, cameras, and
hand-guiding separately. Opening the TUI starts none of those services.
It starts the existing recording and replay tools only when you select
their actions.

For a supervised physical capture session, run this in the real ROS
devcontainer with the cell operator at the physical stop.

```bash
python3 tools/demo-tui.py --drive-replay
```

1. **T** selects the task. Clear the cell, release hand-guiding, and press
   **H**. The TUI uses the existing Rust runner to move to its home pose.
2. **C** arms the existing Rust recorder. Hold the physical hand-guiding
   button to record and release it to stage the Human take.
3. Clear the cell and press **R**. The TUI homes the arm again, starts the
   three-camera bag and Rust recorder, and runs the replay once both are ready.
   **Esc** or **Ctrl-C** requests a stop during the arm sequence.
4. **S** saves the pair after the Rust runner reports completion and the
   recordings finish. **R** repeats the No human take from the same Human take.

The physical runner uses its fixed home pose. Imported and freshly guided
takes must start within its home tolerance. If you use the cabinet's existing
replay controls, start the TUI in observer mode.

```bash
python3 tools/demo-tui.py --observe-replay
```

1. **T** selects the task. After the cell operator moves the arm to the
   approved start pose, **H** opens Whistle's start-pose check in the same
   terminal. The check returns to the demo TUI only after fresh joint
   feedback confirms the pose.
2. **C** arms Whistle's existing Rust recorder on domain 42. It waits for
   a fresh hold of the physical hand-guiding button and saves the Human
   take when the button is released. The default source is the named
   right camera at 640×360. Use `--camera-topic /camera/color/image_raw/compressed`
   for the original 640×480 camera.
3. **R** loads the Human take into Whistle's replay policy and prepares a
   second Rust recorder and the three-camera bag. The cell operator homes
   and starts replay using the existing controls and the displayed policy
   URL. The first replay action request starts the second Rust capture.
4. After the operator finishes replay, **F** ends capture. **S** saves once
   the Rust JSON and verified camera bag are ready.

Both takes keep Whistle's episode ID, for example `ep_20261128_123456`.
They are stored under `recording/<task_id>/<episode_id>/` as `human.json` and
`no_human.json`. The three-camera bag is in `no_human/` and timing and
provenance are in `demo.json`. The adjacent
`<episode_id>_human.json` link lets existing JSON replay and video extraction
tools read the Human take. **R** in review lets you redo the No human take
with the same Human take.

Pass `--input path/to/episode.json` to reuse a Human take with its existing task
and episode IDs. Use `--task-pack` or `WHISTLE_TASK_PACK` to select a different
manifest. For automated mock replay, open the TUI inside the isolated mock
container with `--input` and a writable `--out-dir` on the shared checkout.
It uses the existing `mise run test-policy` and `mise run policy-runner` tasks.
The mock No human take contains mock state and a replay plan, with no camera
recording.
