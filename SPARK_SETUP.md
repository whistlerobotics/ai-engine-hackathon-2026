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

## Real-arm ROS preflight

From the Spark ROS Kilted environment that can see the existing real ROS domain,
run:

```bash
ROS_DOMAIN_ID=42 mise run real-arm-preflight
```

This checks for arm and gripper messages, the External Control program status,
active arm and gripper controllers, and frames from the three cameras. It exits
nonzero on missing or inactive inputs. It sends no arm or gripper commands.

The real driver launch follows Whistle's existing calibration wiring. A prior
Spark driver startup reported a calibration checksum mismatch. Check the
driver's console with the cell operator before commissioning physical motion.

A passing preflight reports a live ROS graph. It does not verify the physical
stop or hold outcome, watchdog, or camera calibration. The v3 policy runner is
still mock-only. The cell operator must verify those behaviors before enabling
the policy command path on the physical arm.
