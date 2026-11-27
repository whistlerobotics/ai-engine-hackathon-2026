# Current Spark workspace

This private repository preserves the Whistle Robotics history and the current Spark changes.
The active checkout is `/home/joe-spark-2/ai-engine-hackathon-2026`.

Open a terminal on the Spark desktop (including through Moonlight) and run:

```bash
bash ~/run-rc-ros-sim.sh
```

The Rust terminal UI uses real right-camera frames and an isolated ROS mock arm
and gripper. All three camera feeds are relayed into domain 77; the real stack
stays on domain 42. This test uses the deterministic test policy, not Qwen.
Enter moves the mock arm to start; Enter again runs; Esc aborts; q quits while idle.
The terminal shows camera status, not a video preview.
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
