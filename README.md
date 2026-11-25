# Policy Runner for the AI Engine Reality Check Hackathon

28–29 November 2026, London

Check the [hackathon website](https://www.aiengine.build/reality-check-hack) for more details.

This repo contains a policy runner and ROS 2 stack configuration for the hackathon test stand: a
UR12e with a Robotiq 2F-140 gripper and an Orbbec Gemini 336 camera. The policy's API is in
[API.md](API.md).

## Devcontainer setup

We use [devcontainers](https://containers.dev) as development environment to unify everyone's
development setup and also to avoid ROS turning our package installations into unrepairable mess.

The dev container expects a Linux host with a Wayland session. It mounts the host's `~/.gitconfig`
and `~/.ssh` read-only, so both must exist. Open the workspace in VS Code and run
**Dev Containers: Rebuild and Reopen in Container**.

The container sees the host's USB devices, for the Orbbec camera. To be allowed to open the camera,
install Orbbec's udev rules on the host, then plug the camera in again:

```bash
sudo curl -fsSL -o /etc/udev/rules.d/99-obsensor-libusb.rules \
  https://raw.githubusercontent.com/orbbec/OrbbecSDK_ROS2/v2.10.7/orbbec_camera/scripts/99-obsensor-libusb.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Inside the container, `lsusb` should then list the camera.

## Running tasks

Tasks are run with [mise](https://mise.jdx.dev). Run `mise tasks` to see all tasks and what they
do. Extra arguments go through to the launch file or the program, e.g. `mise run mock-robot rviz:=true`.

### On mock hardware

In three terminals:

```bash
mise run mock-robot
mise run test-policy
mise run policy-runner -- --task <task_id>
```

`mock-robot` brings up the UR12e and the gripper on mock hardware. `test-policy` serves a stand-in
policy on port 8000; `mise run test-policy -- --help` lists the replies it can send, including late,
chunked and invalid ones. The policy runner is a terminal UI that runs episodes of the policy
against the arm; it sends a black frame unless given `--camera <topic>`.

### On the real robot

```bash
mise run real-robot robot_ip:=<address>
mise run policy-runner -- --task <task_id> --policy <url> --camera /camera/color/image_raw/compressed
```

By default `real-robot` sends the driver's program on connect, which needs the arm in Remote Control
mode; with `headless_mode:=false`, play External Control on the pendant in Local mode instead. It
also starts the Gemini 336 unless given `camera:=false`.

### Collecting data

```bash
mise run real-robot freedrive:=true
mise run record -- --task <task_id>
```

The thumb throttle, which includes a throttle and a self-holding red button at the wrist, enables the start and the stop of the recording and the gripper position control. When the button is pressed down after running the command, it starts the recording. Turn the throttle to control the angle to which the gripper opens. When the button is pressed again and goes to the top position, the recording is saved to `recording/<task_id>/<episode_id>.json` and the gripper control is disabled.

The maximum time length for each task is capped at 5mins, in order to prevent filling up the disk space if recording never ends.