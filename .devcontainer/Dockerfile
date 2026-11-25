# rolling, kilted, jazzy
ARG ROS_DISTRO=kilted

FROM ros:${ROS_DISTRO}-ros-base

ARG USERNAME=USERNAME
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG ROS_DISTRO

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m --shell /bin/bash $USERNAME \
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y curl sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

RUN install -dm 755 /etc/apt/keyrings && \
    curl -fsSL https://mise.jdx.dev/gpg-key.pub -o /etc/apt/keyrings/mise-archive-keyring.asc && \
    echo "deb [signed-by=/etc/apt/keyrings/mise-archive-keyring.asc] https://mise.jdx.dev/deb stable main" \
        > /etc/apt/sources.list.d/mise.list

RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y --no-install-recommends \
        python-is-python3 \
        bash-completion ffmpeg git-lfs iproute2 iputils-ping jq mise ninja-build nmap rustup \
        usbutils vim zstd \
        ros-${ROS_DISTRO}-launch ros-${ROS_DISTRO}-ur ros-${ROS_DISTRO}-ros2-control \
        ros-${ROS_DISTRO}-ros2-controllers ros-${ROS_DISTRO}-rviz2 ros-${ROS_DISTRO}-xacro \
        # rclrs vendors these interface packages and links them unconditionally
        ros-${ROS_DISTRO}-example-interfaces ros-${ROS_DISTRO}-test-msgs \
        # OrbbecSDK_ROS2's build dependencies, as its README lists them
        libgflags-dev nlohmann-json3-dev libdw-dev libssl-dev mesa-utils libgoogle-glog-dev \
        ros-${ROS_DISTRO}-image-transport ros-${ROS_DISTRO}-image-transport-plugins \
        ros-${ROS_DISTRO}-compressed-image-transport ros-${ROS_DISTRO}-image-publisher \
        ros-${ROS_DISTRO}-camera-info-manager ros-${ROS_DISTRO}-diagnostic-updater \
        ros-${ROS_DISTRO}-diagnostic-msgs ros-${ROS_DISTRO}-statistics-msgs \
        ros-${ROS_DISTRO}-backward-ros && \
    apt-get autoremove -y && \
    rm -rf /var/lib/apt/lists/*

# The mounted host `~/.ssh` is read-only in the container, so new host keys go to
# a container-local file; the host's known_hosts is still consulted for already-trusted hosts.
RUN printf 'Host *\n\
    StrictHostKeyChecking accept-new\n\
    UserKnownHostsFile /home/rosdev/.ssh_known_hosts /home/rosdev/.ssh/known_hosts\n' \
    >> /etc/ssh/ssh_config

# Build-time: `source` in the RUN steps below is a bashism; dash lacks it.
SHELL ["/bin/bash", "-c"]
# Run-time: terminal emulators pick the shell from $SHELL.
ENV SHELL="/bin/bash"
USER $USERNAME

# `minimal` profile saves us from installing Rust docs
# that bloat the container image, but are also available online.
# Keep in step with rust-toolchain.toml.
RUN rustup toolchain install 1.100 --profile minimal --component clippy,rustfmt && \
    rustup default 1.100

RUN echo 'eval "$(mise activate bash)"' >> ~/.bashrc && \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# Silences uv's hardlink warning: its cache and the mounted workspace are on different filesystems.
ENV UV_LINK_MODE=copy

# colcon copies into `install`, so each workspace deletes its sources in the RUN that builds it.

# Kilted's ros2_robotiq_gripper release lacks the 2F-140's description and the driver, and its
# tests need linters the image doesn't have. `git clone --depth 1` can't check out a SHA.
RUN clone_commit() { \
        git init --quiet "$3" && \
        git -C "$3" fetch --quiet --depth 1 "$1" "$2" && \
        git -C "$3" checkout --quiet FETCH_HEAD; \
    } && \
    mkdir -p ~/robotiq_ws/src && cd ~/robotiq_ws/src && \
    clone_commit https://github.com/PickNikRobotics/ros2_robotiq_gripper.git \
        a5e1405098166dd9bd63fd24b3c03173fc1cb984 ros2_robotiq_gripper && \
    clone_commit https://github.com/tylerjw/serial.git \
        d8d160678aa0b31cdf467c052b954fa287cc6cdf serial && \
    cd ~/robotiq_ws && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --packages-up-to robotiq_description robotiq_driver \
        --cmake-args -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=OFF && \
    rm -rf build log src && \
    echo "source ~/robotiq_ws/install/local_setup.sh" >> ~/.bashrc

# OrbbecSDK_ROS2 isn't released for Kilted.
RUN mkdir -p ~/orbbec_ws/src && cd ~/orbbec_ws/src && \
    git clone --depth 1 --branch v2.10.7 https://github.com/orbbec/OrbbecSDK_ROS2.git && \
    cd ~/orbbec_ws && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --cmake-args -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF && \
    rm -rf build log src && \
    echo "source ~/orbbec_ws/install/local_setup.sh" >> ~/.bashrc

# Shared by rosdev and by CI, which runs as root with its own HOME.
ENV MISE_DATA_DIR=/opt/mise
# For shells that don't run `mise activate`, such as CI's and editors'.
ENV PATH="${MISE_DATA_DIR}/shims:${PATH}"
ENV MISE_TRUSTED_CONFIG_PATHS=/home/${USERNAME}/ros2_ws
COPY mise-tools.toml /etc/mise/config.toml
# The dev container remaps the user's UID at start. Same RUN, or chmod duplicates the layer.
RUN sudo install -d -o $USERNAME -g $USERNAME $MISE_DATA_DIR && \
    mise install && \
    chmod -R a+rwX $MISE_DATA_DIR && \
    rm -rf ~/.cache/mise

CMD ["/bin/bash"]
