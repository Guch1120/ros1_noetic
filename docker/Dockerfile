FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    wget \
    nano \
    curl \
    lsb-release \
    sudo \
    gnupg2 \
    python3-pip \
    ros-dev-tools \
    terminator \
    tree \
    x11-apps \
    dbus-x11 \
    ca-certificates \
    libcanberra-gtk3-module \
    libxkbcommon-x11-0 \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-randr0 \
    libxcb-render-util0 \
    libxcb-xinerama0 \
    libxcb-cursor0 \
    && rm -rf /var/lib/apt/lists/*

RUN rm -f /etc/ros/rosdep/sources.list.d/20-default.list && rosdep init && rosdep update

RUN useradd -m -s /bin/bash dockeruser && echo "dockeruser:yakiniku" | chpasswd && adduser dockeruser sudo
USER dockeruser
WORKDIR /home/dockeruser

COPY --chown=dockeruser:dockeruser --chmod=755 docker/entrypoint.sh /home/dockeruser/entrypoint.sh
ENTRYPOINT ["/home/dockeruser/entrypoint.sh"]
CMD ["bash"]