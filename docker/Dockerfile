# ベースイメージは君が選んだもの
FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

# 君が書いたパッケージインストール
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
    ros-humble-gazebo-* \
    ros-humble-rqt-* \
    apt-transport-https \
    software-properties-common  \
    && rm -rf /var/lib/apt/lists/*

# 君が書いたrealsense2のセットアップ
RUN mkdir -p /etc/apt/keyrings && \
    curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | tee /etc/apt/keyrings/librealsense.pgp > /dev/null
RUN echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/librealsense.list

# --- ここを少し変更 ---
# realsenseと一緒に、rtabmapもインストールする
RUN apt-get update && apt-get install -y \
    librealsense2-utils \
    librealsense2-dev \
    librealsense2-dbg \
    ros-humble-realsense2-camera \
    ros-humble-rtabmap-ros \
    && rm -rf /var/lib/apt/lists/*
# --- 変更ここまで ---

RUN rm -f /etc/ros/rosdep/sources.list.d/20-default.list && rosdep init && rosdep update

# 君が書いたユーザー設定
RUN useradd -m -s /bin/bash dockeruser && echo "dockeruser:yakiniku" | chpasswd && adduser dockeruser sudo
RUN groupadd -g 110 render || true && usermod -aG video,render dockeruser
USER dockeruser
WORKDIR /home/dockeruser

# --- ここから追加 ---
# ワークスペースと設定ファイル用のディレクトリを作成
RUN mkdir -p /home/dockeruser/ros2_ws/src/my_robot_configs/launch
RUN mkdir -p /home/dockeruser/ros2_ws/src/my_robot_configs/config

COPY --chown=dockeruser:dockeruser scripts/my_realsense.launch.py /home/dockeruser/ros2_ws/src/my_robot_configs/launch/
COPY --chown=dockeruser:dockeruser src/rtabmap_params.yaml /home/dockeruser/ros2_ws/src/my_robot_configs/config/
# 君が書いたエントリーポイント
COPY --chown=dockeruser:dockeruser --chmod=755 docker/entrypoint.sh /home/dockeruser/entrypoint.sh
ENTRYPOINT ["/home/dockeruser/entrypoint.sh"]
CMD ["bash"]