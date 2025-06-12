# ベースイメージをROS2 Humble Desktopにする
FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

# 基本ツールと、ROSのビルドツール、そして証明書などを追加
# FlexBEに直接必要だったものは一旦コメントアウト、必要なら後で追加する
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
    libxkbcommon-x11-0 \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-randr0 \
    libxcb-render-util0 \
    libxcb-xinerama0 \
    libxcb-cursor0 \
    && rm -rf /var/lib/apt/lists/* \
    gazebo \
    ros-humble-gazebo-* \
    ros-humble-rqt-* \
    ros-humble-nav2-behavior-tree \
    && rm -rf /var/lib/apt/lists/*

# rosdepの初期化
RUN rm -rf /etc/ros/rosdep/sources.list.d/20-default.list && rosdep init && rosdep update

# === ユーザー設定 ===
RUN useradd -m -s /bin/bash dockeruser && echo "dockeruser:yakiniku" | chpasswd && adduser dockeruser sudo
USER dockeruser
WORKDIR /home/dockeruser
ENV XDG_CONFIG_HOME=/tmp/.chromium
ENV XDG_CACHE_HOME=/tmp/.chromium


# === エントリーポイント設定 ===
# entrypoint.shはコンテナ起動時にD-Busを起動するために、そのまま使う
COPY --chown=dockeruser:dockeruser --chmod=755 docker/entrypoint.sh /home/dockeruser/entrypoint.sh
ENTRYPOINT ["/home/dockeruser/entrypoint.sh"]
CMD ["bash"]