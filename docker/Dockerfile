# ベースイメージをROS2 Humble Desktopにする
FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

# 基本ツールと、BehaviorTree、そして証明書を追加
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
    gir1.2-keybinder-3.0 \
    gir1.2-notify-0.7 \
    at-spi2-core \
    librsvg2-common \
    adwaita-icon-theme-full \
    yaru-theme-icon \
    gazebo \
    ros-humble-gazebo-* \
    ros-humble-rqt-* \
    ros-humble-nav2-behavior-tree \
    && rm -rf /var/lib/apt/lists/*

# rosdepの更新処理のみ行う
RUN rosdep update

# === FlexBEの導入処理 ===
# ワークスペースの作成
RUN mkdir -p /opt/ros2_ws/src

# ワークスペースのsrcディレクトリに移動
WORKDIR /opt/ros2_ws/src

# 必要なFlexBEリポジトリを、それぞれ正しいhumbleブランチを指定してクローン
RUN git clone -b humble https://github.com/FlexBE/flexbe_behavior_engine.git
RUN git clone -b humble https://github.com/FlexBE/flexbe_app.git

# ワークスペースのルートに移動
WORKDIR /opt/ros2_ws

# 依存関係のインストール、ビルド、そしてnwjsの手動インストール
RUN ["/bin/bash", "-c", \
    "set -e && \
    apt-get update && \
    . /opt/ros/humble/setup.bash && \
    cd /opt/ros2_ws && \
    rosdep install --from-paths src --ignore-src -r -y --skip-keys \"actionlib genpy catkin rospack rostest flexbe_app flexbe_behavior_engine flexbe_input flexbe_mirror flexbe_msgs flexbe_onboard flexbe_states flexbe_testing flexbe_widget\" && \
    colcon build --symlink-install && \
    . install/setup.bash && \
    cd /tmp && \
    NWJS_VERSION=v0.75.1 && \
    NWJS_FILENAME=nwjs-sdk-${NWJS_VERSION}-linux-x64 && \
    wget https://dl.nwjs.io/${NWJS_VERSION}/${NWJS_FILENAME}.tar.gz && \
    tar -xzf ${NWJS_FILENAME}.tar.gz && \
    # ↓ ディレクトリを先に作るのを忘れてたから、この一行を追加するよ
    mkdir -p /opt/ros2_ws/install/flexbe_app/lib/flexbe_app/nwjs && \
    mv ${NWJS_FILENAME}/* /opt/ros2_ws/install/flexbe_app/lib/flexbe_app/nwjs/ && \
    chown -R 1000:1000 /opt/ros2_ws/install/flexbe_app/lib/flexbe_app/nwjs && \
    rm -rf /tmp/*" \
]

# === ユーザー設定 ===
RUN useradd -m -s /bin/bash dockeruser && echo "dockeruser:yakiniku" | chpasswd && adduser dockeruser sudo
RUN mkdir -p /dev/bus/usb
USER dockeruser
WORKDIR /home/dockeruser/ros2_ws

# === エントリーポイント設定 ===
COPY --chmod=755 docker/entrypoint.sh /home/dockeruser/entrypoint.sh
ENTRYPOINT ["/home/dockeruser/entrypoint.sh"]
CMD ["bash"]