FROM ros:noetic
 
ENV DEBIAN_FRONTEND=noninteractive

# 基本ツールのインストール
RUN apt-get update && apt-get install -y \
    git \
    wget \
    nano \
    curl \
    lsb-release \
    sudo \
    gnupg2 \
    python3-pip \
    python3-catkin-tools \
    ros-noetic-rviz \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    terminator \
    && rm -rf /var/lib/apt/lists/*

# ユーザー作成
RUN useradd -m -s /bin/bash dockeruser && echo "dockeruser:docker" | chpasswd && adduser dockeruser sudo

#パスワード設定
RUN echo "dockeruser:yakiniku" | chpasswd

#USBデバイスのアクセス権限設定
RUN mkdir -p /dev/bus/usb

# entrypoint.sh をコンテナ内にコピー＆実行可能化
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# terminatorの設定ファイルをコピー
COPY terminator_config /home/dockeruser/.config/terminator/config
RUN chown -R dockeruser:dockeruser /home/dockeruser/.config

# dockeruser に切り替え
USER dockeruser
WORKDIR /home/dockeruser

RUN echo "source" /opt/ros/noetic/setup.bash >> /home/dockeruser/.bashrc

# 環境設定
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# エントリーポイント
ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash", "-c", "/entrypoint.sh"]
