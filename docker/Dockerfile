# ベースイメージをROS2 Humble Desktopにする
FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

#基本ツールのインストール
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
    # インストール後のお掃除
    && rm -rf /var/lib/apt/lists/*

# コンテナ内で作業するユーザーを作成
RUN useradd -m -s /bin/bash dockeruser && echo "dockeruser:yakiniku" | chpasswd && adduser dockeruser sudo

##USBデバイスのアクセス権限設定
RUN mkdir -p /dev/bus/usb 

# ユーザーをdockeruserに変更
USER dockeruser

# terminatorの設定ファイルをコピーしておく場所を作成
# (entrypoint.shがこの場所に設定をコピーする)
RUN mkdir -p /home/dockeruser/.config/terminator/

# ホストのterminator_configをコンテナにコピー
COPY terminator_config/ /home/dockeruser/terminator_config/
# ホストのフォルダをコンテナにコピー
COPY terminator_config/ /home/dockeruser/terminator_config/
COPY scripts/ /home/dockeruser/scripts/


# ワークスペースを作成
WORKDIR /home/dockeruser/ros2_ws

# エントリーポイントのスクリプトをコピー 
COPY docker/entrypoint.sh /home/dockeruser/entrypoint.sh

ENTRYPOINT ["/home/dockeruser/entrypoint.sh"]
# デフォルトでbashを起動しておく
CMD ["bash"]