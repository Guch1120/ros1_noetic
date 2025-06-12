#!/bin/bash
set -e

echo "--- FlexBEワークスペースのセットアップを開始します ---"

WS_DIR=~/ros2_ws
mkdir -p $WS_DIR/src
cd $WS_DIR

# --- 1. FlexBEのリポジトリを準備 ---
echo "--- 1. Cloning FlexBE repositories ---"
if [ ! -d "src/flexbe_behavior_engine" ]; then
    git clone -b humble https://github.com/FlexBE/flexbe_behavior_engine.git src/flexbe_behavior_engine
else
    echo "flexbe_behavior_engine は既に存在します。"
fi

if [ ! -d "src/flexbe_app" ]; then
    git clone -b humble https://github.com/FlexBE/flexbe_app.git src/flexbe_app
else
    echo "flexbe_app は既に存在します。"
fi

# --- 1.5. nwjs_install スクリプトを自動修正 ---
echo "--- 1.5. Patching nwjs_install script ---"
NWJS_INSTALL_SCRIPT=src/flexbe_app/bin/nwjs_install

# curl に -L オプションを追加して、リダイレクトを追跡できるようにする
sed -i 's/curl -s -o/curl -sL -o/' $NWJS_INSTALL_SCRIPT
# VERSION変数を、実際にダウンロードするバージョンに合わせる
sed -i 's/VERSION="v0.55.0"/VERSION="v0.83.0"/' $NWJS_INSTALL_SCRIPT
echo "nwjs_install script has been patched."


# --- 2. 依存関係をインストール ---
echo "--- 2. Installing dependencies with rosdep ---"
source /opt/ros/humble/setup.bash
sudo apt-get update # NOTE: 2回目以降は不要な場合が多い
rosdep update
rosdep install --from-paths src --ignore-src -r -y


# --- 3. ワークスペースをビルド ---
echo "--- 3. Building the workspace with colcon ---"
sudo chown -R dockeruser:dockeruser ~/ros2_ws
colcon build --symlink-install

# --- 4. nwjsをインストール (パッチ済み) ---
echo "--- 4. Installing nwjs ---"
source $WS_DIR/install/setup.bash
ros2 run flexbe_app nwjs_install

echo "--- セットアップが完了しました！ ---"
echo "FlexBEワークスペースは $WS_DIR にあります。"

# --- 5. 環境変数の設定 ---　#要改良　bashでは反映されず結局現在は手動で打ち込んでいる
echo "--- 5. Setting up environment variables ---"
echo "source $WS_DIR/install/setup.bash ->
export XDG_CONFIG_HOME=/tmp/.chromium ->
export XDG_CACHE_HOME=/tmp/.chromium"
