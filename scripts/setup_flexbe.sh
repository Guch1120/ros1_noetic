#!/bin/bash
set -e

echo "--- FlexBE setup Start ---"

WS_DIR=~/ros2_ws
mkdir -p $WS_DIR/src
cd $WS_DIR

# --- 1. FlexBEのリポジトリを準備 ---
echo "--- 1. Cloning FlexBE repositories ---"
if [ ! -d "src/flexbe_behavior_engine" ]; then
    git clone --branch humble https://github.com/FlexBE/flexbe_behavior_engine.git src/flexbe_behavior_engine
else
    echo "flexbe_behavior_engine is already present. change branch for humble..."
    (cd src/flexbe_behavior_engine && git checkout humble && git pull)
fi

if [ ! -d "src/flexbe_app" ]; then
    git clone --branch humble https://github.com/FlexBE/flexbe_app.git src/flexbe_app
else
    echo "flexbe_app is already present. change branch for humble..."
    (cd src/flexbe_app && git checkout humble && git pull)
fi

# --- 1.5. nwjs_install スクリプトを自動修正 ---
echo "--- 1.5. Patching nwjs_install script ---"
NWJS_INSTALL_SCRIPT=src/flexbe_app/bin/nwjs_install
sed -i 's/curl -s -o/curl -sL -o/' $NWJS_INSTALL_SCRIPT
sed -i 's/VERSION="v0.55.0"/VERSION="v0.83.0"/' $NWJS_INSTALL_SCRIPT
echo "nwjs_install script has been patched."

# --- 1.6. flexbe_ocs.launch.py を自動修正 ---
echo "--- 1.6. Patching flexbe_ocs.launch.py ---"
OCS_LAUNCH_FILE=src/flexbe_app/launch/flexbe_ocs.launch.py
cat <<EOF > $OCS_LAUNCH_FILE
# Copyright 2023 Philipp Schillinger, Christopher Newport University
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and a disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Philipp Schillinger,
#      Christopher Newport University nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals


def generate_launch_description():

    offline = DeclareLaunchArgument("offline",
                                    description="Treat FlexBE App as offline editor () Editor mode as default",
                                    default_value="false")

    # Change the default value based on passing a true/false string to offline, or allow setting directly
    offline_arg = DeclareLaunchArgument("offline_arg",
                                        description="Optionally specify FlexBE App offline Editor mode ('--offline')",
                                        default_value="--offline",
                                        condition=LaunchConfigurationEquals("offline", "true"))
    online_arg = DeclareLaunchArgument("offline_arg",
                                        description="Optionally specify FlexBE App offline Editor mode ('--offline') default=''",
                                        default_value="",
                                        condition=LaunchConfigurationEquals("offline", "false"))

    no_app = DeclareLaunchArgument("no_app", default_value="false")
    use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="False")

    flexbe_app = Node(name="flexbe_app", package="flexbe_app", executable="run_app", output="screen",
                    arguments=[
                        LaunchConfiguration("offline_arg"),
                        '--disable-features=VaapiVideoDecoder',
                        '--use-gl=desktop',
                        '--ozone-platform=x11'
                    ])

    behavior_mirror = Node(name="behavior_mirror", package="flexbe_mirror",
                        executable="behavior_mirror_sm",
                        condition=LaunchConfigurationNotEquals("offline_arg", "--offline"))

    behavior_launcher = Node(name="behavior_launcher", package="flexbe_widget",
                            executable="be_launcher", output="screen",
                            condition=LaunchConfigurationNotEquals("offline_arg", "--offline"))

    return LaunchDescription([
        offline,
        offline_arg,
        online_arg,
        no_app,
        use_sim_time,
        behavior_mirror,
        flexbe_app,
        behavior_launcher
    ])
EOF
echo "flexbe_ocs.launch.py has been patched."


# --- 2. 依存関係をインストール ---
echo "--- 2. Installing dependencies with rosdep ---"
source /opt/ros/humble/setup.bash
sudo apt-get update

# rosdepのデータベースを一般ユーザー(dockeruser)として更新する
echo "Updating rosdep database as current user..."
rosdep update

# rosdep installは内部でsudoを使うので、ここではsudoは不要
rosdep install --from-paths src --ignore-src -r -y --skip-keys=flexbe_testing


# --- 3. ワークスペースをビルド ---
echo "--- 3. Building the workspace with colcon ---"
echo "Ensuring you own the workspace files..."
sudo chown -R dockeruser:dockeruser ~/ros2_ws
colcon build --symlink-install

# --- 4. nwjsをインストール (patched) ---
echo "--- 4. Installing nwjs ---"
source $WS_DIR/install/setup.bash
ros2 run flexbe_app nwjs_install

# --- 5. 環境設定を更新 ---
echo "--- 5. Sourcing the workspace setup file ---"
echo "source $WS_DIR/install/setup.bash" >> ~/.bashrc
echo "export XDG_CONFIG_HOME=/tmp/.chromium" >> ~/.bashrc
echo "export XDG_CACHE_HOME=/tmp/.chromium" >> ~/.bashrc
source $WS_DIR/install/setup.bash

echo ""
echo "--- complete setup! ---"
echo -e "\e[44;37m [Please]close terminator and run ./run_docker.bash at <HOST teminal>\e[0m\n"
