import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # ハードウェア同期を有効にする
    config_params = {
        'align_depth.enable': True,
        'pointcloud.enable': True,
        'initial_reset': True,

        # ハードウェア同期を有効化！これが本命
        'enable_sync': True,

        'enable_infra1': False,
        'enable_infra2': False,

        'depth_module.profile': '640x480x30',
        'rgb_camera.profile': '640x480x30',
        
        'depth_qos': 'SENSOR_DATA',
        'color_qos': 'SENSOR_DATA',
        'aligned_depth_to_color_qos': 'SENSOR_DATA',
        'camera_info_qos': 'SENSOR_DATA',
        'pointcloud.pointcloud_qos': 'SENSOR_DATA',
    }

    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            namespace='/camera',
            output='screen',
            parameters=[config_params]
        )
    ])