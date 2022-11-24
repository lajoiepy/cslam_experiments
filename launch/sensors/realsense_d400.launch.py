import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import TimerAction, OpaqueFunction, PushLaunchConfigurations, PopLaunchConfigurations, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("realsense2_camera"),
                             "launch", "rs_launch.py")),
            launch_arguments={
                "align_depth.enable": "true",
                "enable_sync": "true",
                "log_level": "warn",
                "rgb_camera.profile": "424,240,30",
                "depth_module.profile": "424,240,30",
                "camera_name": LaunchConfiguration('namespace').perform(context)[1:],
            }.items(),
        )
    ]


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='/r0',
                              description=''),
        OpaqueFunction(function=launch_setup)
        ])
