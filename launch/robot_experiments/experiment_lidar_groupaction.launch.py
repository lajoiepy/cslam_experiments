import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import TimerAction, OpaqueFunction, PushLaunchConfigurations, PopLaunchConfigurations, DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap
import launch_testing
import launch_testing.actions
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):

    # Params
    cslam_proc = GroupAction([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("cslam_experiments"),
                         "launch", "cslam", "cslam_lidar.launch.py")),
        launch_arguments={
            "config_path": os.path.join(get_package_share_directory("cslam_experiments"), "config/"),
            "config_file": LaunchConfiguration('cslam_config_file').perform(context),
            "robot_id": LaunchConfiguration('robot_id').perform(context),
            "namespace": "/r" + LaunchConfiguration('robot_id').perform(context),
            "max_nb_robots": LaunchConfiguration('max_nb_robots').perform(context),
        }.items()),
        SetRemap(src="/r0/odom", dst="/odometry/filtered")])

    zenoh_dds_brigde_process = ExecuteProcess(
                        cmd=['zenoh-bridge-ros2dds', '-c', os.path.join(get_package_share_directory("cslam_experiments"), "config", "zenoh", "zenoh_cslam.json5")],
                    ),

    # Launch schedule
    schedule = []

    schedule.append(SetEnvironmentVariable('ROS_DOMAIN_ID',  LaunchConfiguration('robot_id').perform(context)))
    schedule.append(zenoh_dds_brigde_process)

    schedule.append(PushLaunchConfigurations())
    schedule.append(cslam_proc)
    schedule.append(PopLaunchConfigurations())
    
    return schedule


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('robot_id', default_value='0'),
        DeclareLaunchArgument('max_nb_robots', default_value='5'),
        DeclareLaunchArgument('cslam_config_file',
                              default_value='ouster_lidar.yaml',
                              description=''),
        OpaqueFunction(function=launch_setup)
    ])
