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
    config_path = os.path.join(get_package_share_directory("cslam_tests"),
                               "config/")
    config_file = LaunchConfiguration('config_file').perform(context)

    cslam_processes_0 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("cslam_tests"), "launch", "cslam",
                         "cslam_lidar.launch.py")),
        launch_arguments={
            "config_path": config_path,
            "config_file": config_file,
            "robot_id": "0",
            "namespace": "/r0",
            "nb_robots": "2",
            #"launch_prefix_cslam": "gdbserver localhost:3000",
        }.items(),
    )

    cslam_processes_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("cslam_tests"), "launch", "cslam",
                         "cslam_lidar.launch.py")),
        launch_arguments={
            "config_path": config_path,
            "config_file": config_file,
            "robot_id": "1",
            "namespace": "/r1",
            "nb_robots": "2",
        }.items(),
    )

    bag_file_0 = os.path.join(get_package_share_directory("cslam_tests"),
                              "data", "KITTI00-0-s-lidar")

    bag_process_0 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("cslam_tests"),
                "launch",
                "sensors",
                "bag_kitti.launch.py",
            )),
        launch_arguments={
            "namespace": "/r0",
            "bag_file": bag_file_0,
            "rate": "1"
        }.items(),
    )

    bag_file_1 = os.path.join(get_package_share_directory("cslam_tests"),
                              "data", "KITTI00-2-e-lidar")

    bag_process_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("cslam_tests"),
                "launch",
                "sensors",
                "bag_kitti.launch.py",
            )),
        launch_arguments={
            "namespace": "/r1",
            "bag_file": bag_file_1,
            "rate": "1"
        }.items(),
    )

    tf_process = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments="0 0 0 0 0 0 velo_link base_link".split(" "),
        parameters=[]
    )

    return [
        PushLaunchConfigurations(),
        TimerAction(
            period='30',
            actions=[bag_process_0]),
        PopLaunchConfigurations(),
        PushLaunchConfigurations(),
        TimerAction(
            period='90',
            actions=[bag_process_1]),
        PopLaunchConfigurations(),
        PushLaunchConfigurations(),
        cslam_processes_0,
        PopLaunchConfigurations(),
        PushLaunchConfigurations(),
        cslam_processes_1,
        PopLaunchConfigurations(),
        tf_process,
    ]


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('config_file',
                              default_value='test_default_kitti_lidar.yaml',
                              description=''),
        OpaqueFunction(function=launch_setup)
        ])
