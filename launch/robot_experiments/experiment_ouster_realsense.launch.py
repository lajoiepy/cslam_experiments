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

    # Params
    cslam_proc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("cslam_experiments"),
                         "launch", "cslam", "cslam_rgbd.launch.py")),
        launch_arguments={
            "config_path": os.path.join(get_package_share_directory("cslam_experiments"), "config/"),
            "config_file": LaunchConfiguration('cslam_config_file').perform(context),
            "robot_id": LaunchConfiguration('robot_id').perform(context),
            "namespace": "/r" + LaunchConfiguration('robot_id').perform(context),
            "max_nb_robots": LaunchConfiguration('max_nb_robots').perform(context),
        }.items(),
    )

    # Camera
    camera_proc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('cslam_experiments'), 'launch',
                         'sensors', 'realsense_d400.launch.py')),
        launch_arguments={
            "namespace": "/r" + LaunchConfiguration('robot_id').perform(context),
        }.items(),
    )

    # Odom
    odom_proc = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('cslam_experiments'), 'launch',
                             'odometry', 'rtabmap_ouster_lidar_odometry.launch.py')),
            launch_arguments={
                "namespace": "",
                "robot_id": LaunchConfiguration('robot_id').perform(context),
                'log_level': "info",
            }.items(),
        )

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros2_ouster'), 'launch',
                         'driver_launch.py')),
	)

    imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('vectornav'), 'launch',
                         'vectornav.launch.py')),
        )

    tf_process = Node(package="tf2_ros",
                      executable="static_transform_publisher",
                      arguments="0 0 0 0 0 0 laser_sensor_frame base_link".split(" "),
                      parameters=[])

    tf_process_imu = Node(package="tf2_ros",
                      executable="static_transform_publisher",
                      arguments="-0.13 0.045 -0.02 0 0 0 laser_sensor_frame vectornav".split(" "),
                      parameters=[])

    # Launch schedule
    schedule = []

    schedule.append(PushLaunchConfigurations())
    schedule.append(cslam_proc)
    schedule.append(PopLaunchConfigurations())
     
    schedule.append(PushLaunchConfigurations())
    schedule.append(camera_proc)
    schedule.append(PopLaunchConfigurations())   

    schedule.append(PushLaunchConfigurations())
    schedule.append(odom_proc)
    schedule.append(PopLaunchConfigurations())  

    schedule.append(PushLaunchConfigurations())
    schedule.append(lidar)
    schedule.append(PopLaunchConfigurations())  

    schedule.append(PushLaunchConfigurations())
    schedule.append(tf_process)
    schedule.append(PopLaunchConfigurations())  

    schedule.append(PushLaunchConfigurations())
    schedule.append(tf_process_imu)
    schedule.append(PopLaunchConfigurations())

    schedule.append(PushLaunchConfigurations())
    schedule.append(imu)
    schedule.append(PopLaunchConfigurations()) 

    return schedule


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('robot_id', default_value='0'),
        DeclareLaunchArgument('max_nb_robots', default_value='5'),
        DeclareLaunchArgument('cslam_config_file',
                              default_value='realsense_rgbd.yaml',
                              description=''),
        OpaqueFunction(function=launch_setup)
    ])