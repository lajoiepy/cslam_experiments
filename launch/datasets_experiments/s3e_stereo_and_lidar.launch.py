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
    config_path = os.path.join(
        get_package_share_directory("cslam_experiments"), "config/")
    config_file = LaunchConfiguration('config_file').perform(context)

    # Params
    max_nb_robots = int(LaunchConfiguration('max_nb_robots').perform(context))
    dataset = "S3E_" + LaunchConfiguration('sequence').perform(context)
    launch_delay_s = LaunchConfiguration('launch_delay_s').perform(context)  
    rate = float(LaunchConfiguration('rate').perform(context))

    # Ajust value according to rate
    launch_delay_s = float(launch_delay_s) / rate

    cslam_processes = []
    odom_processes = []

    for i in range(max_nb_robots):
        proc = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("cslam_experiments"),
                             "launch", "cslam", "cslam_stereo.launch.py")),
            launch_arguments={
                "config_path": config_path,
                "config_file": config_file,
                "robot_id": str(i),
                "namespace": "/r" + str(i),
                "max_nb_robots": str(max_nb_robots),
                "enable_simulated_rendezvous": LaunchConfiguration('enable_simulated_rendezvous'),
                "rendezvous_schedule_file": os.path.join(get_package_share_directory("cslam_experiments"),
                             "config", "rendezvous", LaunchConfiguration('rendezvous_config').perform(context)),
            }.items(),
        )

        cslam_processes.append(proc)

        odom_proc = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('cslam_experiments'), 'launch',
                             'odometry', 'rtabmap_s3e_lidar_odometry.launch.py')),
            launch_arguments={
                'log_level': "info",
                "namespace": "/r" + str(i),
                "robot_id": str(i),
            }.items(),
        )

        odom_processes.append(odom_proc)

    # Launch schedule
    schedule = []

    for i in range(max_nb_robots):
        schedule.append(PushLaunchConfigurations())
        schedule.append(cslam_processes[i])
        schedule.append(PopLaunchConfigurations())
        schedule.append(PushLaunchConfigurations())
        schedule.append(odom_processes[i])
        schedule.append(PopLaunchConfigurations())        

    bag_file = os.path.join(
            get_package_share_directory("cslam_experiments"), "data",
            dataset)
    bag_proc = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("cslam_experiments"),
                    "launch",
                    "sensors",
                    "bag_s3e.launch.py",
                )),
            launch_arguments={
                "namespace": "/r",
                "bag_file": bag_file,
                "rate": str(rate)
            }.items(),
        )
    schedule.append(PushLaunchConfigurations())
    schedule.append(
        TimerAction(period=float(launch_delay_s),
                    actions=[bag_proc]))
    schedule.append(PopLaunchConfigurations())

    return schedule


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('max_nb_robots', default_value='3'),
        DeclareLaunchArgument('sequence', default_value='Playground'),
        DeclareLaunchArgument('launch_delay_s', default_value='10', description="Delay between launching the bag and the robot. In order to let the robot initialize properly and not loose the first bag data frames."),
        DeclareLaunchArgument('config_file',
                              default_value='s3e_stereo_and_lidar.yaml',
                              description=''),
        DeclareLaunchArgument('rate', default_value='0.5'),
        DeclareLaunchArgument('enable_simulated_rendezvous', default_value='true'),
        DeclareLaunchArgument('rendezvous_config', default_value='s3e_playground.config'),
        OpaqueFunction(function=launch_setup)
    ])
