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
    dataset = "Graco_" + LaunchConfiguration('sequence').perform(context)
    robot_delay_s = LaunchConfiguration('robot_delay_s').perform(context)  
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
                             "launch", "cslam", "cslam_lidar.launch.py")),
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
        schedule.append(
            TimerAction(period=float(robot_delay_s) * i,
                        actions=[cslam_processes[i]]))
        schedule.append(PopLaunchConfigurations())
        schedule.append(PushLaunchConfigurations())
        schedule.append(
            TimerAction(period=float(robot_delay_s) * i,
                        actions=[odom_processes[i]]))
        schedule.append(PopLaunchConfigurations())         

    bag_processes = []
    for i in range(max_nb_robots):
        bag_file = os.path.join(
                get_package_share_directory("cslam_experiments"), "data",
                dataset, "Graco-" + str(i))
        bag_proc = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("cslam_experiments"),
                        "launch",
                        "sensors",
                        "bag_graco.launch.py",
                    )),
                launch_arguments={
                    "namespace": "/r" + str(i),
                    "bag_file": bag_file,
                    "rate": str(rate)
                }.items(),
            )
        bag_processes.append(bag_proc)

    for i in range(max_nb_robots):
        schedule.append(PushLaunchConfigurations())
        schedule.append(
            TimerAction(period=float(robot_delay_s) * i + float(launch_delay_s),
                        actions=[bag_processes[i]]))
        schedule.append(PopLaunchConfigurations())

    tf_process = Node(package="tf2_ros",
                      executable="static_transform_publisher",
                      arguments="0 0 0 0 0 0 velodyne base_link".split(" "),
                      parameters=[])
    tf_process_imu = Node(package="tf2_ros",
                      executable="static_transform_publisher",
                      arguments="-0.01192 -0.0197 0.1226 0 0 0 velodyne gnss".split(" "),
                      parameters=[])
    tf_process_cam0 = Node(package="tf2_ros",
                      executable="static_transform_publisher",
                      arguments="0 0 0 0 0 0 camera_21239776 base_link".split(" "),
                      parameters=[])
    tf_process_cam1 = Node(package="tf2_ros",
                      executable="static_transform_publisher",
                      arguments="-0.23223 0 0 0 0 0 camera_21387977 camera_21239776".split(" "),
                      parameters=[])
    schedule.append(PushLaunchConfigurations())
    schedule.append(tf_process)
    schedule.append(PopLaunchConfigurations())
    schedule.append(PushLaunchConfigurations())
    schedule.append(tf_process_imu)
    schedule.append(PopLaunchConfigurations())
    # schedule.append(PushLaunchConfigurations())
    # schedule.append(tf_process_cam0)
    # schedule.append(PopLaunchConfigurations())
    # schedule.append(PushLaunchConfigurations())
    # schedule.append(tf_process_cam1)
    # schedule.append(PopLaunchConfigurations())

    return schedule


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('max_nb_robots', default_value='3'),
        DeclareLaunchArgument('sequence', default_value='Ground'),
        DeclareLaunchArgument('robot_delay_s', default_value='400', description="Delay between launching each robot. Ajust depending on the computing power of your machine."),
        DeclareLaunchArgument('launch_delay_s', default_value='10', description="Delay between launching the bag and the robot. In order to let the robot initialize properly and not loose the first bag data frames."),
        DeclareLaunchArgument('config_file',
                              default_value='graco_lidar.yaml',
                              description=''),
        DeclareLaunchArgument('rate', default_value='1.0'),
        DeclareLaunchArgument('enable_simulated_rendezvous', default_value='true'),
        DeclareLaunchArgument('rendezvous_config', default_value='graco_ground.config'),
        OpaqueFunction(function=launch_setup)
    ])
