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
    dataset = "KITTI" + LaunchConfiguration('sequence').perform(context)
    robot_delay_s = LaunchConfiguration('robot_delay_s').perform(context)  
    launch_delay_s = LaunchConfiguration('launch_delay_s').perform(context)  
    rate = float(LaunchConfiguration('rate').perform(context))

    # Ajust value according to rate
    robot_delay_s = float(robot_delay_s) / rate
    launch_delay_s = float(launch_delay_s) / rate

    cslam_processes = []
    bag_processes = []
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

        bag_file = os.path.join(
            get_package_share_directory("cslam_experiments"), "data",
            dataset + "_" + str(max_nb_robots) + "robots", dataset + "-" + str(i))

        bag_proc = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("cslam_experiments"),
                    "launch",
                    "sensors",
                    "bag_kitti.launch.py",
                )),
            launch_arguments={
                "namespace": "/r" + str(i),
                "bag_file": bag_file,
                "rate": str(rate)
            }.items(),
        )

        bag_processes.append(bag_proc)

        odom_proc = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('cslam_experiments'), 'launch',
                             'odometry', 'rtabmap_kitti_lidar_odometry.launch.py')),
            launch_arguments={
                "namespace": "/r" + str(i),
                "robot_id": str(i),
                'log_level': "fatal",
                "use_sim_time": "false",
                "wait_imu_to_init": "false",
            }.items(),
        )

        odom_processes.append(odom_proc)

    # KITTI specific transform
    tf_process = Node(package="tf2_ros",
                      executable="static_transform_publisher",
                      arguments="0 0 0 0 0 0 base_link velo_link".split(" "),
                      parameters=[])
    tf_process_imu = Node(package="tf2_ros",
                      executable="static_transform_publisher",
                      arguments="0 0 0 0 0 0 base_link imu_link".split(" "),
                      parameters=[])

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

    for i in range(max_nb_robots):
        schedule.append(PushLaunchConfigurations())
        schedule.append(
            TimerAction(period=float(robot_delay_s) * i + float(launch_delay_s),
                        actions=[bag_processes[i]]))
        schedule.append(PopLaunchConfigurations())

    schedule.append(tf_process)
    schedule.append(tf_process_imu)

    return schedule


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('max_nb_robots', default_value='5'),
        DeclareLaunchArgument('sequence', default_value='360-09'),
        DeclareLaunchArgument('robot_delay_s', default_value='350', description="Delay between launching each robot. Ajust depending on the computing power of your machine."),
        DeclareLaunchArgument('launch_delay_s', default_value='10', description="Delay between launching the bag and the robot. In order to let the robot initialize properly and not loose the first bag data frames."),
        DeclareLaunchArgument('config_file',
                              default_value='kitti_lidar.yaml',
                              description=''),
        DeclareLaunchArgument('rate', default_value='0.5'),
        DeclareLaunchArgument('enable_simulated_rendezvous', default_value='true'),
        DeclareLaunchArgument('rendezvous_config', default_value='kitti09_5robots.config'),
        OpaqueFunction(function=launch_setup)
    ])
