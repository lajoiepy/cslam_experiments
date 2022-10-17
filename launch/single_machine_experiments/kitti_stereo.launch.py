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
    nb_robots = int(LaunchConfiguration('nb_robots').perform(context))
    dataset = "KITTI" + LaunchConfiguration('sequence').perform(context)
    startup_delay_s = LaunchConfiguration('startup_delay_s').perform(context)  
    bag_delay_s = LaunchConfiguration('bag_delay_s').perform(context)  
    rate = float(LaunchConfiguration('rate').perform(context))

    # Ajust value according to rate
    startup_delay_s = float(startup_delay_s) / rate
    bag_delay_s = float(bag_delay_s) / rate

    cslam_processes = []
    bag_processes = []
    odom_processes = []

    for i in range(nb_robots):
        proc = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("cslam_experiments"),
                             "launch", "cslam", "cslam_stereo.launch.py")),
            launch_arguments={
                "config_path": config_path,
                "config_file": config_file,
                "robot_id": str(i),
                "namespace": "/r" + str(i),
                "nb_robots": str(nb_robots),
            }.items(),
        )

        cslam_processes.append(proc)

        bag_file = os.path.join(
            get_package_share_directory("cslam_experiments"), "data",
            dataset + "-" + str(i))

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
                             'odometry', 'rtabmap_kitti_stereo_odometry.launch.py')),
            launch_arguments={
                'log_level': "fatal",
                "robot_id": str(i),
                "nb_robots": LaunchConfiguration('nb_robots'),
            }.items(),
        )

        odom_processes.append(odom_proc)

    # KITTI specific transform
    tf_process = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments="0 0 0 0 0 0 camera_gray_left camera_link".split(" "),
        parameters=[]
    )

    # Launch schedule
    schedule = []

    for i in range(nb_robots):
        schedule.append(PushLaunchConfigurations())
        schedule.append(
            TimerAction(period=float(startup_delay_s) * i,
                        actions=[cslam_processes[i]]))
        schedule.append(PopLaunchConfigurations())
        schedule.append(PushLaunchConfigurations())
        schedule.append(
            TimerAction(period=float(startup_delay_s) * i,
                        actions=[odom_processes[i]]))
        schedule.append(PopLaunchConfigurations())        

    for i in range(nb_robots):
        schedule.append(PushLaunchConfigurations())
        schedule.append(
            TimerAction(period=float(startup_delay_s) * i + float(bag_delay_s),
                        actions=[bag_processes[i]]))
        schedule.append(PopLaunchConfigurations())

    schedule.append(tf_process)

    return schedule


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('nb_robots', default_value='5'),
        DeclareLaunchArgument('sequence', default_value='00'),
        DeclareLaunchArgument('startup_delay_s', default_value='120', description="Delay between launching each robot. Ajust depending on the computing power of your machine."),
        DeclareLaunchArgument('bag_delay_s', default_value='10', description="Delay between launching the bag and the robot. In order to let the robot initialize properly and not loose the first bag data frames."),
        DeclareLaunchArgument('config_file',
                              default_value='kitti_stereo.yaml',
                              description=''),
        DeclareLaunchArgument('rate', default_value='0.5'),
        OpaqueFunction(function=launch_setup)
    ])
