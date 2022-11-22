import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import SetParameter


def launch_setup(context, *args, **kwargs):

    return [
        DeclareLaunchArgument('bag_file', default_value='', description=''),
        DeclareLaunchArgument('namespace', default_value='/r', description=''),
        DeclareLaunchArgument('rate', default_value='1.0', description=''),
        DeclareLaunchArgument('bag_start_delay',
                              default_value='5.0',
                              description=''),
        TimerAction(
            period=LaunchConfiguration('bag_start_delay'),
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'bag', 'play',
                        LaunchConfiguration('bag_file').perform(context), '-r',
                        LaunchConfiguration('rate'), '--remap',
                        '/Alpha/imu/data:=' +
                        LaunchConfiguration('namespace').perform(context) +
                        '0/imu/data',
                        LaunchConfiguration('rate'), '--remap',
                        '/Bob/imu/data:=' +
                        LaunchConfiguration('namespace').perform(context) +
                        '1/imu/data',
                        LaunchConfiguration('rate'), '--remap',
                        '/Carol/imu/data:=' +
                        LaunchConfiguration('namespace').perform(context) +
                        '2/imu/data'
                    ],
                    name='bag',
                    output='screen',
                )
            ]),
        Node(package='image_transport',
             executable='republish',
             name='republish',
             arguments=[
                 'compressed', 'raw', '--ros-args', '--remap', '/in/compressed:=/Alpha/left_camera/compressed', '--remap',
                 'out:=' + LaunchConfiguration('namespace').perform(context) +
                 '0/stereo_camera/left/image_color'
                 ]),
        Node(package='image_transport',
             executable='republish',
             name='republish',
             arguments=[
              'compressed', 'raw', '--ros-args', '--remap', '/in/compressed:=/Alpha/right_camera/compressed', '--remap',
                 'out:=' + LaunchConfiguration('namespace').perform(context) +
                 '0/stereo_camera/right/image_color'
                 ]),

        Node(package='image_transport',
             executable='republish',
             name='republish',
             arguments=[
                 'compressed', 'raw', '--ros-args', '--remap', '/in/compressed:=/Bob/left_camera/compressed', '--remap',
                 'out:=' + LaunchConfiguration('namespace').perform(context) +
                 '1/stereo_camera/left/image_color'
                 ]),
        Node(package='image_transport',
             executable='republish',
             name='republish',
             arguments=[
              'compressed', 'raw', '--ros-args', '--remap', '/in/compressed:=/Bob/right_camera/compressed', '--remap',
                 'out:=' + LaunchConfiguration('namespace').perform(context) +
                 '1/stereo_camera/right/image_color'
                 ]),

        Node(package='image_transport',
             executable='republish',
             name='republish',
             arguments=[
                 'compressed', 'raw', '--ros-args', '--remap', '/in/compressed:=/Carol/left_camera/compressed', '--remap',
                 'out:=' + LaunchConfiguration('namespace').perform(context) +
                 '2/stereo_camera/left/image_color'
                 ]),
        Node(package='image_transport',
             executable='republish',
             name='republish',
             arguments=[
              'compressed', 'raw', '--ros-args', '--remap', '/in/compressed:=/Carol/right_camera/compressed', '--remap',
                 'out:=' + LaunchConfiguration('namespace').perform(context) +
                 '2/stereo_camera/right/image_color'
                 ]),

        Node(package='cslam_experiments',
             executable='publish_stereo_calibration_s3e.py',
             name='calibration_publisher',
             parameters=[
                        {
                            "robot_id": 0,
                            "frontend.stereo_calibration_file": os.path.join(get_package_share_directory('cslam_experiments'),
                                  'config', 's3e', 'alpha.yaml'),
                        }
                    ]),


        Node(package='cslam_experiments',
             executable='publish_stereo_calibration_s3e.py',
             name='calibration_publisher',
             parameters=[
                        {
                            "robot_id": 1,
                            "frontend.stereo_calibration_file": os.path.join(get_package_share_directory('cslam_experiments'),
                                  'config', 's3e', 'bob.yaml'),
                        }
                    ]),

        Node(package='cslam_experiments',
             executable='publish_stereo_calibration_s3e.py',
             name='calibration_publisher',
             parameters=[
                        {
                            "robot_id": 2,
                            "frontend.stereo_calibration_file": os.path.join(get_package_share_directory('cslam_experiments'),
                                  'config', 's3e', 'carol.yaml'),
                        }
                    ]),
    ]


def generate_launch_description():

    return LaunchDescription([OpaqueFunction(function=launch_setup)])
