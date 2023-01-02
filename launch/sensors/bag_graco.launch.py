import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import PushLaunchConfigurations, PopLaunchConfigurations, DeclareLaunchArgument, ExecuteProcess, TimerAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):

    rectification_nodes = []
    for i in range(int(LaunchConfiguration('max_nb_robots').perform(context))):
        rectification_nodes.append(
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='rectify_color_node',
                # Remap subscribers and publishers
                remappings=[
                    ('image', LaunchConfiguration('namespace').perform(context) + '/stereo_camera/left/image_raw'),
                    ('camera_info', LaunchConfiguration('namespace').perform(context) + '/stereo_camera/left/camera_info'),
                    ('image_rect', LaunchConfiguration('namespace').perform(context) + '/stereo_camera/left/image_color'),
                ],
            ))
        rectification_nodes.append(ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='rectify_color_node',
                # Remap subscribers and publishers
                remappings=[
                    ('image', LaunchConfiguration('namespace').perform(context) + '/stereo_camera/right/image_raw'),
                    ('camera_info', LaunchConfiguration('namespace').perform(context) + '/stereo_camera/right/camera_info'),
                    ('image_rect', LaunchConfiguration('namespace').perform(context) + '/stereo_camera/right/image_color'),
                ],
            ))
    rectification_container = ComposableNodeContainer(
        name='image_proc_container',
        namespace=LaunchConfiguration('namespace').perform(context),
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=rectification_nodes,
        output='screen'
    )
    load_composable_nodes = LoadComposableNodes(
        composable_node_descriptions=rectification_nodes,
        target_container=rectification_container
    )
    return [
        # PushLaunchConfigurations(),
        # rectification_container,
        # PopLaunchConfigurations(),
        # PushLaunchConfigurations(),
        # load_composable_nodes,
        # PopLaunchConfigurations(),
        TimerAction(
            period=LaunchConfiguration('bag_start_delay'),
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'bag', 'play',
                        LaunchConfiguration('bag_file').perform(context), '-r',
                        LaunchConfiguration('rate'), '--remap',
                        '/camera_left/camera_info:=' +
                        LaunchConfiguration('namespace').perform(context) +
                        '/stereo_camera/left/camera_info',
                        '/camera_left/image_raw:=' +
                        LaunchConfiguration('namespace').perform(context) +
                        '/stereo_camera/left/image_raw',
                        '/camera_right/camera_info:=' +
                        LaunchConfiguration('namespace').perform(context) +
                        '/stereo_camera/right/camera_info',
                        '/camera_right/image_raw:=' +
                        LaunchConfiguration('namespace').perform(context) +
                        '/stereo_camera/right/image_raw',
                        '/velodyne/points:=' +
                        LaunchConfiguration('namespace').perform(context) +
                        '/pointcloud',
                        '/gnss/gps:=' +
                        LaunchConfiguration('namespace').perform(context) +
                        '/gps/fix',
                        '/gnss/imu:=' +
                        LaunchConfiguration('namespace').perform(context) +
                        '/imu/data',
                    ],
                    name='bag',
                    output='screen',
                )
            ]),
    ]


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('bag_file', default_value='', description=''),
        DeclareLaunchArgument('namespace', default_value='/r0',
                              description=''),
        DeclareLaunchArgument('rate', default_value='1.0', description=''),
        DeclareLaunchArgument('bag_start_delay',
                              default_value='5.0',
                              description=''),
        OpaqueFunction(function=launch_setup)])
