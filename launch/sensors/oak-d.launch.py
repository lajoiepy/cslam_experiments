import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):

    return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('cslam_experiments'), "launch", "sensors", "oak-d_description.launch.py")
                ),
                launch_arguments={
								    "tf_prefix": LaunchConfiguration('namespace').perform(context)[1:],
								}.items(),
            ),
            ComposableNodeContainer(
                name="container",
                namespace="",
                package="rclcpp_components",
                executable="component_container",
                composable_node_descriptions=[
                    # Driver itself
                    ComposableNode(
                        package="depthai_ros_driver",
                        plugin="depthai_ros_driver::RGBDCamera",
                        name="camera",
                        parameters=[{"i_rgb_fps": 30.0}],
                        remappings=[
								            ('/camera/color/image_raw', LaunchConfiguration('namespace').perform(context) + '/color/image_raw'),
								            ('/camera/color/camera_info', LaunchConfiguration('namespace').perform(context) + '/color/camera_info'),
								            ('/camera/depth/image_raw', LaunchConfiguration('namespace').perform(context) + '/aligned_depth_to_color/image_raw'),
								        ],
                    ),
                ],
                output="screen",
            ),
        ]

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='/r0',
                              description=''),
        OpaqueFunction(function=launch_setup)
        ])
