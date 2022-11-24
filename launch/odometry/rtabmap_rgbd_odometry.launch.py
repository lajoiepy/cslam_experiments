from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    parameters=[{
          'frame_id': LaunchConfiguration('namespace').perform(context)[1:] + '_link',
          'subscribe_depth':True,
          'approx_sync':False, # Set to True for OAK-D
          }]

    remappings=[
          ('rgb/image', LaunchConfiguration('namespace').perform(context) + '/color/image_raw'),
          ('rgb/camera_info', LaunchConfiguration('namespace').perform(context) + '/color/camera_info'),
          ('depth/image', LaunchConfiguration('namespace').perform(context) + '/aligned_depth_to_color/image_raw'),
          ('odom', LaunchConfiguration('namespace').perform(context) + '/odom'),]

    return [
        # Nodes to launch
        Node(
            package='rtabmap_ros', executable='rgbd_odometry', output='screen', name='rgbd_odometry',
            parameters=parameters,
            remappings=remappings,
            ),
    ]

def generate_launch_description():
    
    return LaunchDescription([            
        DeclareLaunchArgument('namespace', default_value='/r0',
                              description=''),
        OpaqueFunction(function=launch_setup)
    ])
