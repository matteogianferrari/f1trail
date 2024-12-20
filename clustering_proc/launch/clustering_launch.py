from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, OpaqueFunction

# to be used with the OpaqueFunction action. approach needed to obtain the substitution of 'transform_config', cast it to a list and concatenate other arguments
def setup_launch(context, *args, **kwargs):

    config = PathJoinSubstitution([FindPackageShare('clustering_proc'), 'config', 'clustering_node.yaml'])
    # define launch configuration to store the transform argument for the static_transform node
    # LaunchConfiguration substitutions allow us to acquire the value of the launch argument in any part of the launch description.
    transform_config = LaunchConfiguration('transform')
    
    clustering = Node(
        package='clustering_proc',
        executable='clustering',
        name='scan_clustering',
        parameters=[config]
    )
    
    # transform_config argument will be returned as a string here
    tf_args = transform_config.perform(context).split() + ['lidar_link', 'base_link']
    static_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_to_base_static_transform_publisher',
        arguments = tf_args
    )
    
    return [clustering, static_transform]

def generate_launch_description():
    # define launch argument that can be passed form the console (or above launch files)
    transform_config_launch_arg = DeclareLaunchArgument(
        'transform',
        default_value='0 0 0 0 0 0',
        description='Static transform from lidar_link to base_link. Format: x y z (in meters) yaw pitch roll (in radians)'
    )
    return LaunchDescription([
        transform_config_launch_arg,
        OpaqueFunction(function=setup_launch)
    ])
