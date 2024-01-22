from launch_ros.actions          import Node 
from launch                      import LaunchDescription
from launch.substitutions        import LaunchConfiguration, PathJoinSubstitution
from launch.actions              import DeclareLaunchArgument
from launch_ros.substitutions    import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    package_name           = "ground_robot"
    package_name_map_saver = "nav2_map_server"
    
    ld = LaunchDescription()
    
    map_name_launch_arg = DeclareLaunchArgument(
        'map_name',
        default_value='my_map1'
    )

    path = get_package_share_directory(package_name) + "/maps/"
    path_joined = PathJoinSubstitution([path, LaunchConfiguration('map_name')])

    save_map_node = Node(
        package=package_name_map_saver,
        executable="map_saver_cli",
        name="map_saver_cli",
        arguments=["-f",path_joined]
    )

    ld.add_action(map_name_launch_arg)
    ld.add_action(save_map_node)

    return ld


