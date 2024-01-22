from launch import LaunchDescription

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


from launch.actions                    import IncludeLaunchDescription
from launch_ros.substitutions          import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


package_name = "ground_robot"


def generate_launch_description():

    package_name_bringup = "nav2_bringup"
    launch_file_name_bringup = "bringup_launch.py"

    ld = LaunchDescription()

    launch_navigation = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([FindPackageShare(package_name_bringup), '/launch/', launch_file_name_bringup]),
            launch_arguments = {"use_sim_time":"True", "map":"/home/ppa/ros2_ws_2/src/ground_robot/maps/my_map.yaml"}.items()
            )

    ld.add_action(launch_navigation)


    return ld