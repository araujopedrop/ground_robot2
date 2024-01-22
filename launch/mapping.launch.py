from launch import LaunchDescription

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


from launch.actions                    import IncludeLaunchDescription
from launch_ros.substitutions          import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions                    import RegisterEventHandler, EmitEvent
from launch.event_handlers             import OnProcessExit
from launch.events                     import Shutdown
package_name = "ground_robot"


def generate_launch_description():

    package_name_navigation = "nav2_bringup"
    launch_file_name_navigation = "navigation_launch.py"
    package_name_slam = "slam_toolbox"
    launch_file_name_slam = "online_async_launch.py"

    ld = LaunchDescription()

    launch_navigation = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([FindPackageShare(package_name_slam), '/launch/', launch_file_name_slam]),
            launch_arguments = {'use_sim_time':"True"}.items()
            )

    
    launch_to_slam = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([FindPackageShare(package_name_navigation), '/launch/', launch_file_name_navigation]),
            launch_arguments = {'use_sim_time':"True"}.items()
            )
    
    
    '''
    handler_event_launch_navigation = RegisterEventHandler(
        event_handler=OnProcessExit(
        target_action=launch_navigation,
        on_exit=[EmitEvent(event=Shutdown())],
        )
        )
    
    handler_event_slam = RegisterEventHandler(
        event_handler=OnProcessExit(
        target_action=handler_event_slam,
        on_exit=[EmitEvent(event=Shutdown())],
        )
        )
    
    
    ld.add_action(handler_event_slam)
    ld.add_action(handler_event_launch_navigation)
    '''
    ld.add_action(launch_navigation)
    ld.add_action(launch_to_slam)


    return ld


