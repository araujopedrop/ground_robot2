import os
import launch.actions

from launch import LaunchDescription
from launch_ros.actions import Node 
from launch.actions import ExecuteProcess

# Retrieving path information 
from ament_index_python.packages import get_package_share_directory
from pathlib import Path 

# Utilizing launch files from other packages
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# Working with environment variables
from launch.actions import SetEnvironmentVariable

# Simulation event handling 
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown

from launch.substitutions import ThisLaunchFileDir
from launch.substitutions import LaunchConfiguration

from launch.actions                    import IncludeLaunchDescription
from launch_ros.substitutions          import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


package_name = "ground_robot"


def generate_launch_description():

    ld = LaunchDescription()


    action_print = launch.actions.LogInfo(msg='Hello World!')

    ld.add_action(action_print)

    return ld


