import os

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

from launch_ros.substitutions          import FindPackageShare

package_name = "ground_robot"

# Path Variables 
ignition_ros_package_path  = get_package_share_directory("ros_gz_sim")
package_name_path        = get_package_share_directory(package_name)
simulation_world_file_path = Path(package_name_path, "worlds/world_land_robot.sdf").as_posix()
simulation_models_path     = Path(package_name_path, "models").as_posix()

simulation = ExecuteProcess(
    cmd=['ign', 'gazebo', '-r', simulation_world_file_path],
    output='screen'
)


def generate_launch_description():
    ld = LaunchDescription()

    teleop_GUI = Node(
        package=package_name,
        executable="teleop_GUI.py",
        name="teleop_GUI"
    )

    teleop_key_client_node = Node(
        package=package_name,
        executable="teleop_client.py",
        name="teleop_key"
    )

    teleop_key_server_node = Node(
        package=package_name,
        executable="teleop_server.py",
        name="teleop_key"
    )

    robot_state_publisher_node = Node(
        package=package_name,
        executable="robot_state_publisher.py",
        name="robot_state_publisher"
    )

    navigation_manager_node = Node(
        package=package_name,
        executable="navigation_manager.py",
        name="navigation_manager"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d' + os.path.join(package_name_path, 'config', 'rviz_config.rviz')], 
        remappings=[],
        output="screen" 
    )

    '''
    rosbrigde_server = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([FindPackageShare("rosbridge_server"), '/launch/', "rosbridge_websocket_launch.xml"])
            )
    '''

    #/TOPIC@ROS_MSG@IGN_MSG
    bridge_node = Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        arguments=["/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist",
                   "/camera@sensor_msgs/msg/Image@ignition.msgs.Image",
                   "cmd_camera@std_msgs/msg/Float64@ignition.msgs.Double",
                   "/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
                   "/model/land_robot/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
                   "/model/land_robot/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
                   "/world/visualize_lidar_world/model/land_robot/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model"], 

        remappings=[("/camera","/camera/image_raw"),("/lidar","/scan_"),("/model/land_robot/odometry","/odom"),("/model/land_robot/tf","/tf_"),("/world/visualize_lidar_world/model/land_robot/joint_state","/joint_states")],
        output="screen"
    )

    set_env = SetEnvironmentVariable(
    name="IGN_GAZEBO_RESOURCE_PATH",
    value=simulation_models_path
    )

    simulation = ExecuteProcess(
    cmd=['ign', 'gazebo', '-r', simulation_world_file_path],
    output='screen'
    )

    rosbrigde_server = ExecuteProcess(
    cmd=['ros2', 'launch', 'rosbridge_server', "rosbridge_websocket_launch.xml"],
    output='screen'
    )
        
    #Para parar el roslaunch al cerrar la simulacion
    handler_event_sim = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=simulation,
            on_exit=[EmitEvent(event=Shutdown())],
        )
    )

    handler_event_rosbridge = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=bridge_node,
            on_exit=[EmitEvent(event=Shutdown())],
        )
    )

    ld.add_action(teleop_GUI)
    ld.add_action(bridge_node)
    ld.add_action(handler_event_sim)
    ld.add_action(handler_event_rosbridge)
    ld.add_action(set_env)
    ld.add_action(simulation)
    ld.add_action(teleop_key_client_node)
    ld.add_action(teleop_key_server_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(navigation_manager_node)
    ld.add_action(rviz_node)
    ld.add_action(rosbrigde_server)
    

    return ld
