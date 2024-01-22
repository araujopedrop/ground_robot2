#!/usr/bin/python3





import os
import time
import rclpy
import launch

import asyncio
import threading
import subprocess

from rclpy.node                        import Node
from signal                            import SIGINT, SIGABRT
from std_msgs.msg                      import String
from my_robot_interfaces.srv           import CmdVehicle

from launch                            import LaunchService
from launch.actions                    import ExecuteProcess
from launch_ros.substitutions          import FindPackageShare
from launch                            import LaunchDescription
from launch.actions                    import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions              import PathJoinSubstitution, TextSubstitution
from ament_index_python.packages       import get_package_share_directory


class navigationManagerNode(Node):

    MAPPING  = "Map"
    SAVE_MAP = "Save Map"
    NAVIGATE = "Nav"

    LAST_CMD = ""

    VELOCITY = 1.0
    ACTION = 1

    ls_mapping    = LaunchService()
    ls_saving_map = None
    ls_navigate   = LaunchService()
    ld            = None
    package_name  = "ground_robot"

    launch_task = None

    def __init__(self):
        super().__init__("navigation_manager_node")

        self.cmd_service_ = self.create_service(CmdVehicle,"nav_cmd",self.check_cmd_test)
        self.pub_cmd_ = self.create_publisher(String,"last_cmd",10)
        self.timer_pub_ = self.create_timer(0.1,self.publish_last_cmd)

        self.LAST_CMD = "Modo normal"

        self.get_logger().info("nav_cmd service is up!")

        self.event_loop = asyncio.get_event_loop()

        self.event_loop.run_forever()

    def publish_last_cmd(self):
        msg = String()
        msg.data = "navigation_manager_node: " + self.LAST_CMD

        self.pub_cmd_.publish(msg)

    def check_cmd_test(self, request, response):
        response.result = "Mapeo"
        return response

    def check_cmd(self,request,response):

        cmd = request.command
        

        try:

            if cmd == self.MAPPING:
                self.launch_mapping_task()
                response.result = "Mapeo"
                return response

            elif self.is_save_cmd(cmd):
                map_name = self.get_map_name(cmd)
                response.result = self.launch_saving_map(map_name)
                return response

            elif cmd == self.NAVIGATE:
                response.result = self.launch_saving_map()
                return response

            else:
                self.LAST_CMD = "No se que me llego"
                self.get_logger().warn("check_cmd: Could not execute command! Command not available = " + str(cmd))
                response.result = "No se que me llego"

                return response
            
            #await asyncio.sleep(1.0)

        except Exception as e:
            self.get_logger().error("check_cmd: Could not execute command! Error: " + str(e))
            response.result = "Error"

            self.LAST_CMD = "Error"

            return response
        

    def launch_mapping_task(self):
        # This is a trampoline method to run the episode on the async event loop
        fut = asyncio.run_coroutine_threadsafe(self.launch_mapping(), self.event_loop)
        # block and wait for future to return a result
        return fut.result()
        
    async def launch_mapping(self):

        try: 

            # Op 1
            '''
            package_name = "ground_robot"

            ld = LaunchDescription()

            launch_to_import = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        FindPackageShare(package_name), '/launch', '/mapping.launch.py'])
                        #FindPackageShare(package_name), '/launch', '/test.launch.py'])
                )
            
            ld.add_action(launch_to_import)

            self.ls_mapping.include_launch_description(ld)

            
            #self.ls_mapping.run()

            '''

            # Op 2
            # self.launch_process = subprocess.Popen(["ros2", "launch", "ground_robot", "mapping.launch.py"], text=True)

            # Op 3
            '''
            ld = LaunchDescription()


            simulation = ExecuteProcess(
            cmd=['ros2', 'launch', 'ground_robot','mapping.launch.py']
            )

            ld.add_action(simulation)
            self.ls_mapping.include_launch_description(ld)
            self.ls_mapping.run()
            '''

            # Op 4
            
            package_name = "ground_robot"

            ld = LaunchDescription()

            launch_to_import = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        FindPackageShare(package_name), '/launch', '/mapping.launch.py'])
                )
            
            ld.add_action(launch_to_import)

            self.ls_mapping.include_launch_description(ld)
            self.launch_task = asyncio.create_task(self.ls_mapping.run_async(shutdown_when_idle=True))
            #self.ls_mapping.run()   
            
            
            
            self.LAST_CMD = "Mapeo"

            #return "Mapeo"

            return self.launch_task

        except Exception:

            self.LAST_CMD = "Error realizando Mapeo"

            return "Error Mapeo"

    def launch_saving_map(self, map_name):

        try:
        
            self.save_map_roslaunch(map_name)

            #self.save_map_func(map_name)

            self.LAST_CMD = "Mapa guardado"

            result = self.wait_until_map_is_created(map_name,get_package_share_directory(self.package_name) + "/maps", 3.0)

            if result:

                # -------------------- Closing mapping launch file --------------------

                try:
                    
                    self.terminate_mapping()

                    self.get_logger().warn("Mapping Launchfile was shutdowned")
                    
                    self.LAST_CMD = "Guardar mapa"

                    return "Guardando Mapa: " + map_name

                except Exception:

                    self.LAST_CMD = "Error realizando Guardado de mapa"

                    return "Error Guardando Mapa 1"
                
            else:
                
                self.terminate_mapping()
                
                self.get_logger().warn("Mapping Launchfile was shutdowned")

                self.LAST_CMD = "Guardar mapa"

                return "Error guardando mapa: " + map_name
            
        except Exception as e:

            self.LAST_CMD = "Error realizando Guardado de mapa"

            return "Error Guardando Mapa 2: " + str(e)

    def launch_navigate(self):

        try:

            # -------------------- Starting launch file for saving map --------------------

            self.ld = LaunchDescription()

            launch_to_import = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        FindPackageShare(self.package_name), '/launch', '/navigate.launch.py'])
                )
            
            self.ld.add_action(launch_to_import)

            self.ls.include_launch_description(self.ld)

            self.ls.run()

            self.LAST_CMD = "Navegando"

            return "Navigating"

        except Exception:

            self.LAST_CMD = "Error realizando Guardado de mapa"

            return "Error in Navigating"

    def is_save_cmd(self, cmd):
        if cmd.find(self.SAVE_MAP + "->") != -1:
            return True
        return False
    
    def get_map_name(self, cmd: str):
        #map_name = cmd[len(self.SAVE_MAP + "->"):len(cmd)]
        
        map_name = cmd.split("->")[1]
        return map_name

    def wait_until_map_is_created(self, map_name: str, path: str, timeout = 5.0):
        
        time.sleep(0.5)
        start_time = time.time()

        while self.check_if_name_exists(map_name, path) == False:
            end_time = time.time()
            if end_time - start_time > timeout:
                return False
            else:
                self.ls_saving_map.run()
                time.sleep(0.5)
                
        
        return True

    def check_if_name_exists(self, map_name: str, path: str):
        
        for elem in os.listdir(path):
            if elem.find(map_name + ".yaml") != -1:
                return True
        return False

    def save_map_func(self,map_name):
        self.launch_process2 = subprocess.Popen(["ros2", "launch", "ground_robot", "save_map.launch.py", "map_name:="+ map_name, ], text=True)

    def save_map_roslaunch(self,map_name):

        self.ls_saving_map = LaunchService()
        self.ld            = LaunchDescription()

        launch_to_import = IncludeLaunchDescription(
                                PythonLaunchDescriptionSource([
                                    FindPackageShare(self.package_name), 
                                        '/launch', 
                                        '/save_map.launch.py']),
                                    launch_arguments={
                                        'map_name': map_name
                                    }.items())

        self.ld.add_action(launch_to_import)

        self.ls_saving_map.include_launch_description(self.ld)

        val = self.ls_saving_map.run()

    def terminate_mapping(self):
        self.ls_mapping.shutdown()
        self.ls_saving_map.shutdown()
        self.launch_process.send_signal(SIGINT)
        #self.launch_process.terminate()
        self.launch_process.wait(timeout=30)
        #while self.launch_process.poll() is None:
        #    self.launch_process.wait(timeout=30)
        #    self.launch_process.kill()

def main(args=None):
    rclpy.init(args=args)
    node = navigationManagerNode() 
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
