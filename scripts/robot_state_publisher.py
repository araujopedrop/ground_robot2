#!/usr/bin/python3

import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.qos import QoSProfile
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler, euler_from_quaternion

from pathlib import Path
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory

class RobotStatePublisher(Node):

    static_joints = []
    links = {}
    msg_lidar = LaserScan()

    def __init__(self):
        super().__init__("robot_state_publisher")
        
        # TF topic
        self.pub_tf = self.create_publisher(TFMessage,"/tf",10)
        self.sub_tf = self.create_subscription(TFMessage,"/tf_",self.subscriber_callback_tf,10)

        # Joints state topic
        self.sub_js = self.create_subscription(JointState,"/joint_states",self.subscriber_joint_state_callback,10)
        
        # Get links and joints from sdf
        package_name_path  = get_package_share_directory("ground_robot")
        sdf_file_path      = Path(package_name_path, "models/land_robot/model.sdf").as_posix()

        self.joints          = self.get_joints_from_sdf(sdf_file_path)
        self.static_joints   = self.get_fixed_joints(self.joints)
        self.dinamics_joints = self.get_non_fixed_joints(self.joints)
        self.links           = self.get_links_data_from_sdf(sdf_file_path)

        # Publish TF_Static topic
        self.publish_tf_static()

        # Scan topic
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)
        self.sub_scan = self.create_subscription(LaserScan,"/scan_",self.subcriber_callback_lidar,10)
        self.pub_scan = self.create_publisher(LaserScan,"/scan", qos_profile=qos_policy)




        self.get_logger().info("robot_state_publisher is up!")
        


    def subscriber_joint_state_callback(self, msg: JointState):
        '''
            Gets non fixed joints
            EJ:
            header:
                stamp:
                    sec: 7
                    nanosec: 194000000
                frame_id: ''
            name:
            - base_link_JOINT_rueda_der_tras
            - base_link_JOINT_rueda_der_del
            - base_link_JOINT_rueda_izq_tras
            position:
            - 3.1792388252282815
            - 5.914497722425877
            - 3.765506316134049
            velocity:
            - -0.0005288435865165663
            - -1.245119519183669e-10
            - 5.619567410381138e-06
            effort:
            - 0.0
            - 0.0
            - 0.0
            ---
        '''

        joints_name   = []
        joints_angles = []
        joints_dict   = {}

        joints_name   = msg.name
        joints_angles = msg.position

        for index in range(0,len(joints_name)):
            joints_dict[joints_name[index]] = joints_angles[index]

            msg = TFMessage()
            static_transformStamped = TransformStamped()

            parent  = self.dinamics_joints[joints_name[index]][0]
            child   = self.dinamics_joints[joints_name[index]][1]
            axis    = float(self.dinamics_joints[joints_name[index]][2][2])

            if abs(axis) > 1:
                axis /= abs(axis)

            ang_inc = joints_angles[index]*axis

            list_diff = self.get_diff(parent,child,self.links)

            roll, pitch, yaw = euler_from_quaternion([list_diff[3], list_diff[4], list_diff[5], list_diff[6]])

            static_transformStamped.header.stamp = self.get_clock().now().to_msg()
            static_transformStamped.header.frame_id = parent
            static_transformStamped.child_frame_id  = child

            if roll == 0 and pitch ==0 and yaw == 0:
                # Si coincide la orientacion del padre con la del hijo
                x, y, z, w = quaternion_from_euler(roll, pitch, yaw + ang_inc)
                
                static_transformStamped.transform.translation.x = list_diff[0]
                static_transformStamped.transform.translation.y = list_diff[1]
                static_transformStamped.transform.translation.z = list_diff[2]

            else:
                # Si NO coincide la orientacion del padre con la del hijo
                # Entiendo que la orientacion y traslacion es la del hijo con respecto al padre.
                # Asi esta hardcodeado mal, habria que ver como realizar bien la funcionalidad
                static_transformStamped.transform.translation.x = list_diff[1]
                static_transformStamped.transform.translation.y = list_diff[0]
                static_transformStamped.transform.translation.z = list_diff[2]

                x, y, z, w = quaternion_from_euler(roll, pitch + ang_inc*-1, yaw)


            static_transformStamped.transform.rotation.x = x
            static_transformStamped.transform.rotation.y = y
            static_transformStamped.transform.rotation.z = z
            static_transformStamped.transform.rotation.w = w
            
            msg.transforms = [static_transformStamped]
            self.pub_tf.publish(msg)

    def subscriber_callback_tf(self,msg: TFMessage):
        
        tf_msg_received = msg.transforms[0]
        tf_msg_to_send = TransformStamped()

        tf_msg_to_send = tf_msg_received
        tf_msg_to_send.header.frame_id = "odom"
        tf_msg_to_send.child_frame_id  = "base_footprint"
        
        msg.transforms = [tf_msg_to_send]
        self.pub_tf.publish(msg)

    def subcriber_callback_lidar(self,msg: LaserScan):

        #msg.header.frame_id = "base_scan"
        #self.msg_lidar = msg
        msg.header.frame_id = "lidar"
        self.pub_scan.publish(msg)

    def publish_lidar_scan(self):
        self.pub_scan.publish(self.msg_lidar)

    def get_joints_from_sdf(self,sdf_file_path):
        '''
            Gets all the joints form a sdf file
            Returns a dict {name:[link_parent, link_child, axis, type], name:[link_parent, link_child, axis, type], ...}
        '''

        joints_dict = {}
        joint_info = []

        tree = ET.parse(sdf_file_path)
        root = tree.getroot()
        model = root.find("model")
        joints = model.findall("joint")

        for joint in joints:
            name        = joint.attrib["name"]
            link_parent = joint.find("parent").text
            link_child  = joint.find("child").text
            axis_tag    = joint.find("axis")
            if axis_tag != None:
                axis    = joint.find("axis").find("xyz").text.split(" ")
            else:
                axis    = [0.0, 0.0, 0.0]
            type        = joint.attrib["type"]
            joints_dict[name] = [link_parent, link_child, axis, type]

        return joints_dict
    
    def get_fixed_joints_from_sdf(self,sdf_file):
        '''
            Gets all the fixed joints form a sdf file
            Then used to publish in tf static
            Returns a list of lists [[link_parent, link_child, pose], [link_parent, link_child, pose], [link_parent, link_child, pose], ... ]
        '''
        
        fixed_joint = []
        fixed_joints = []

        tree = ET.parse(sdf_file)
        root = tree.getroot()
        model = root.find("model")
        joints = model.findall("joint")

        for joint in joints:
            if joint.attrib["type"] == "fixed":
                link_parent = joint.find("parent").text
                link_child = joint.find("child").text
                pose = joint.find("pose").text            #str
                fixed_joint.append(link_parent)
                fixed_joint.append(link_child)
                fixed_joint.append(pose)
                fixed_joints.append(fixed_joint)
                fixed_joint = []

        return fixed_joints

    def get_fixed_joints(self, joints: dict):
        '''
            Gets all the fixed joints form a dict of joints
            Then used to publish in tf static
            Returns a list of lists [[link_parent, link_child], [link_parent, link_child], [link_parent, link_child], ... ]
        '''

        fixed_joint  = []
        fixed_joints = []

        for joint_item in joints.values():
            if joint_item[3] == "fixed":
                fixed_joint = [joint_item[0], joint_item[1]]
                fixed_joints.append(fixed_joint)
        
        return fixed_joints

    def get_non_fixed_joints(self, joints: dict):
        '''
            Gets all the non fixed joints form a dict of joints
            Then used to publish in tf static
            Returns a dict: {joint_name: [parent, child, axis, type], joint_name: [parent, child, axis, type], ...}
            Ej: { base_link_JOINT_rueda_der_tras: ['base_link', 'rueda_der_tras', ['0', '0', '-1'], 'revolute'], ...}
        '''

        joints_dict_result = {}

        for key in joints:
            if joints[key][3] != "fixed":
                joints_dict_result[key] = joints[key]

        return joints_dict_result

    def get_links_data_from_sdf(self,sdf_file):
        '''
            links_dict = {link_name: link_pose, link_name: link_pose, ...}
            Ej link_pose: <pose>-0.437626 -0.571744 -0.298299 1.5708 -0 0</pose>
        '''

        links_dict = {}

        tree  = ET.parse(sdf_file)
        root  = tree.getroot()
        model = root.find("model")
        links = model.findall("link")

        for link in links:
            link_name = link.attrib["name"]
            link_pose = link.find("pose").text
            links_dict[link_name] = link_pose

        return links_dict
    
    def get_diff(self, parent, child, links_data):

        list_diff = []
        list_diff_quaternion = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        parent_pose = links_data[parent].split(" ")
        child_pose = links_data[child].split(" ")

        for elem in range(0,len(parent_pose)):
            list_diff.append(float(child_pose[elem])-float(parent_pose[elem]))

        x, y, z, w = quaternion_from_euler(list_diff[3], list_diff[4], list_diff[5])

        list_diff_quaternion[0] = list_diff[0]
        list_diff_quaternion[1] = list_diff[1]
        list_diff_quaternion[2] = list_diff[2]
        list_diff_quaternion[3] = x
        list_diff_quaternion[4] = y
        list_diff_quaternion[5] = z
        list_diff_quaternion[6] = w

        return list_diff_quaternion

    def publish_tf_static(self):

        parent = ""
        child  = ""
        list_diff = []
        
        for joint in self.static_joints:
            
            static_transformStamped = TransformStamped()
            broadcaster = tf2_ros.StaticTransformBroadcaster(self)

            parent = joint[0]
            child  = joint[1]

            list_diff = self.get_diff(parent,child,self.links)

            static_transformStamped.header.stamp = self.get_clock().now().to_msg()
            static_transformStamped.header.frame_id = joint[0]
            static_transformStamped.child_frame_id  = joint[1]

            static_transformStamped.transform.translation.x = list_diff[0]
            static_transformStamped.transform.translation.y = list_diff[1]
            static_transformStamped.transform.translation.z = list_diff[2]
            
            static_transformStamped.transform.rotation.x = list_diff[3]
            static_transformStamped.transform.rotation.y = list_diff[4]
            static_transformStamped.transform.rotation.z = list_diff[5]
            static_transformStamped.transform.rotation.w = list_diff[6]

            broadcaster.sendTransform(static_transformStamped)



def main():
    rclpy.init()
    my_sub = RobotStatePublisher()
    try:
        rclpy.spin(my_sub)
    except KeyboardInterrupt:
        my_sub.destroy_node()
        rclpy.shutdown()
    

if __name__ == "__main__":
    main() 