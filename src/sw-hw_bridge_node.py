#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray

from motor_control.gripper_controller import GripperController

class SwHwBridgeNode:
    def __init__(self):
        # Setting up publishers
        self.get_joint_angles_pub = rospy.Publisher('hand/motors/get_joint_angles', Float32MultiArray, queue_size=1)
        self.get_motor_positions_pub = rospy.Publisher('hand/motors/get_motor_positions', Float32MultiArray, queue_size=1)
        self.get_motor_statuses_pub = rospy.Publisher('hand/motors/get_motor_statuses', Float32MultiArray, queue_size=1)

        # Setting up subscribers 
        self.string_subscriber = rospy.Subscriber('hand/motors/cmd_joint_angles', Float32MultiArray, self.cmd_joint_angles_callback)

        # Rate setup
        self.rate = rospy.Rate(20)  # Based on rate that Dynamixel motors communicate

        # Non-ROS setup
        self.gripper_controller = GripperController(port="/dev/ttyUSB0",calibration=True)

        self.joint_positions = [0, 0]
        self.joint_angles = [
            0.0,    # Thumb Flexion/Extension
            0.0,    # Thumb Adduction/Abduction
            0.0,    # Thumb MCP
            0.0,    # Thumb PIP/DIP
            0.0,    # Index MCP
            0.0,    # Index PIP/DIP
            0.0,    # Middle MCP
            0.0,    # Middle PIP/DIP
            0.0,    # Pinky MCP
            0.0,    # Pinky PIP/DIP
        ]

    # --- Publisher stuff ---
    def run_publishers(self):
        while not rospy.is_shutdown():
            self.publish_get_joint_angles()
            self.publish_get_motor_positions()
            self.publish_get_motor_statuses()

            self.rate.sleep()

    def publish_get_joint_angles(self):
        # TODO update based on new information
        current_motor_pos = self.gripper_controller.get_motor_pos()
        self.joint_angles[0] = current_motor_pos[0]
        self.joint_angles[1] = current_motor_pos[1]

        joint_angles_msg = Float32MultiArray()
        joint_angles_msg.data = self.joint_angles
        self.get_joint_angles_pub.publish(joint_angles_msg)

    def publish_get_motor_positions(self):
        # TODO update based on improved motor mapping 
        current_motor_pos = self.gripper_controller.get_motor_pos()
        self.joint_angles[0] = current_motor_pos[0]
        self.joint_angles[1] = current_motor_pos[1]

        motor_positions_msg = Float32MultiArray()
        motor_positions_msg.data = self.joint_angles
        self.get_motor_positions_pub.publish(motor_positions_msg)

    def publish_get_motor_statuses(self):
        motor_statuses_msg = Float32MultiArray()
        motor_statuses_msg.data = [1, -1]
        self.get_motor_statuses_pub.publish(motor_statuses_msg)


    # --- Subscriber stuff ---
    def cmd_joint_angles_callback(self, msg):
        # self.joint_positions = msg.data

        self.joint_angles[0] = msg.data[4]
        self.joint_angles[1] = msg.data[5]
        self.joint_angles[2] = msg.data[6]
        self.joint_angles[3] = msg.data[7]
        self.joint_angles[4] = msg.data[8]
        self.joint_angles[5] = msg.data[9]

        self.gripper_controller.write_desired_joint_angles(self.joint_angles)
        rospy.loginfo(f"Commanding these joint angles: {self.joint_angles}")
        # self.gripper_controller.wait_for_motion()


if __name__ == '__main__':
    rospy.init_node('sw_hw_bridge_node', anonymous=True)
    sw_hw_bridge_node = SwHwBridgeNode()

    rospy.loginfo("Software-Hardware Bridge node is running")

    try:
        sw_hw_bridge_node.run_publishers()
    except rospy.ROSInterruptException:
        pass