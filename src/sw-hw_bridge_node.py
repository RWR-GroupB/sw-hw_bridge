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
        self.rate = rospy.Rate(100)  # 100 Hz

        # Non-ROS setup
        self.gripper_controller = GripperController(port="/dev/ttyUSB0",calibration=False)

        self.joint_positions = [0, 0]

    # --- Publisher stuff ---
    def run_publishers(self):
        while not rospy.is_shutdown():
            self.publish_get_joint_angles()
            self.publish_get_motor_positions()
            self.publish_get_motor_statuses()

            self.rate.sleep()

    def publish_get_joint_angles(self):
        joint_angles_msg = Float32MultiArray()
        joint_angles_msg.data = [1, -1]
        self.get_joint_angles_pub.publish(joint_angles_msg)

    def publish_get_motor_positions(self):
        motor_positions_msg = Float32MultiArray()
        motor_positions_msg.data = self.gripper_controller.get_motor_pos()
        self.get_motor_positions_pub.publish(motor_positions_msg)

    def publish_get_motor_statuses(self):
        motor_statuses_msg = Float32MultiArray()
        motor_statuses_msg.data = [1, -1]
        self.get_motor_statuses_pub.publish(motor_statuses_msg)


    # --- Subscriber stuff ---
    def cmd_joint_angles_callback(self, msg):
        self.joint_positions = msg.data

        self.gripper_controller.write_desired_joint_angles(msg.data)
        rospy.loginfo(f"Commanding these joint angles: {msg.data}")
        # self.gripper_controller.wait_for_motion()


if __name__ == '__main__':
    rospy.init_node('sw_hw_bridge_node', anonymous=True)
    sw_hw_bridge_node = SwHwBridgeNode()

    rospy.loginfo("Software-Hardware Bridge node is running")

    try:
        sw_hw_bridge_node.run_publishers()
    except rospy.ROSInterruptException:
        pass