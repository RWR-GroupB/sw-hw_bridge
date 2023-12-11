#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray

from motor_control.gripper_controller import GripperController

# Numbers correspond to position in list self.joint_angles and their range limits
hand_finger_joint_map = {
    'thumb' : {
        'thumb_adduction-abduction' : (0, (-35, 35)),
        'thumb_mcp' : (1, (0, 90)),
        'thumb_pip-dip': (2, (0, 90)),
    },

    'index': {
        'index_mcp': (3, (0, 90)),
        'index_pip-dip' : (4, (0, 90)),
    },

    'middle': {
        'middle_mcp' : (5, (0, 90)),
        'middle_pip-dip' : (6, (0, 90)),
    },

    'pinky' : {
        'pinky_mcp' : (7, (0, 90)),
        'pinky_pip-dip' : (8, (0, 90)),
    },
}

class SwHwBridgeNode:
    def __init__(self):
        # Setting up publishers
        self.get_motor_positions_pub = rospy.Publisher('hand/motors/get_motor_positions', Float32MultiArray, queue_size=1)

        # Setting up subscribers 
        self.string_subscriber = rospy.Subscriber('hand/motors/cmd_joint_angles', Float32MultiArray, self.cmd_joint_angles_callback)

        # Rate setup
        self.rate = rospy.Rate(20)  # Based on rate that Dynamixel motors communicate

        # Non-ROS setup
        self.gripper_controller = GripperController(port="/dev/ttyUSB0",calibration=True)
        self.number_of_joints = 9
        self.joint_angles = [0] * self.number_of_joints
        self.motor_positions = [0] * self.number_of_joints  # equal to number of motors 

    # --- Publisher stuff ---
    def run_publishers(self):
        while not rospy.is_shutdown():
            self.publish_get_motor_positions()

            self.rate.sleep()

    def publish_get_motor_positions(self):
        current_motor_pos = self.gripper_controller.get_motor_pos()
        self.motor_positions = current_motor_pos

        motor_positions_msg = Float32MultiArray()
        motor_positions_msg.data = self.motor_positions
        self.get_motor_positions_pub.publish(motor_positions_msg)

    # --- Subscriber stuff ---
    def cmd_joint_angles_callback(self, msg: Float32MultiArray):
        self.joint_angles = msg.data

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