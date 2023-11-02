#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray

class SwHwBridgeNode:
    def __init__(self):
        # Setting up publishers
        self.get_joint_angles_pub = rospy.Publisher('hand/motors/get_joint_angles', Float32MultiArray, queue_size=1)
        self.get_motor_statuses_pub = rospy.Publisher('hand/motors/get_motor_statuses', Float32MultiArray, queue_size=1)

        # Setting up subscribers 
        self.string_subscriber = rospy.Subscriber('hand/motors/cmd_joint_angles', Float32MultiArray, self.cmd_joint_angles_callback)
        
        # Rate setup
        self.rate = rospy.Rate(5)  # 5 Hz

        self.iterator, self.value1, self.value2 = 0, 0, 0
        self.motorValue1, self.motorValue2 = 10, -10

    # --- Publisher stuff ---
    def run_publishers(self):
        while not rospy.is_shutdown():
            self.publish_get_joint_angles()
            self.publish_get_motor_statuses()

            self.rate.sleep()

    def publish_get_joint_angles(self):
        joint_angles_msg = Float32MultiArray()
        joint_angles_msg.data = [self.value1, self.value2]

        self.get_joint_angles_pub.publish(joint_angles_msg)
        self.value1 += self.iterator
        self.value2 -= self.iterator
        self.iterator += 1

    def publish_get_motor_statuses(self):
        motor_statuses_msg = Float32MultiArray()
        motor_statuses_msg.data = [self.motorValue1, self.motorValue2]

        self.get_motor_statuses_pub.publish(motor_statuses_msg)
        self.value1 += self.iterator
        self.value2 -= self.iterator
        self.iterator += 1


    # --- Subscriber stuff ---
    def cmd_joint_angles_callback(self, msg):
        rospy.loginfo(f"Received: {msg.data}")
        # Here, you can add code to process the received message and possibly influence the publishing behavior.


if __name__ == '__main__':
    rospy.init_node('sw_hw_bridge_node', anonymous=True)
    node = SwHwBridgeNode()

    try:
        node.run_publishers()
    except rospy.ROSInterruptException:
        pass