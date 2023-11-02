#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray

def SwHwBridge():
    rospy.init_node('sw-hw_bridge', anonymous=True)
    joint_angles_pub = rospy.Publisher('hand/motors/get_joint_angles', Float32MultiArray, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        joint_angles_msg = Float32MultiArray()
        joint_angles_msg.data = [1.0, 2.0]
         # Publish the message

        rospy.loginfo("Published array: %s", joint_angles_msg.data)
        joint_angles_pub.publish(joint_angles_msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        SwHwBridge()
    except rospy.ROSInterruptException:
        pass