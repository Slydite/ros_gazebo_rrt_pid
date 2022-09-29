#!/usr/bin/env python
import sys
import rospy
from std_msgs.msg import Float64MultiArray


def talker(endX,endY):
    pub = rospy.Publisher("obstacle_vals", Float64MultiArray, queue_size=10)
    rospy.init_node("obstacle_coords", anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        arr = Float64MultiArray()
        arr.data = [
            float(endX),
            float(endY),
            0,
            1.5,
            0,
            3,
            0,
            4.5,
            1.5,
            0,
            1.5,
            1.5,
            1.5,
            3,
            1.5,
            4.5,
            3,
            0,
            3,
            1.5,
            3,
            3,
            3,
            4.5,
            4.5,
            0,
            4.5,
            1.5,
            4.5,
            3,
            4.5,
            4.5,
        ]
        pub.publish(arr)
        rate.sleep()


if __name__ == "__main__":
    try:
        talker(sys.argv[1],sys.argv[2])
    except rospy.ROSInterruptException:
        pass

