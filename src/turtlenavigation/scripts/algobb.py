#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    x=1

def algo():
 #rrt
    rospy.init_node('rrtalgo', anonymous=True)
    rospy.Subscriber("obstacle_vals", Float64MultiArray,callback)
    pub = rospy.Publisher("path_array", Float64MultiArray , queue_size=10)
    
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        arr = Float64MultiArray()
        # calcute arr using sub
        arr.data = [-1,-1,-2,-2,-3,-3]
        pub.publish(arr)
        rate.sleep()
    rospy.spin()    


if __name__ == "__main__":
    try:
        algo()
    except rospy.ROSInterruptException:
        pass