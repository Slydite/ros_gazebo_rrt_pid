import rospy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
import math
import numpy as np
import matplotlib.pyplot as pl

import time


class Controller:

    def __init__(self):
        rospy.init_node("pid", anonymous=True)
        self.sub1 = rospy.Subscriber("odom", Odometry, self.callback1)
        self.sub2 = rospy.Subscriber("path_vals", Float64MultiArray,
                                     self.callback2)
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        vel_msg = Twist()
        rate = rospy.Rate(10)

        self.x = 0
        self.y = 0
        self.theta = 0

        self.path_array = []

        self.error = 0
        self.error_last = 0
        self.integral_error = 0
        self.derivative_error = 0
        self.output = 0
        self.max_omega = 0.2
        self.min_omega = -0.2

        self.max_speed = 0.2
        self.min_speed = -0.2

        self.rot_flag = True

        self.kp_dist = 0.002
        self.ki_dist = 0.001
        self.kd_dist = 0.001

        self.kp_ang = 0.01
        self.ki_ang = 0.001
        self.kd_ang = 0.01 #p down d increase

        while not rospy.is_shutdown():
            vel_msg = Twist()
            print(self.path_array)
            if self.path_array:
                print(self.rot_flag)
                print(f"yg {self.goal[1]} , yc {self.y} , xg {self.goal[0]}, xc {self.x}")
                print(
                    "Current angle", self.theta, " Wanted angle=",
                    math.degrees(
                        math.atan2(self.goal[1] - self.y,
                                   self.goal[0] - self.x)))
                print(
                    self.calculate_angle(
                        self.theta,
                        math.degrees(
                            math.atan2(self.goal[1] - self.y,
                                       self.goal[0] - self.x))))

                if math.fabs(self.theta - math.degrees(
                        math.atan2(self.goal[1] - self.y, self.goal[0] -
                                   self.x))) < 1:
                    self.rot_flag = False

                if math.fabs(self.theta - math.degrees(
                        math.atan2(self.goal[1] - self.y, self.goal[0] -
                                   self.x))) > 5:
                    self.rot_flag = True

                if math.slope(self.x - self.goal[0],
                              self.y - self.goal[1]) < 0.075:
                    if self.idx == len(self.path_array) - 1:
                        vel_msg = Twist()
                        self.pub.publish(vel_msg)
                        break
                    self.idx += 1

                    self.start = self.goal
                    self.goal = self.path_array[self.idx]
                    self.error = 0
                    self.error_last = 0
                    self.integral_error = 0
                    self.derivative_error = 0
                    #vel_msg = Twist()

                    self.rot_flag = True

                omega = self.compute_angle()
                print("aspeed=", omega)
                if self.rot_flag:
                    if omega > 4.3:
                        vel_msg.linear.x = 0
                        vel_msg.angular.z = 0
                        self.clear()

                    elif omega < -4.3:
                        vel_msg.linear.x = 0
                        vel_msg.angular.z = 0

                        self.clear()

                    else:
                        vel_msg.angular.z = omega

                if not self.rot_flag:

                    speed = self.compute_dist()
                    print("speed=", speed)
                    speed = abs(speed)
                    if speed > 0.22:
                        vel_msg.linear.x = 0
                        vel_msg.angular.z = 0
                        self.clear()

                    elif speed < -0.22:
                        vel_msg.linear.x = 0
                        vel_msg.angular.z = 0
                        self.clear()

                    else:
                        vel_msg.linear.x = speed
            print("cmd_vel vals", vel_msg.linear.x, vel_msg.angular.z)
            self.pub.publish(vel_msg)
            rate.sleep()

    def callback1(self, data):
        ox, oy, oz, ow = data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w

        x, y = data.pose.pose.position.x, data.pose.pose.position.y

        yaw_z = self.quaternion2euler(ox, oy, oz, ow)
        yaw_z = math.degrees(yaw_z)
        # if 90 <= yaw_z <= 0:
        #     yaw_z = 90 - yaw_z
        # if 180 <= yaw_z <= 90:
        #     yaw_z = 360 - (yaw_z - 90)
        # if 0 < yaw_z <= -90:
        #     yaw_z = (90 - yaw_z)
        # if -180 <= yaw_z < -90:
        #     yaw_z = (90 - yaw_z)
        self.x, self.y, self.theta = x, y, yaw_z

    def callback2(self, data):
        list_coords = []
        for i in range(0, len(data.data) - 1, 2):
            list_coords.append((data.data[i], data.data[i + 1]))

        if not self.path_array:
            self.path_array = list_coords.copy()
            self.idx = 0

            self.start = self.path_array[self.idx]
            self.idx += 1

            self.goal = self.path_array[self.idx]

    @staticmethod
    def quaternion2euler(ox, oy, oz, ow):
        t3 = +2.0 * (ow * oz + ox * oy)
        t4 = +1.0 - 2.0 * (oy * oy + oz * oz)
        yaw_z = math.atan2(t3, t4)

        return yaw_z

    def compute_angle(self):
        self.error = self.calculate_angle(
            self.theta,
            math.degrees(
                math.atan2(self.goal[1] - self.y, self.goal[0] - self.x)))
        self.integral_error += self.error
        self.derivative_error = self.error - self.error_last
        self.error_last = self.error
        output = self.kp_ang * self.error + self.ki_ang * \
            self.integral_error + self.kd_ang * self.derivative_error

        return output

    @staticmethod
    def calculate_angle(theta, goal_ang):
        #what
        if 90 <= goal_ang <= 180 and -180 <= theta <= -90:
            return -(180 - goal_ang + 180 - abs(theta))
        if 90 <= theta <= 180 and -180 <= goal_ang <= -90:
            return 180 - abs(goal_ang) + 180 - theta
        if goal_ang - theta > 180:
            return goal_ang - theta - 180
        if goal_ang - theta < -180:
            return goal_ang - theta + 180

        return goal_ang - theta

    def compute_dist(self):
        self.error = self.calculate_dist(
            self.x, self.y, self.goal,
            [self.goal[0] - self.start[0], self.goal[1] - self.start[1]])
        self.integral_error += self.error
        self.derivative_error = self.error - self.error_last
        self.error_last = self.error
        output = self.kp_dist * self.error + self.ki_dist * \
            self.integral_error + self.kd_dist * self.derivative_error

        return output

    @staticmethod
    def calculate_dist(x, y, goal, dir):
        dist = math.slope(x - goal[0], y - goal[1])

        vect1 = np.array([goal[0] - x, goal[1] - y]) / \
            np.linalg.norm([goal[0] - x, goal[1] - y])
        vect2 = np.array(dir) / np.linalg.norm(dir)

        if round(np.dot(vect1, vect2), 2) < 0:
            return -dist
        else:
            return dist

    def clear(self):
        print("CLEARED")
        self.error = 0
        self.error_last = 0
        self.integral_error = 0
        self.derivative_error = 0


def reset():
    rospy.init_node("pid", anonymous=True)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    vel_msg = Twist()

    while not rospy.is_shutdown():
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        print("resetting")
        pub.publish(vel_msg)
        break


if __name__ == '__main__':
    try:
        controller = Controller()
        #reset()
    except rospy.ROSInterruptException:
        pass