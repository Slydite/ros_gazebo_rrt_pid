# !/usr/bin/env python

import random
import math
import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
import matplotlib.pyplot as pl


class Algo:

    def __init__(self, start, goal, rate, step_len):
        self.start = Points(start[0], start[1])
        self.goal = Points(goal[0], goal[1])
        self.rate = rate
        self.step_len = step_len
        self.delta = 0.01
        self.botSize = math.sqrt(0.15**2 + 0.19**2)
        self.error = 0.25 + self.botSize
        self.obs = []
        self.boolGoal = False
        self.minStep = 0.076
        self.xBounds = (0, 6)
        self.yBounds = (0, 6)
        self.nodes = [self.start]

    def path(self):
        while not self.boolGoal:
            sample_node = self.custom()
            near_node = self.nodeJoiner(sample_node)
            step_node = self.deltaAdder(near_node, sample_node)
            if step_node and not self.collisionDetector(near_node, step_node):
                self.nodes.append(step_node)
                step_node.parent = near_node
                if math.slope(step_node.x - self.goal.x,
                              step_node.y - self.goal.y) <= 0.5:
                    self.boolGoal = True
                    self.goal.parent = step_node
        path = self.getPath(self.nodes)
        return path

    @staticmethod
    def getPath(nodes_list):
        node_temp = nodes_list[-1]
        path = []

        while node_temp.parent:
            path.append(node_temp)
            node_temp = node_temp.parent

        return path

    def collisionDetector(self, start, end):
        if self.intersectingObject(start) or self.intersectingObject(end):

            return True

        o, d = self.get_ray(start, end)
        e = [end.x, end.y]

        for (x, y, r) in self.obs:
            if self.circleCollision(o, e, d, [x, y], r):
                return True

    def deltaAdder(self, start_node, goal_node):
        dist, theta = self.find_dist_angle(start_node, goal_node)
        dist = min(dist, self.step_len)
        dist = max(dist, self.minStep)
        new_x = start_node.x + dist * math.cos(theta)
        new_y = start_node.y + dist * math.sin(theta)

        return Points(new_x, new_y)

    def circleCollision(self, o, e, d, a, r):
        d_perp = [d[1], -d[0]]

        v1 = [e[0] - a[0], e[1] - a[1]]
        v2 = [o[0] - a[0], o[1] - a[1]]

        t1 = d_perp[0] * v1[1] - d_perp[1] * v1[0]
        t2 = d_perp[0] * v2[1] - d_perp[1] * v2[0]

        if d[0] != 0:

            if t1 * t2 > 0:
                return False

            else:
                m = d[1] / d[0]
                c = o[1] - m * o[0]

                dist = (a[1] - m * a[0] - c) / math.slope(1, m)

                if dist <= self.error:
                    return True

        else:
            if t1 * t2 > 0:
                return False

            else:
                dist = (o[0] - a[0])

                if math.fabs(dist) <= self.error:
                    return True

        return False

    def custom(self):
        if np.random.random() > self.rate:
            x = random.uniform(self.xBounds[0] + self.delta,
                               self.xBounds[1] - self.delta)
            y = random.uniform(self.yBounds[0] + self.delta,
                               self.yBounds[1] - self.delta)

            return Points(x, y)

        return self.goal

    def intersectingObject(self, node):

        for (x, y, r) in self.obs:
            if math.slope(node.x - x, node.y - y) <= self.error:
                return True

        return False

    def nodeJoiner(self, node):
        # print(self.nodes_list)
        distances = [
            math.slope(nd.x - node.x, nd.y - node.y) for nd in self.nodes
        ]
        return self.nodes[np.argmin(distances)]

    @staticmethod
    def find_dist_angle(start_node, goal_node):
        dist = math.slope(start_node.x - goal_node.x,
                          start_node.y - goal_node.y)
        theta = math.atan2(goal_node.y - start_node.y,
                           goal_node.x - start_node.x)

        return dist, theta

    @staticmethod
    def get_ray(start, end):
        orig = [start.x, start.y]
        direc = [end.x - start.x, end.y - start.y]
        return orig, direc


class Planner:

    def __init__(self):
        self.sub = rospy.init_node("rrtalgo", anonymous=True)
        self.sub = rospy.Subscriber("/obstacle_vals", Float64MultiArray,
                                    self.callback)
        self.pub = rospy.Publisher("/path_vals",
                                   Float64MultiArray,
                                   queue_size=10)
        self.rrt = None

        self.step = 0.5

        self.goal_sample_rate = 0.95
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:

                self.pub.publish(self.path)
                rate.sleep()
            except Exception as e:
                print(e)
                pass
        rospy.spin()

    def callback(self, data):
        obstacle_vals = data.data
        self.goal = Points(6, 6)
        self.start = Points(1, 1)

        if not self.rrt:

            self.rrt = Algo((self.goal.x, self.goal.y),
                            (self.start.x, self.start.y),
                            self.goal_sample_rate, self.step)
            for i in range(0, len(obstacle_vals) - 1, 2):
                self.rrt.obs.append(
                    (obstacle_vals[i], obstacle_vals[i + 1], 0.25))
            path_arr = self.rrt.path()
            path_arr.append(self.goal)
            path_arr.insert(0, self.start)
            path_arr_temp = []
            for i in path_arr:
                path_arr_temp.append(i.x)
                path_arr_temp.append(i.y)
            self.path = Float64MultiArray()
            self.path.data = path_arr_temp

        for i in range(0, len(obstacle_vals) - 1, 2):
            self.rrt.obs.append((obstacle_vals[i], obstacle_vals[i + 1], 0.25))


class Points:

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None

    def __repr__(self):
        return f"{self.x}, {self.y}"

    def __str__(self):
        return self.__repr__()


if __name__ == "__main__":
    try:
        planner = Planner()
    except rospy.ROSInterruptException:
        pass
