#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-
import numpy as np
import rospy
import tf
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan

import dwa

from threading import Lock, Thread
import time
import math

ROBOT_TF_NAME = "/robot_base"  # "/robot_tf"
MAP_TOPIC_NAME = "/map"  # "/map"

def limitVal(minV, maxV, v):
    if v < minV:
        return minV
    if v > maxV:
        return maxV
    return v


class LocalPlanner:
    def __init__(self):
        self.arrive = 0.1
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.vw = 0.0
        # init plan_config for once
        self.laser_lock = Lock()

        self.plan_config = dwa.Config()
        self.plan_config.robot_type = dwa.RobotType.rectangle
        self.dwa = dwa.DWA(self.plan_config)
        c = self.plan_config
        self.threshold = 2.5

        self.path = Path()
        self.tf = tf.TransformListener()
        self.path_sub = rospy.Subscriber('/course_agv/global_path', Path, self.pathCallback)
        self.vel_pub = rospy.Publisher('/course_agv/velocity', Twist, queue_size=1)
        self.path_pub = rospy.Publisher('/course_agv/local_path', Path, queue_size=1)
        self.midpose_pub = rospy.Publisher('/course_agv/mid_goal', PoseStamped, queue_size=1)
        self.laser_sub = rospy.Subscriber('/course_agv/laser/scan', LaserScan, self.laserCallback)
        self.map_sub = rospy.Subscriber(MAP_TOPIC_NAME, OccupancyGrid, self.mapCallback)
        self.planner_thread = None
        self.ob = None

        self.plan_rx = []
        self.plan_ry = []
        self.my_path = []

        self.need_exit = False
        pass

    def updateGlobalPose(self):
        try:
            self.tf.waitForTransform("/map", ROBOT_TF_NAME, rospy.Time(), rospy.Duration(4))
            (self.trans, self.rot) = self.tf.lookupTransform('/map', ROBOT_TF_NAME, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("get tf error!")
        euler = tf.transformations.euler_from_quaternion(self.rot)
        roll, pitch, yaw = euler[0], euler[1], euler[2]
        self.x = self.trans[0]
        self.y = self.trans[1]
        self.yaw = yaw

        # print("original length: ", len(self.path.poses))
        self.my_path = [[self.path.poses[0].pose.position.x, self.path.poses[0].pose.position.y]]
        for i in range(len(self.path.poses) - 1):
            cur_p = self.path.poses[i].pose.position
            next_p = self.path.poses[i+1].pose.position
            new_p = cur_p
            angle = np.arctan2(next_p.y - cur_p.y, next_p.x - cur_p.x)
            while math.hypot(next_p.x - new_p.x, next_p.y - new_p.y) > 1:
                new_p.x += np.cos(angle)
                new_p.y += np.sin(angle)
                self.my_path.append([new_p.x, new_p.y])
            self.my_path.append([next_p.x, next_p.y])
        self.path.poses = []
        for i in range(len(self.my_path)):
            pose = PoseStamped()
            pose.header.seq = i
            pose.header.stamp = rospy.Time(0)
            pose.header.frame_id = 'map'
            pose.pose.position.x = self.my_path[i][0]
            pose.pose.position.y = self.my_path[i][1]
            pose.pose.position.z = 0.01
            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 0
            pose.pose.orientation.z = 0
            pose.pose.orientation.w = 1
            self.path.poses.append(pose)
        # print("fixed length: ", len(self.path.poses))

        ind = self.goal_index
        self.goal_index = len(self.path.poses) - 1
        while ind < len(self.path.poses):
            p = self.path.poses[ind].pose.position
            dis = math.hypot(p.x - self.x, p.y - self.y)
            if dis < self.threshold:
                # print("ind: ", ind)
                # print("dis: ", dis)
                # print("p: ", p.x, self.x, p.y, self.y)
                self.goal_index = ind
            ind += 1
        # print("goal: ", self.goal_index)
        goal = self.path.poses[self.goal_index]

        self.midpose_pub.publish(goal)
        lgoal = self.tf.transformPose(ROBOT_TF_NAME, goal)
        self.plan_goal = np.array([lgoal.pose.position.x, lgoal.pose.position.y])
        self.goal_dis = math.hypot(self.x - self.path.poses[-1].pose.position.x,
                                   self.y - self.path.poses[-1].pose.position.y)

    def laserCallback(self, msg):
        # print("get laser msg!!!!",msg)
        self.laser_lock.acquire()
        # preprocess
        self.ob = [[100, 100]]
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        for i in range(len(msg.ranges)):
            a = angle_min + angle_increment * i
            r = msg.ranges[i]
            if r < self.threshold:
                self.ob.append([math.cos(a) * r, math.sin(a) * r])
        self.laser_lock.release()
        pass

    def updateObstacle(self):
        self.laser_lock.acquire()
        self.plan_ob = []
        self.plan_ob = np.array(self.ob) if self.ob is not None else None
        self.laser_lock.release()
        pass

    def pathCallback(self, msg):
        self.need_exit = True
        time.sleep(0.1)
        self.path = msg
        self.planner_thread = Thread(target=self.planThreadFunc)
        self.initPlanning()
        self.planner_thread.start()

    def initPlanning(self):
        self.goal_index = 1
        self.updateGlobalPose()
        cx = []
        cy = []
        for pose in self.path.poses:
            cx.append(pose.pose.position.x)
            cy.append(pose.pose.position.y)
        self.goal = np.array([cx[0], cy[0]])
        self.plan_cx, self.plan_cy = np.array(cx), np.array(cy)
        self.plan_goal = np.array([cx[-1], cy[-1]])
        self.plan_x = np.array([0.0, 0.0, 0.0, self.vx, self.vw])
        pass

    def planThreadFunc(self):
        print("running planning thread!!")
        rr = rospy.Rate(10)
        self.need_exit = False
        while not self.need_exit:
            self.planOnce()
            if self.goal_dis < self.arrive:
                print("arrive goal!")
                break
            rr.sleep()
        print("exit planning thread!!")
        self.publishVel(True)
        self.planner_thread = None
        pass

    def planOnce(self):
        self.updateGlobalPose()
        # Update plan_x [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
        self.plan_x = [0.0, 0.0, 0.0, self.vx, self.vw]
        # Update obstacle
        self.updateObstacle()
        u, trajectory = self.dwa.plan(self.plan_x, self.plan_goal, self.plan_ob, self.goal_dis)
        self.plan_rx = []
        self.plan_ry = []
        for i, state in enumerate(trajectory):
            self.plan_rx.append(state[0])
            self.plan_ry.append(state[1])
        # filter
        alpha = 0.5
        self.vx = u[3] * alpha + self.vx * (1 - alpha)
        self.vw = u[4] * alpha + self.vw * (1 - alpha)
        # print("mdbg; ",u)
        self.publishVel()
        self.publishPath()
        pass

    def publishVel(self, zero=False):
        if zero:
            self.vx = 0
            self.vw = 0
        cmd = Twist()
        cmd.linear.x = self.vx
        cmd.angular.z = self.vw
        self.vel_pub.publish(cmd)

    def publishPath(self):
        path = Path()
        path.header.seq = 0
        path.header.stamp = rospy.Time(0)
        path.header.frame_id = ROBOT_TF_NAME
        for i in range(len(self.plan_rx)):
            pose = PoseStamped()
            pose.header.seq = i
            pose.header.stamp = rospy.Time(0)
            pose.header.frame_id = ROBOT_TF_NAME
            pose.pose.position.x = self.plan_rx[i]
            pose.pose.position.y = self.plan_ry[i]
            pose.pose.position.z = 0.01
            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 0
            pose.pose.orientation.z = 0
            pose.pose.orientation.w = 1
            path.poses.append(pose)
        self.path_pub.publish(path)
        
    def mapCallback(self, msg):
        map_data = np.array(msg.data).reshape((-1, msg.info.height)).transpose()
        ox, oy = np.nonzero(map_data > 50)
        obstacle_x = (ox * msg.info.resolution + msg.info.origin.position.x).tolist()
        obstacle_y = (oy * msg.info.resolution + msg.info.origin.position.y).tolist()
        self.ob = np.vstack((obstacle_x, obstacle_y)).T

def main():
    rospy.init_node('path_Planning')
    lp = LocalPlanner()
    rospy.spin()
    pass


if __name__ == '__main__':
    main()
