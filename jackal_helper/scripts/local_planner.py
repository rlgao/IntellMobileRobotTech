#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import numpy as np
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Path
from geometry_msgs.msg import Point, PoseStamped, Twist, Quaternion
from visualization_msgs.msg import Marker

# ========================================================
import use_dwa as dwa
# ========================================================

from threading import Thread
import time
from colorama import Fore, Back, Style

from my_gazebo_simulation import MyGazeboSimulation


def limitVal(minV, maxV, v):
    if v < minV:
        return minV
    if v > maxV:
        return maxV
    return v


class LocalPlanner:
    def __init__(self):
        self.gazebo_sim = MyGazeboSimulation()

        # state
        self.x   = 0.0
        self.y   = 0.0
        self.yaw = 0.0
        self.vx  = 0.0
        self.vw  = 0.0

        # ==============================================
        # tunable params
        # init plan_config for once
        self.plan_config = dwa.Config()
        self.plan_config.robot_type = dwa.RobotType.rectangle
        self.dwa = dwa.DWA(self.plan_config)
        c = self.plan_config

        self.arrive = 0.49
        self.ob_thresh = 1.2  # float("inf")  # 1.0
        # ==============================================

        self.path = Path()  # path from global planner, consisting of several way points
        self.marker_ob = Marker()
        self.marker_traj = Marker()

        self.path_sub = rospy.Subscriber('/my_planner/global_path', Path, self.pathCallback)
        self.midgoal_pub = rospy.Publisher('/my_planner/mid_goal', PoseStamped, queue_size=10)
        self.marker_ob_pub = rospy.Publisher("/my_planner/obstacles_laser", Marker, queue_size=10)
        self.marker_traj_pub = rospy.Publisher("/my_planner/predicted_traj", Marker, queue_size=10)

        self.planner_thread = None
        self.plan_ob = None

        self.received_global_path = False
        self.need_exit = False


    def updateGlobalPose(self):     
        # update x, y, yaw in world frame
        pos = self.gazebo_sim.get_model_state().pose.position
        ori = self.gazebo_sim.get_model_state().pose.orientation
        ori_list = [ori.x, ori.y, ori.z, ori.w]
        euler = euler_from_quaternion(ori_list)
        roll, pitch, yaw = euler[0], euler[1], euler[2]

        self.x = pos.x
        self.y = pos.y
        self.yaw = yaw


        # update self.goal_index
        idx = self.goal_index
        while idx < len(self.path.poses):
            p = self.path.poses[idx].pose.position
            dis = math.hypot(p.x - self.x, p.y - self.y)

            ################################################
            # if dis < self.threshold:
            if dis < self.arrive:
                self.goal_index = idx + 1
            ################################################
                
            idx += 1

        if self.goal_index > len(self.path.poses) - 1:
            self.goal_index = len(self.path.poses) - 1

        # current goal in world frame
        goal = self.path.poses[self.goal_index]
        self.midgoal_pub.publish(goal)
        
        goal_x = goal.pose.position.x
        goal_y = goal.pose.position.y
        # current goal in local frame
        lgoal_x =  (goal_x - self.x) * math.cos(self.yaw) + (goal_y - self.y) * math.sin(self.yaw)
        lgoal_y = -(goal_x - self.x) * math.sin(self.yaw) + (goal_y - self.y) * math.cos(self.yaw)
        self.plan_goal = np.array([lgoal_x, lgoal_y])

        self.goal_dis = math.hypot(self.x - self.path.poses[-1].pose.position.x,
                                   self.y - self.path.poses[-1].pose.position.y)


    def updateObstacle(self):
        laser_data = self.gazebo_sim.get_laser_scan()  # LaserScan
        # print('len(laser_data.ranges): %d' % (len(laser_data.ranges)))  # 720

        ob = []
        ob_Points = []

        angle_min = laser_data.angle_min
        angle_increment = laser_data.angle_increment

        # for i in range(len(laser_data.ranges)):
        i = 0
        interval = 30
        while i < len(laser_data.ranges):
            a = angle_min + angle_increment * i
            r = laser_data.ranges[i]

            ################################################
            if r < self.ob_thresh:
                x = math.cos(a) * r
                y = math.sin(a) * r
                ob.append([x, y])

                p = Point()
                p.x = x
                p.y = y
                p.z = 0
                ob_Points.append(p) 
            ################################################
            i += interval


        self.plan_ob = np.array(ob) if ob is not None else None

        # marker of obstacles
        self.marker_ob.header.frame_id = laser_data.header.frame_id
        self.marker_ob.header.stamp = laser_data.header.stamp
        self.marker_ob.type = self.marker_ob.POINTS
        self.marker_ob.action = self.marker_ob.ADD

        self.marker_ob.points = ob_Points
        # print('len(ob_Points): %d' % (len(ob_Points)))
        self.marker_ob.scale.x = 0.1
        self.marker_ob.scale.y = 0.1
        self.marker_ob.scale.z = 0.1
        self.marker_ob.color.a = 1.0
        self.marker_ob.color.r = 1.0

        self.marker_ob_pub.publish(self.marker_ob)


    def pathCallback(self, msg):
        if not self.received_global_path:
            self.received_global_path = True

            time.sleep(0.1)
            self.path = msg
            self.planner_thread = Thread(target=self.planThreadFunc)
            self.initPlanning()
            self.planner_thread.start()


    def initPlanning(self):
        self.goal_index = 0
        self.updateGlobalPose()
        self.state = np.array([0.0, 0.0, 0.0, self.vx, self.vw])


    def planThreadFunc(self):
        print("--- Running local planner thread ---")

        rr = rospy.Rate(10)
        self.need_exit = False
        while not self.need_exit:
            self.planOnce()

            if self.goal_dis < self.arrive:
                print("Arrived!")
                self.need_exit = True
                break

            rr.sleep()

        print("--- Exit local planner thread ---")
        self.publishVel(need_stop=True)
        self.planner_thread = None


    def planOnce(self):
        self.updateGlobalPose()

        # Update [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
        self.state = [0.0, 0.0, 0.0, self.vx, self.vw]

        # Update obstacle
        self.updateObstacle()

        # =========================================
        u, traj = self.dwa.plan(self.state, self.plan_goal, self.plan_ob)
        # u[0]: vx
        # u[1]: vw
        # traj: predicted trajectory
        # =========================================

        ########################################################
        # plot predicted trajectory
        traj_x = traj[:, 0]
        traj_y = traj[:, 1]

        # if len(traj_x) == 1:  # 0:
        #     print(Fore.RED + Back.GREEN + 'No pred traj')
        #     print('u[0]: %.2f' % (u[0]))
        #     print('u[1]: %.2f' % (u[1]))
        #     print(Style.RESET_ALL)
        # else:
        #     print(Fore.GREEN + str(len(traj_x)))
        #     print(Style.RESET_ALL)

        traj_Points = []
        for idx in range(len(traj_x)):
            p = Point()
            p.x = traj_x[idx]
            p.y = traj_y[idx]
            p.z = 0.3
            traj_Points.append(p)

        # marker of traj
        self.marker_traj.header.frame_id = 'base_link'
        self.marker_traj.header.stamp = rospy.get_rostime()
        self.marker_traj.type = self.marker_traj.POINTS
        self.marker_traj.action = self.marker_traj.ADD

        self.marker_traj.points = traj_Points
        self.marker_traj.scale.x = 0.05
        self.marker_traj.scale.y = 0.05
        self.marker_traj.scale.z = 0.05
        self.marker_traj.color.a = 1.0
        self.marker_traj.color.b = 1.0

        self.marker_traj_pub.publish(self.marker_traj)
        ########################################################


        alpha = 0.9  # 0.8  #0.5
        self.vx = u[0] * alpha + self.vx * (1.0 - alpha)
        self.vw = u[1] * alpha + self.vw * (1.0 - alpha)

        # print('------')
        # print('time: %d' % (rospy.get_rostime().secs))
        # print('vx: %.2f' % (self.vx))
        # print('vw: %.2f' % (self.vw))

        self.publishVel()


    def publishVel(self, need_stop=False):
        if need_stop:
            print('need_stop')
            self.vx = 0
            self.vw = 0

        self.gazebo_sim.pub_cmd_vel([self.vx, self.vw])


def main():
    print('')
    print('--- Local planner ---')

    rospy.init_node('my_local_planner')
    localPlanner = LocalPlanner()
    rospy.spin()


if __name__ == '__main__':
    main()
