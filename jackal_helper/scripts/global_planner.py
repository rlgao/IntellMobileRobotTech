#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
# import tf
# from nav_msgs.srv import GetMap
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

# from course_agv_nav.srv import Plan,PlanResponse
# from jackal_helper.srv import Plan, PlanResponse

from nav_msgs.msg import OccupancyGrid
# from std_msgs.msg import Bool
# import numpy as np
# import matplotlib.pyplot as plt
# import pandas as pd
import time


# ========================================================
# from a_star import AStarPlanner as Planner
# from rrt_old_self_written import RRT as Planner
# from use_rrt import Use_RRT as Planner
from use_rrt_star import Use_RRTStar as Planner
# ========================================================


# from (ws)/src/scripts/run.py
INIT_POSITION = [-2, 3, 1.57]  # in world frame
GOAL_POSITION = [0, 10]  # relative to the initial position


# ROBOT_TF_NAME = "/base_link"  # "/robot_base"
MAP_TOPIC_NAME = "/map"  # "/map"


class GlobalPlanner:
    def __init__(self):
        # start position
        self.plan_sx = INIT_POSITION[0]
        self.plan_sy = INIT_POSITION[1]
        # goal position
        self.plan_gx = INIT_POSITION[0] + GOAL_POSITION[0]
        self.plan_gy = INIT_POSITION[1] + GOAL_POSITION[1]

        print('start position: (%f, %f)' % (self.plan_sx, self.plan_sy))
        print('goal position: (%f, %f)' % (self.plan_gx, self.plan_gy))

        self.plan_grid_size = 0.05
        self.plan_robot_radius = 0.6

        # obstacles
        self.plan_ox = []  
        self.plan_oy = []
        # returned path points
        self.plan_rx = []
        self.plan_ry = []

        self.path = Path()
        self.marker = Marker()

        # count to update map
        # self.map_count = 0

        # self.tf = tf.TransformListener()

        # self.map = OccupancyGrid()

        # self.goal_sub = rospy.Subscriber('/my_planner/goal', PoseStamped, self.goalCallback)
        # self.plan_srv = rospy.Service('/my_planner/global_plan', Plan, self.replan)


        # ==============================================
        self.map_sub = rospy.Subscriber(MAP_TOPIC_NAME, OccupancyGrid, self.mapCallback)
        self.path_pub = rospy.Publisher('/my_planner/global_path', Path, queue_size=10)
        self.marker_pub = rospy.Publisher("/my_planner/waypoints", Marker, queue_size=10)
        # ==============================================

        
        # self.collision_sub = rospy.Subscriber('/collision_checker_result', Bool, self.collisionCallback)

        # self.updateMap()
        # self.updateGlobalPose()

        # self.goalCallback()


    # def goalCallback(self, msg=None):
    #     self.plan_goal = msg
    #     print("-----get new goal!!!-----", self.plan_goal)

    #     # self.plan_gx = msg.pose.position.x
    #     # self.plan_gy = msg.pose.position.y
    #     self.plan_gx = -2.0 + 0.0
    #     self.plan_gy = 3.0  + 10.0

    #     self.replan()


    # def collisionCallback(self, msg):
    #     self.replan()


    # def updateGlobalPose(self):
    #     # try:
    #     #     self.tf.waitForTransform("/map", ROBOT_TF_NAME, rospy.Time(), rospy.Duration(4.0))
    #     #     (self.trans, self.rot) = self.tf.lookupTransform('/map','/robot_base',rospy.Time(0))
    #     # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    #     #     print("get tf error!")

    #     self.plan_sx = 0.0  # self.trans[0]  # x at present
    #     self.plan_sy = 0.0  # self.trans[1]  # y at present


    def mapCallback(self, msg):
        # print('--- Map callback ---')
        self.map = msg
        
        self.replan()


    # def replan(self, req):
    def replan(self):
        # print('--- Got request for replan ---')

        self.initPlanner()
        # self.updateGlobalPose()

        ################################################
        t0 = time.time()
        self.plan_rx, self.plan_ry = self.planner.plan(self.plan_sx, self.plan_sy, self.plan_gx, self.plan_gy)
        t1 = time.time()
        print('Global planner run time = ' + str(t1 - t0))
        print('')
        ################################################

        self.getAndPubPath()

        # res = True
        # return PlanResponse(res)
        

    def initPlanner(self):
        # print('--- Init global planner ---')
        print('self.map.info.height')
        print(self.map.info.height)
        print('self.map.info.width')
        print(self.map.info.width)

        # map_data = np.array(self.map.data).reshape((-1, self.map.info.height)).transpose()
        # print('map_data.shape')
        # print(map_data.shape)
        # # np.savetxt('/home/grl/map_data.txt', np.array(map_data,dtype=np.int))

        # # df = pd.DataFrame(map_data)
        # # filepath = '/home/grl/map_data.xlsx'
        # # df.to_excel(filepath, index=False)

        # # 0: obstacle (white)
        # # 100: free (black)
        # # ox, oy = np.nonzero(map_data > 50)
        # ox, oy = np.nonzero(map_data < 50)

        # self.plan_ox = (ox * self.map.info.resolution + self.map.info.origin.position.x).tolist()
        # self.plan_oy = (oy * self.map.info.resolution + self.map.info.origin.position.y).tolist()

        # # ----------------------
        # print('len(self.plan_ox)')
        # print(len(self.plan_ox))
        # print('len(self.plan_oy)')
        # print(len(self.plan_oy))

        # # for i in range(len(self.plan_ox)):
        # #     print('Obstacle No. %d: (%f, %f)' % (i, self.plan_ox[i], self.plan_oy[i]))
        # # ----------------------
        

        for width in range(self.map.info.width):
            for height in range(self.map.info.height):
                value = self.map.data[height * self.map.info.width + width]
                # if value == 0:
                if value == 100:
                    # obstacle
                    ox = width * self.map.info.resolution + 0.5 * self.map.info.resolution + self.map.info.origin.position.x
                    oy = height * self.map.info.resolution + 0.5 * self.map.info.resolution + self.map.info.origin.position.y
                    self.plan_ox.append(ox)
                    self.plan_oy.append(oy)

        ################################################
        self.planner = Planner(self.plan_ox, self.plan_oy, self.plan_grid_size, self.plan_robot_radius)
        ################################################


    def getAndPubPath(self):
        # print('--- Getting global path ---')

        self.path.header.seq = 0
        self.path.header.stamp = rospy.Time(0)
        self.path.header.frame_id = 'map'

        wayPoints = []
        for i in range(len(self.plan_rx)):
            # path
            pose = PoseStamped()
            pose.header.seq = i
            pose.header.stamp = rospy.Time(0)
            pose.header.frame_id = 'map'
            pose.pose.position.x = self.plan_rx[i]  ############
            pose.pose.position.y = self.plan_ry[i]  ############
            pose.pose.position.z = 0.01
            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 0
            pose.pose.orientation.z = 0
            pose.pose.orientation.w = 1

            self.path.poses.append(pose)

            # way points
            p = Point()
            p.x = self.plan_rx[i]
            p.y = self.plan_ry[i]
            p.z = 0
            wayPoints.append(p) 

        # marker of way points
        self.marker.header.frame_id = '/map'
        self.marker.type = self.marker.POINTS
        self.marker.action = self.marker.ADD
        # self.marker.pose.orientation.w = 1

        self.marker.points = wayPoints
        # t = rospy.Duration()
        # self.marker.lifetime = t
        self.marker.scale.x = 0.2
        self.marker.scale.y = 0.2
        self.marker.scale.z = 0.2
        self.marker.color.a = 1.0
        self.marker.color.r = 1.0
    

        rate = rospy.Rate(1) # Hz
        while not rospy.is_shutdown():
            self.publish()
            rate.sleep()


    def publish(self):
        self.path_pub.publish(self.path)
        self.marker_pub.publish(self.marker)


def main():
    print('')
    print('--- Global planner ---')

    rospy.init_node('my_global_planner', anonymous = True)
    globalPlanner = GlobalPlanner()
    rospy.spin()


if __name__ == '__main__':
    main()
