#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import Point, PoseStamped
from visualization_msgs.msg import Marker

import time

# ========================================================
from use_rrt_star import Use_RRTStar as Planner
# ========================================================

# from (ws)/src/scripts/run.py
INIT_POSITION = [-2, 3, 1.57]  # in world frame
GOAL_POSITION = [0, 10]  # relative to the initial position


class GlobalPlanner:
    def __init__(self):
        # start position (world frame)
        self.plan_sx = INIT_POSITION[0]
        self.plan_sy = INIT_POSITION[1]
        # goal position (world frame)
        self.plan_gx = INIT_POSITION[0] + GOAL_POSITION[0]
        self.plan_gy = INIT_POSITION[1] + GOAL_POSITION[1]

        print('start position: (%f, %f)' % (self.plan_sx, self.plan_sy))
        print('goal position: (%f, %f)' % (self.plan_gx, self.plan_gy))

        # ==============================================
        # tunable params
        self.plan_grid_size = 0.2
        self.plan_robot_radius = 0.6  # useless
        self.ob_size = 0.4
        # ==============================================

        # obstacles
        self.plan_ox = []  
        self.plan_oy = []
        # returned way points
        self.plan_rx = []
        self.plan_ry = []

        self.path = Path()
        self.marker = Marker()

        self.map_sub    = rospy.Subscriber('/map', OccupancyGrid, self.mapCallback)
        self.path_pub   = rospy.Publisher('/my_planner/global_path', Path, queue_size=10)
        self.marker_pub = rospy.Publisher("/my_planner/waypoints", Marker, queue_size=10)


    def mapCallback(self, msg):
        # print('--- Map callback ---')
        self.map = msg
        self.replan()


    # def replan(self, req):
    def replan(self):
        # print('--- Got request for replan ---')
        self.initPlanner()

        t0 = time.time()
        #########################################
        self.plan_rx, self.plan_ry = self.planner.plan(self.plan_sx, self.plan_sy, self.plan_gx, self.plan_gy)
        #########################################
        t1 = time.time()
        print('Global planner run time = %f' % (t1 - t0))
        print('')

        self.getAndPubPath()
        

    def initPlanner(self):
        # print('--- Init global planner ---')
        print('self.map.info.height')
        print(self.map.info.height)
        print('self.map.info.width')
        print(self.map.info.width)

        for width in range(self.map.info.width):
            for height in range(self.map.info.height):
                value = self.map.data[height * self.map.info.width + width]
                if value == 100:
                    # obstacle
                    ox = width  * self.map.info.resolution + 0.5 * self.map.info.resolution + self.map.info.origin.position.x
                    oy = height * self.map.info.resolution + 0.5 * self.map.info.resolution + self.map.info.origin.position.y
                    self.plan_ox.append(ox)
                    self.plan_oy.append(oy)

        #########################################
        self.planner = Planner(self.plan_ox, self.plan_oy, self.plan_grid_size, self.plan_robot_radius, self.ob_size)
        #########################################


    def getAndPubPath(self):
        # print('--- Getting global path ---')
        self.path.header.seq = 0
        self.path.header.stamp = rospy.get_rostime()
        self.path.header.frame_id = 'map'

        wayPoints = []
        for i in range(len(self.plan_rx)):

            # remove all the way points except the final goal
            # if i < len(self.plan_rx) - 1:
            #     continue

            # path
            pose = PoseStamped()
            pose.header.seq = i
            pose.header.stamp = rospy.get_rostime()
            pose.header.frame_id = 'map'
            pose.pose.position.x = self.plan_rx[i]  ############
            pose.pose.position.y = self.plan_ry[i]  ############
            pose.pose.position.z = 0.01
            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 0
            pose.pose.orientation.z = 0
            pose.pose.orientation.w = 1

            self.path.poses.append(pose)

            # way points marker
            p = Point()
            p.x = self.plan_rx[i]
            p.y = self.plan_ry[i]
            p.z = 0
            wayPoints.append(p) 

        # marker of way points
        self.marker.header.frame_id = 'map'
        self.marker.type = self.marker.POINTS
        self.marker.action = self.marker.ADD

        self.marker.points = wayPoints
        self.marker.scale.x = 0.2
        self.marker.scale.y = 0.2
        self.marker.scale.z = 0.2
        self.marker.color.a = 1.0
        self.marker.color.g = 1.0
    

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
