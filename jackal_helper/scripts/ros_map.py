#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np


map_pub = rospy.Publisher('/map_out', OccupancyGrid, queue_size = 10)
marker_pub = rospy.Publisher("/obstacle_marker", Marker, queue_size = 10)


# def mapCallback(msg):
#     map_in = msg
#     map_in_data = np.array(map_in.data).reshape((-1, map_in.info.height)).transpose()
#     print('map_in_data.shape')
#     print(map_in_data.shape)

#     map_out = map_in

#     data = list(map_out.data)

#     # 0: obstacle (white)
#     # 100: free (black)
#     for i in range(len(data)):
#         data[i] = 100 - data[i]
#         # if data[i] == 100:
#         #     data[i] = 50
#         # elif data[i] == 0:
#         #     data[i] = 100

#     map_out.data = tuple(data)

#     rate = rospy.Rate(1) # Hz
#     while not rospy.is_shutdown():
#         map_pub.publish(map_out)
#         rate.sleep()


def mapCallback(msg):
    map = msg
    # map_data = np.array(map.data).reshape((-1, map.info.height)).transpose()
    # print('map_data.shape')
    # print(map_data.shape)

    print('self.map.info.height')
    print(map.info.height)
    print('self.map.info.width')
    print(map.info.width)

    # 0: obstacle (white)
    # 100: free (black)
    # ox, oy = np.nonzero(map_data < 50)
    # oy, ox = np.nonzero(map_data - 50)
    # ox = []
    # oy = []
    plan_ox = []
    plan_oy = []

    # for height in range(map_data.shape[0]):
    #     for width in range(map_data.shape[1]):
    #         if map_data[height][width] == 0:
    #             # ox.append(width)
    #             # oy.append(height)
    #             plan_ox.append(width * map.info.resolution + map.info.origin.position.x)
    #             plan_oy.append(height * map.info.resolution + map.info.origin.position.y)

    for width in range(map.info.width):
        for height in range(map.info.height):
            value = map.data[height * map.info.width + width]
            if value == 0:
                ox = width * map.info.resolution + 0.5 * map.info.resolution + map.info.origin.position.x
                oy = height * map.info.resolution + 0.5 * map.info.resolution + map.info.origin.position.y
                plan_ox.append(ox)
                plan_oy.append(oy)


    # plan_ox = (ox * map.info.resolution + map.info.origin.position.x).tolist()
    # plan_ox = ox * map.info.resolution + map.info.origin.position.x
    # plan_ox = (ox * map.info.resolution + map.info.origin.position.x).tolist()
    # plan_oy = oy * map.info.resolution + map.info.origin.position.y
    
    print('len(plan_ox)')
    print(len(plan_ox))

    triplePoints = []
    for i in range(len(plan_ox)):
        # print('Obstacle No. %d: (%f, %f)' % (i, plan_ox[i], plan_oy[i]))
        p = Point()
        p.x = plan_ox[i]
        p.y = plan_oy[i]
        p.z = 0
        triplePoints.append(p)

    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        marker = Marker()
        marker.header.frame_id = "/map"

        marker.type = marker.POINTS
        marker.action = marker.ADD
        marker.pose.orientation.w = 1

        marker.points = triplePoints
        t = rospy.Duration()
        marker.lifetime = t
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0

        marker_pub.publish(marker)
        rate.sleep()



def listener():
    rospy.init_node('listener', anonymous = True)
    rospy.Subscriber('/map', OccupancyGrid, mapCallback)
    rospy.spin()


if __name__ == '__main__':
    listener()

