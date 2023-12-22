#!/usr/bin/env python

import rospy
import tf2_ros
import geometry_msgs.msg

from my_gazebo_simulation import MyGazeboSimulation


class MyTfBroadcaster:
    def __init__(self):
        self.br = tf2_ros.TransformBroadcaster()
        self.gazebo_sim = MyGazeboSimulation()
        self.robot_frame = 'base_link'
        self.world_frame = 'map'

        self.broadcast()


    def broadcast(self):
        rate = rospy.Rate(50) # Hz
        
        while not rospy.is_shutdown():
            pos  = self.gazebo_sim.get_model_state().pose.position     # Point()
            ori  = self.gazebo_sim.get_model_state().pose.orientation  # Quaternion()
            time = self.gazebo_sim.get_model_state().header.stamp      # time
            
            t = geometry_msgs.msg.TransformStamped()

            t.header.stamp = time
            t.header.frame_id = self.world_frame
            t.child_frame_id  = self.robot_frame

            t.transform.translation.x = pos.x
            t.transform.translation.y = pos.y
            t.transform.translation.z = 0.0

            t.transform.rotation.x = ori.x
            t.transform.rotation.y = ori.y
            t.transform.rotation.z = ori.z
            t.transform.rotation.w = ori.w

            self.br.sendTransform(t)

            rate.sleep()


def main():
    rospy.init_node('robot_tf2_broadcaster')
    myTfBroadcaster = MyTfBroadcaster()
    rospy.spin()


if __name__ == '__main__':
    main()
    