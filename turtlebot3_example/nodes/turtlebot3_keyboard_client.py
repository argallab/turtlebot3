#!/usr/bin/env python
# Authors: Mahdieh Nejati

from __future__ import print_function
import rospy
import actionlib
import turtlebot3_example.msg
import sys
from std_msgs.msg import String

class Client():
    def __init__(self):

        # Subscribers
        self.key_sub = rospy.Subscriber('key', String, self.get_key)
        rospy.loginfo("wait for server")
        self.client_ = actionlib.SimpleActionClient('turtlebot3', turtlebot3_example.msg.Turtlebot3Action)
        self.client_.wait_for_server()

    def get_key(self, msg):

        mode = msg.data
        # TO DO
        area = 0.01
        count = 1
        if mode == 'z':
            self.client_.cancel_all_goals()
            rospy.loginfo("cancelling goals")

        else:
            if mode == 's':
                self.client(1, area, count)
            elif mode == 'x':
                traj_mode = 4
                self.shutdown()
            else:
                rospy.loginfo("you select wrong mode")






    def client(self, mode, area, count):

        goal = turtlebot3_example.msg.Turtlebot3Goal()
        goal.goal.x = mode
        goal.goal.y = area
        goal.goal.z = count
        self.client_.send_goal(goal)
        rospy.loginfo("send to goal")
        self.client_.wait_for_result()
        rospy.loginfo(self.client_.get_result())

    def shutdown(self):
        rospy.sleep(1)

if __name__ == '__main__':
    rospy.init_node('turtlebot3_client')
    result = Client()
    rospy.spin()
