#!/usr/bin/env python

import sys
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
from turtlebot3_dynamic_obstacle.srv import *

class Turtlebot3Trajectory():

    def __init__(self):

        rospy.init_node('turtlebot3_traj_client', anonymous=True)
        rospy.Subscriber("/traj_selection", String, self.get_key, queue_size=1)

        # Call service to reset mico hardware to home
        self.turtlebot3_traj_service = rospy.ServiceProxy("/turtlebot3_traj_service", Trajectory)
        self.turtlebot3_traj_service.wait_for_service()

        # self.supported_traj = [1, 2, 3, 4, 5, 6, 7, 8, 9, 's']
        self.supported_traj = ['1', 's']

        rate = rospy.Rate(25)

        while not rospy.is_shutdown():
            rate.sleep()

    def get_key(self, msg):
        if msg.data in self.supported_traj:
            goal = Vector3()
            if msg.data == 's':
                goal.x = 0 # Shape
            else:
                goal.x = int(msg.data) # Shape
            goal.y = 0.01   # Length
            goal.z = 1        # Count
            self.call_trajectory_service(goal)
        else:
            rospy.loginfo("Sorry! Cannot handle the requested trajectory yet!")

    def call_trajectory_service(self, msg):
        status = self.turtlebot3_traj_service(msg)
        print "RESPONSE: %s"%status.state
        return status.state

def main():
    turtlebot3_trajectory_client = Turtlebot3Trajectory()

if __name__ == '__main__':
    sys.exit(main())
