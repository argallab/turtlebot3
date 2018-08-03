#!/usr/bin/python

import sys
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Quaternion
from turtlebot3_dynamic_obstacle.srv import *

class Turtlebot3Trajectory():

    def __init__(self):

        rospy.Service("/turtlebot3_traj_service", Trajectory, self.move_turtlebot_server)

        # Publisher
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

        self.twist = Twist()
        self.output = String()

    def move_turtlebot_server(self, req):

        if req.goal.x == 0:
            i = 0
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.cmd_pub.publish(self.twist)
            self.output.data = "stopped"
            rospy.logerr("STOPPED")

        elif req.goal.x == 1:
            self.twist.linear.x = 0.1
            self.twist.angular.z = 0.0
            self.cmd_pub.publish(self.twist)
            rospy.loginfo("MOVING")
            self.output.data = "moved"

        return TrajectoryResponse(self.output)

def main():
    s = Turtlebot3Trajectory()
    r = rospy.Rate(25)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node("turtlebot3_traj_service")
    try:
        main()
    except rospy.ROSInterruptException:
        pass
