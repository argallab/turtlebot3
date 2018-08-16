#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from math import radians, copysign, sqrt, pow, pi, atan2
from tf.transformations import euler_from_quaternion
import numpy as np
from std_msgs.msg import String

class GotoPoint():
    def __init__(self):
        rospy.init_node('turtlebot3_pointop_key', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher('cmd_vel_waffle', Twist, queue_size=5)

        self.tf_listener = tf.TransformListener()
        self.odom_frame = 'odom'

        self.r = rospy.Rate(40)

        try:
            self.tf_listener.waitForTransform(self.odom_frame, 'base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = 'base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, 'base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = 'base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between odom and base_link or base_footprint")
                rospy.signal_shutdown("tf Exception")

    def move(self, goal_x, goal_y, goal_z):
        position = Point()
        move_cmd = Twist()

        (position, rotation) = self.get_odom()
        last_rotation = 0
        linear_speed = 1
        angular_speed = 0.5
        if goal_z > 180 or goal_z < -180:
            print("you input worng z range.")
            self.shutdown()
        goal_z = np.deg2rad(goal_z)
        goal_distance = sqrt(pow(goal_x - position.x, 2) + pow(goal_y - position.y, 2))
        distance = goal_distance

        while distance > 0.05:
            (position, rotation) = self.get_odom()
            x_start = position.x
            y_start = position.y
            path_angle = atan2(goal_y - y_start, goal_x- x_start)

            if path_angle < -pi/4 or path_angle > pi/4:
                if goal_y < 0 and y_start < goal_y:
                    path_angle = -2*pi + path_angle
                    # rospy.loginfo("path angle  1 %f", path_angle)
                elif goal_y >= 0 and y_start > goal_y:
                    path_angle = 2*pi + path_angle
                    # rospy.loginfo("path angle  1 %f", path_angle)
            if last_rotation > pi-0.1 and rotation <= 0:
                rotation = 2*pi + rotation
                # rospy.loginfo("rotation 1 %f", rotation)
            elif last_rotation < -pi+0.1 and rotation > 0:
                rotation = -2*pi + rotation
                # rospy.loginfo("rotation 2 %f", rotation)
            move_cmd.angular.z = angular_speed * path_angle-rotation

            distance = sqrt(pow((goal_x - x_start), 2) + pow((goal_y - y_start), 2))
            move_cmd.linear.x = min(linear_speed * distance, 0.1)

            if move_cmd.angular.z > 0:
                move_cmd.angular.z = min(move_cmd.angular.z, .5)
            else:
                move_cmd.angular.z = max(move_cmd.angular.z, -.5)

            last_rotation = rotation
            self.cmd_vel.publish(move_cmd)
            self.r.sleep()
        (position, rotation) = self.get_odom()

        while abs(rotation - goal_z) > 0.05:
            (position, rotation) = self.get_odom()
            if goal_z >= 0:
                if rotation <= goal_z and rotation >= goal_z - pi:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = 0.5
                else:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = -0.5
            else:
                if rotation <= goal_z + pi and rotation > goal_z:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = -0.5
                else:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = 0.5
            self.cmd_vel.publish(move_cmd)
            self.r.sleep()
        rospy.loginfo("Stopping the robot...")
        move_cmd.linear.x = 0.00
        move_cmd.angular.z = 0.00
        self.cmd_vel.publish(move_cmd)
        self.cmd_vel.publish(Twist())

    def getkey(self, msg):
        if msg.data == '1':
            for i in range(10):
                self.move(0.5, 0, -180)
                self.r.sleep()
                self.move(0, 0, 0)
                self.r.sleep()

        elif msg.data == '2':
            self.move(0.5, 0, 180)
            self.r.sleep()
        elif msg.data == '3':
            self.move(0, 0, 0)
            self.r.sleep()

    def get_odom(self):
        got_msg = False
        while got_msg == False:
            rospy.loginfo("in loop")
            try:
                (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
                rotation = euler_from_quaternion(rot)
                rospy.loginfo("waiting for odom message, rotation: %f", rotation[2])
                got_msg = True

            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("TF Exception")
                # return
        return (Point(*trans), rotation[2])


    def shutdown(self):
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            gotopoint = GotoPoint()
            rospy.Subscriber('traj_selection', String, gotopoint.getkey)
            rospy.spin()

    except:
        rospy.loginfo("shutdown program.")
