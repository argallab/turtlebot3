#ifndef TURTLEBOT3_ACTION_SERVER_H_
#define TURTLEBOT3_ACTION_SERVER_H_

// libraries
#include  <ros/ros.h>
#include  <actionlib/server/simple_action_server.h>
#include  <turtlebot3_dynamic_obstacle/Turtlebot3Action.h>
#include  <geometry_msgs/Twist.h>
#include  <geometry_msgs/Point.h>
#include  <geometry_msgs/Quaternion.h>
#include  <geometry_msgs/Vector3.h>
#include  <nav_msgs/Odometry.h>
#include  <sensor_msgs/JointState.h>
#include  <turtlebot3_msgs/SensorState.h>
#include  <tf/transform_listener.h>
#include  <tf/transform_datatypes.h>
#include  <cmath>
#include  <math.h>
#include  <algorithm>


// #include  <Matrix3x3.h>

class Turtlebot3ActionServer
{
public:
  Turtlebot3ActionServer();

  ~Turtlebot3ActionServer(void) {
    }

    // Action Interface
    void executeCB(const actionlib::SimpleActionServer<turtlebot3_dynamic_obstacle::Turtlebot3Action>::GoalConstPtr& goal);

private:
  ros::NodeHandle nh_;

  // Variables
  bool success_;
  geometry_msgs::Point position_, rotation_;
  geometry_msgs::Twist twist_;

  tf::TransformListener listener_;
  tf::StampedTransform transform_;
  tf::Vector3 trans_;
  tf::Quaternion rot_;

  // Publishers
  ros::Publisher cmd_pub_;

  // ACTION server
  actionlib::SimpleActionServer<turtlebot3_dynamic_obstacle::Turtlebot3Action> as_;

  turtlebot3_dynamic_obstacle::Turtlebot3Goal goal_;
  turtlebot3_dynamic_obstacle::Turtlebot3Result result_;
  turtlebot3_dynamic_obstacle::Turtlebot3Feedback feedback_;

  // MEMBER METHODS
  void initializeSubscribers();
  void initializePublishers();

  // geometry_msgs::Point getOdom();
  void getOdom();
  void clearVelocities();
  bool checkPreempt();
  double getRadian(double angle);
  double wrapAngle(double angle);
  void move(double x, double y, double angle);
};

#endif
