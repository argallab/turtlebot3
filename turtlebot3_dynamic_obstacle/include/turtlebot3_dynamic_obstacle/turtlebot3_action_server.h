#ifndef TURTLEBOT3_ACTION_SERVER_H_
#define TURTLEBOT3_ACTION_SERVER_H_

// libraries
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <turtlebot3_dynamic_obstacle/Turtlebot3Action.h>
#include  <geometry_msgs/Twist.h>
#include  <geometry_msgs/Point.h>
#include  <geometry_msgs/Quaternion.h>
#include  <nav_msgs/Odometry.h>
#include  <sensor_msgs/JointState.h>
#include  <turtlebot3_msgs/SensorState.h>

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
  bool init_state_, success_;
  double right_encoder_, init_right_encoder_;
  geometry_msgs::Point position_, start_position_;
  geometry_msgs::Twist twist_;

  // SUBSCRIBERS
  ros::Subscriber state_sub_;
  ros::Subscriber odom_sub_;

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

  void getOdom(const nav_msgs::Odometry::ConstPtr& odom);
  void getState(const sensor_msgs::JointState::ConstPtr& state);
  void turn(float angle);
  void goForward(double length, int count);
  void clearVelocities();
  bool checkPreempt();
  bool getChangeInPosition(double length, int mode); 


};

#endif
