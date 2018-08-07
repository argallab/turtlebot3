#ifndef TURTLEBOT3_ACTION_SERVER_H_
#define TURTLEBOT3_ACTION_SERVER_H_

// libraries
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib_tutorials/FibonacciAction.h>

class FibonacciActionServer
{
public:
  FibonacciActionServer();

  ~FibonacciActionServer(void) {
    }

    // Action Interface
    void executeCB(const actionlib::SimpleActionServer<actionlib_tutorials::FibonacciAction>::GoalConstPtr& goal);

private:
  ros::NodeHandle nh_;

  // ACTION server
  actionlib::SimpleActionServer<actionlib_tutorials::FibonacciAction> as_;

  actionlib_tutorials::FibonacciGoal goal_;
  actionlib_tutorials::FibonacciResult result_;
  actionlib_tutorials::FibonacciFeedback feedback_;

};

#endif
